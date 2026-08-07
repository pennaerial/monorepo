"""UDP bridge node.

Each node listens on its own port and sends to all peer ports on a shared
broadcast address.  Topics to forward are declared at launch time; the bridge
serializes any ROS message type transparently using rclpy.serialization.

Peer liveness
-------------
  Heartbeats are always broadcast to all configured peer ports so peers can
  discover this node.  Data packets are only sent to peers that are currently
  alive (i.e. have sent a heartbeat within peer_ttl seconds).  A peer is
  marked alive on the first heartbeat received and expires after peer_ttl
  seconds of silence.

Naming convention
-----------------
  udp_bridge/out/<topic>  — publish here to send a message to all peers
  udp_bridge/in/<topic>   — subscribe here to receive messages from peers

Packet wire format
------------------
  Data packet
    Byte 0     : 0x01
    Bytes 1-2  : sender port (big-endian uint16)
    Byte  3    : topic_id (uint8, index into configured topics list)
    Bytes 4+   : rclpy-serialized ROS message bytes

  Ping packet (liveness probe — not a ROS message)
    Byte 0     : 0x02
    Bytes 1-2  : sender port (big-endian uint16)
    Bytes 3-6  : sequence number (big-endian uint32)
"""

import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

_TYPE_DATA = 0x01
_TYPE_PING = 0x02
_DATA_HEADER = struct.Struct("!BHB")  # msg_type, sender_port, topic_id
_PING_HEADER = struct.Struct("!BHI")  # msg_type, sender_port, seq
_RECV_TIMEOUT_S = 1.0
_PING_INTERVAL_S = 1.0
_DEFAULT_PEER_TTL_S = 3.0  # 3× ping interval — tolerates 2 dropped pings


class BridgeNode(Node):
    def __init__(self):
        super().__init__("udp_bridge")

        self.declare_parameter("my_port", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("all_ports", rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("broadcast_ip", "10.42.0.255")
        self.declare_parameter("topics", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("peer_ttl", _DEFAULT_PEER_TTL_S)

        self._my_port: int = self.get_parameter("my_port").value
        all_ports: list[int] = list(self.get_parameter("all_ports").value)
        self._broadcast_ip: str = self.get_parameter("broadcast_ip").value
        self._peer_ports: list[int] = [p for p in all_ports if p != self._my_port]
        self._peer_ttl: float = float(self.get_parameter("peer_ttl").value)
        self._ping_seq = 0
        self._stop = threading.Event()

        # peer liveness: port → monotonic timestamp of last received heartbeat
        self._peer_last_seen: dict[int, float] = {}
        self._peer_lock = threading.Lock()

        # --- topic registry ---
        # _topic_types[i] = (topic_name, msg_class)
        self._topic_types: list[tuple[str, type]] = []
        self._in_publishers: list = []  # indexed by topic_id
        self._out_subscribers: list = []  # keep refs so they aren't GC'd

        _topics_raw = self.get_parameter("topics").value
        topics_param: list[str] = list(_topics_raw) if _topics_raw is not None else []
        for i, entry in enumerate(topics_param):
            if ":" not in entry:
                raise ValueError(f"topics entry {entry!r} must be 'topic_name:pkg/msg/MsgType'")
            topic_name, type_str = entry.split(":", 1)
            msg_class = get_message(type_str)
            self._topic_types.append((topic_name, msg_class))

            pub = self.create_publisher(msg_class, f"udp_bridge/in/{topic_name}", 10)
            self._in_publishers.append(pub)

            topic_id = i
            sub = self.create_subscription(
                msg_class,
                f"udp_bridge/out/{topic_name}",
                lambda msg, tid=topic_id: self._on_out(msg, tid),
                10,
            )
            self._out_subscribers.append(sub)

        self._debug_pub = self.create_publisher(String, "udp_bridge/debug", 100)

        self.get_logger().info(
            f"UDP bridge starting: my_port={self._my_port} "
            f"peers={self._peer_ports} broadcast={self._broadcast_ip} "
            f"peer_ttl={self._peer_ttl}s "
            f"topics={[e.split(':')[0] for e in topics_param]}"
        )

        # UDP socket — SO_BROADCAST required for subnet broadcast
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._sock.settimeout(_RECV_TIMEOUT_S)
        self._sock.bind(("0.0.0.0", self._my_port))

        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True, name="udp_recv")
        self._ping_thread = threading.Thread(target=self._ping_loop, daemon=True, name="udp_ping")
        self._recv_thread.start()
        self._ping_thread.start()

    def _debug(self, text: str) -> None:
        ns = self.get_namespace().strip("/")
        self._debug_pub.publish(String(data=f"[{ns}] {text}"))

    # ------------------------------------------------------------------
    # Peer liveness
    # ------------------------------------------------------------------

    def _active_peers(self) -> list[int]:
        """Return ports whose last heartbeat arrived within peer_ttl seconds."""
        now = time.monotonic()
        with self._peer_lock:
            return [
                port
                for port in self._peer_ports
                if now - self._peer_last_seen.get(port, 0.0) <= self._peer_ttl
            ]

    def _record_heartbeat(self, sender_port: int) -> None:
        was_dead = True
        with self._peer_lock:
            was_dead = sender_port not in self._peer_last_seen or (
                time.monotonic() - self._peer_last_seen.get(sender_port, 0.0) > self._peer_ttl
            )
            self._peer_last_seen[sender_port] = time.monotonic()
        if was_dead:
            self._debug(f"PEER connected: port {sender_port}")

    # ------------------------------------------------------------------
    # Outbound path: ROS → UDP
    # ------------------------------------------------------------------

    def _send_to_peers(self, packet: bytes, label: str, *, ports: list[int]) -> None:
        for port in ports:
            try:
                self._sock.sendto(packet, (self._broadcast_ip, port))
                self._debug(f"OUT {label} → {self._broadcast_ip}:{port}")
            except OSError as e:
                self._debug(f"OUT failed → {self._broadcast_ip}:{port} ({e})")

    def _on_out(self, msg, topic_id: int) -> None:
        active = self._active_peers()
        if not active:
            return  # no live peers — skip serialization entirely
        topic_name, _ = self._topic_types[topic_id]
        ros_bytes = serialize_message(msg)
        header = _DATA_HEADER.pack(_TYPE_DATA, self._my_port, topic_id)
        packet = header + ros_bytes
        self._debug(f"OUT data topic={topic_name!r} ({len(ros_bytes)}B payload)")
        self._send_to_peers(packet, f"data topic={topic_name!r}", ports=active)

    # ------------------------------------------------------------------
    # Inbound path: UDP → ROS
    # ------------------------------------------------------------------

    def _recv_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(65535)
            except TimeoutError:
                continue
            except OSError:
                continue

            if len(data) < _DATA_HEADER.size:
                self._debug(f"IN short packet ({len(data)}B) from {addr} — dropping")
                continue

            msg_type = data[0]

            if msg_type == _TYPE_DATA:
                self._handle_data(data, addr)
            elif msg_type == _TYPE_PING:
                self._handle_ping(data, addr)
            else:
                self._debug(f"IN unknown type 0x{msg_type:02x} from {addr} — dropping")

    def _handle_data(self, data: bytes, addr) -> None:
        if len(data) < _DATA_HEADER.size:
            return
        msg_type, sender_port, topic_id = _DATA_HEADER.unpack_from(data, 0)
        ros_bytes = data[_DATA_HEADER.size :]

        self._debug(
            f"IN data {len(data)}B from {addr[0]}:{sender_port} topic_id={topic_id} ros_payload={len(ros_bytes)}B"
        )

        if topic_id >= len(self._topic_types):
            self._debug(
                f"IN unknown topic_id={topic_id} (have {len(self._topic_types)}) — dropping"
            )
            return

        topic_name, msg_class = self._topic_types[topic_id]
        try:
            msg = deserialize_message(ros_bytes, msg_class)
        except Exception as e:
            self.get_logger().error(f"IN deserialize failed for topic={topic_name!r}: {e}")
            return

        self._in_publishers[topic_id].publish(msg)
        self._debug(f"IN data published → udp_bridge/in/{topic_name}")

    def _handle_ping(self, data: bytes, addr) -> None:
        if len(data) < _PING_HEADER.size:
            return
        _, sender_port, seq = _PING_HEADER.unpack_from(data, 0)
        self._record_heartbeat(sender_port)
        self._debug(f"IN ping from {addr[0]}:{sender_port} seq={seq} active={self._active_peers()}")

    # ------------------------------------------------------------------
    # Ping sender — always broadcasts to all ports (enables discovery)
    # ------------------------------------------------------------------

    def _ping_loop(self) -> None:
        while not self._stop.is_set():
            packet = _PING_HEADER.pack(_TYPE_PING, self._my_port, self._ping_seq)
            # Pings go to all configured ports regardless of liveness,
            # so peers can discover this node even after a reconnect.
            self._send_to_peers(
                packet,
                f"ping seq={self._ping_seq}",
                ports=self._peer_ports,
            )
            self._ping_seq += 1
            time.sleep(_PING_INTERVAL_S)

    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self._stop.set()
        self._recv_thread.join(timeout=2.0)
        self._ping_thread.join(timeout=2.0)
        self._sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
