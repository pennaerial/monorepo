from rclpy.node import Node

from payload_interfaces.msg import (
    DriveCommand,
    MotorState,
    PayloadHeartbeat,
    ServoCommand,
)
from payload_interfaces.srv import DeadReckon, TimedDrive

from .Vehicle import Vehicle

_DEFAULT_UDP_HEARTBEAT_HZ = 10.0


class Payload(Vehicle):
    """Mission-side adapter for one payload node namespace."""

    def __init__(
        self,
        node: Node,
        vehicle_name: str,
        udp_heartbeat_hz: float = _DEFAULT_UDP_HEARTBEAT_HZ,
    ):
        super().__init__(
            node,
            vehicle_name,
            has_camera=True,
            camera_namespace=f"/{vehicle_name}",
            image_topic="camera",
            camera_info_topic="camera_info",
            camera_service_name="camera_data",
        )

        self.drive_publisher = self.node.create_publisher(
            DriveCommand, self.namespaced_path("cmd_drive"), 10
        )
        self.servo_publisher = self.node.create_publisher(
            ServoCommand, self.namespaced_path("servo"), 10
        )
        self.timed_drive_client = self.node.create_client(
            TimedDrive, self.namespaced_path("timed_drive")
        )
        self.dead_reckon_client = self.node.create_client(
            DeadReckon, self.namespaced_path("dead_reckon")
        )

        # Mutable dict that modes write into to surface perception state in the
        # UDP heartbeat.  Supported keys:
        #   tag_visible (bool), tag_distance_m (float), tag_bearing_deg (float),
        #   is_error (bool)
        self.heartbeat_state: dict = {}

        self._udp_heartbeat_seq = 0
        self._latest_motor_state: MotorState | None = None

        raw = node._raw_node_api
        self._udp_heartbeat_pub = raw.create_publisher(
            PayloadHeartbeat, "udp_bridge/out/heartbeat", 10
        )
        raw.create_subscription(
            MotorState,
            self.namespaced_path("motor_state"),
            self._on_motor_state,
            1,
        )
        self._udp_heartbeat_timer = raw.create_timer(
            1.0 / float(udp_heartbeat_hz),
            self._publish_udp_heartbeat,
        )

    def _on_motor_state(self, msg: MotorState) -> None:
        self._latest_motor_state = msg

    def _publish_udp_heartbeat(self) -> None:
        msg = PayloadHeartbeat()

        now = self.node._now_seconds()
        msg.stamp.sec = int(now)
        msg.stamp.nanosec = int((now % 1.0) * 1e9)

        msg.vehicle_name = self.name
        msg.seq = self._udp_heartbeat_seq
        self._udp_heartbeat_seq += 1

        msg.active_mode = str(getattr(self.node, "active_mode", None) or "")
        msg.mission_started = getattr(self.node, "timer", None) is not None
        msg.is_error = bool(self.heartbeat_state.get("is_error", False))

        peer_status: dict[str, bool] = self.node._peer_connections.status()
        msg.connected_peers = [p for p, alive in peer_status.items() if alive]
        msg.disconnected_peers = [p for p, alive in peer_status.items() if not alive]

        motor = self._latest_motor_state
        if motor is not None:
            msg.drive_linear_mps = float(motor.linear_setpoint_mps)
            msg.drive_angular_radps = float(motor.angular_setpoint_rad_s)

        msg.tag_visible = bool(self.heartbeat_state.get("tag_visible", False))
        msg.tag_distance_m = float(self.heartbeat_state.get("tag_distance_m", 0.0))
        msg.tag_bearing_deg = float(self.heartbeat_state.get("tag_bearing_deg", 0.0))

        self._udp_heartbeat_pub.publish(msg)

    def drive(self, linear: float, angular: float) -> None:
        self.drive_publisher.publish(
            DriveCommand(linear=float(linear), angular=float(angular))
        )

    def stop(self) -> None:
        self.drive(0.0, 0.0)

    def set_servo(self, degree: float) -> None:
        self.servo_publisher.publish(ServoCommand(degree=float(degree)))

    def dead_reckon(self, linear: float, angular: float, speed: float, timeout_sec: float = 30.0):
        request = DeadReckon.Request()
        request.linear = float(linear)
        request.angular = float(angular)
        request.speed = float(speed)
        request.timeout_sec = float(timeout_sec)
        return self.dead_reckon_client.call_async(request)

    def timed_drive(self, linear: float, angular: float, duration_sec: float):
        request = TimedDrive.Request()
        request.linear = float(linear)
        request.angular = float(angular)
        request.duration_sec = float(duration_sec)
        return self.timed_drive_client.call_async(request)
