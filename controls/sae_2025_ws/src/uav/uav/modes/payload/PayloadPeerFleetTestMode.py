"""
PayloadPeerFleetTestMode: passive two-payload peer communication example.

This mode is intended for manual fleet testing of the peer-aware `ModeManager`
plumbing. Each payload:

- publishes a local status string on `peer_test/state`
- subscribes to the other payload's namespaced `peer_test/state`
- publishes and subscribes on `/shared/peer_test/broadcast`

The mode never transitions on its own. It is meant to stay active while the
operator verifies peer discovery, disconnect handling, and reconnect behavior.
"""

from __future__ import annotations

from rclpy.node import Node
from std_msgs.msg import String

from uav.vehicles.Payload import Payload

from ..Mode import Mode


class PayloadPeerFleetTestMode(Mode):
    """Passive multi-payload communication demo for fleet bring-up."""

    mission_target = "payload"
    required_vision_nodes = ()
    peer_vehicle_names = ("payload_0", "payload_1")
    requires_camera = False
    transition_labels = ()

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        publish_period_s: float = 1.0,
        local_topic: str = "peer_test/state",
        shared_topic: str = "/shared/peer_test/broadcast",
    ) -> None:
        super().__init__(node, vehicle)
        self.vehicle: Payload = vehicle
        self.publish_period_s = float(publish_period_s)
        self._local_topic = vehicle.namespaced_path(local_topic)
        self._shared_topic = str(shared_topic).strip() or "/shared/peer_test/broadcast"
        self._peer_names = tuple(
            sorted(
                peer_name
                for peer_name in self.peer_vehicle_names
                if peer_name != vehicle.name
            )
        )

        self._local_publisher = self.node.create_publisher(
            String, self._local_topic, 10
        )
        self._shared_publisher = self.node.create_publisher(
            String, self._shared_topic, 10
        )
        self._shared_subscription = self.node.create_subscription(
            String,
            self._shared_topic,
            self._on_shared_message,
            10,
        )
        self._peer_subscriptions = {
            peer_name: self.node.create_subscription(
                String,
                vehicle.namespaced_path(local_topic, namespace=f"/{peer_name}"),
                lambda message, peer_name=peer_name: self._on_peer_message(
                    peer_name, message
                ),
                10,
            )
            for peer_name in self._peer_names
        }

        self._last_publish_time = 0.0
        self._last_log_time = 0.0
        self._sequence = 0
        self._shared_message_count = 0
        self._peer_messages: dict[str, str] = {}
        self._last_disconnect_signature: tuple[str, ...] = ()

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_peer_message(self, peer_name: str, message: String) -> None:
        self._peer_messages[peer_name] = str(message.data)

    def _on_shared_message(self, _message: String) -> None:
        self._shared_message_count += 1

    def _publish_status(
        self, *, now: float, state: str, disconnected_peers: tuple[str, ...] = ()
    ) -> None:
        if (
            self._last_publish_time
            and now - self._last_publish_time < self.publish_period_s
        ):
            return

        self._last_publish_time = now
        self._sequence += 1
        suffix = ""
        if disconnected_peers:
            suffix = f" missing={','.join(disconnected_peers)}"
        status = f"{self.vehicle.name} seq={self._sequence} state={state}{suffix}"

        local_message = String()
        local_message.data = status
        self._local_publisher.publish(local_message)

        shared_message = String()
        shared_message.data = status
        self._shared_publisher.publish(shared_message)

    def _log_status(self, *, now: float, state: str) -> None:
        if self._last_log_time and now - self._last_log_time < 5.0:
            return
        self._last_log_time = now
        peers = ", ".join(
            f"{peer_name}={self._peer_messages.get(peer_name, 'none')}"
            for peer_name in self._peer_names
        )
        self.log(
            f"{state}: shared_messages={self._shared_message_count} peer_messages=[{peers}]"
        )

    def on_enter(self) -> None:
        self._last_publish_time = 0.0
        self._last_log_time = 0.0
        self._sequence = 0
        self._shared_message_count = 0
        self._peer_messages.clear()
        self._last_disconnect_signature = ()
        self.log(
            f"starting peer fleet test for {self.vehicle.name}; expecting peers {self._peer_names}"
        )

    def on_update(self, time_delta: float) -> None:
        del time_delta
        now = self._now()
        self._publish_status(now=now, state="connected")
        self._log_status(now=now, state="connected")

    def on_disconnect(
        self, time_delta: float, disconnected_peers: tuple[str, ...]
    ) -> None:
        del time_delta
        now = self._now()
        disconnect_signature = tuple(sorted(disconnected_peers))
        if disconnect_signature != self._last_disconnect_signature:
            self.log("waiting for peers: " + ", ".join(disconnect_signature))
            self._last_disconnect_signature = disconnect_signature
        self._publish_status(
            now=now,
            state="waiting",
            disconnected_peers=disconnect_signature,
        )
        self._log_status(now=now, state="waiting")

    def check_status(self) -> str:
        return "continue"
