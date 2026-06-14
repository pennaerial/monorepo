"""
PayloadPeerFleetTestMode: passive two-payload peer communication example.

This mode is intended for manual fleet testing of the peer-aware `ModeManager`
plumbing. Each payload:

- publishes a local status payload on `peer_test/state`
- subscribes to the other payload's namespaced `peer_test/state`
- publishes and subscribes on `/shared/peer_test/broadcast`

The mode never transitions on its own. It is meant to stay active while the
operator verifies peer discovery, disconnect handling, and reconnect behavior.
"""

from __future__ import annotations

import json
from typing import Mapping

from rclpy.node import Node
from std_msgs.msg import String

from payload.payload import Payload

from vehicle_common.mode import Mode
from vehicle_common.runtime.plugin_loader import register_plugin


@register_plugin(name="payload.PayloadPeerFleetTestMode", base_cls=Mode)
class PayloadPeerFleetTestMode(Mode):
    """Passive multi-payload communication demo for fleet bring-up."""

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
        self._peer_message_counts = {peer_name: 0 for peer_name in self._peer_names}
        self._shared_remote_message_counts = {
            peer_name: 0 for peer_name in self._peer_names
        }
        self._last_disconnect_signature: tuple[str, ...] = ()

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _decode_status(self, message: String) -> dict[str, object] | None:
        try:
            payload = json.loads(str(message.data))
        except json.JSONDecodeError:
            return None
        if not isinstance(payload, dict):
            return None
        return payload

    def _on_peer_message(self, peer_name: str, message: String) -> None:
        payload = self._decode_status(message)
        if payload is None:
            return
        self._peer_message_counts[peer_name] = (
            self._peer_message_counts.get(peer_name, 0) + 1
        )

    def _on_shared_message(self, message: String) -> None:
        self._shared_message_count += 1
        payload = self._decode_status(message)
        if payload is None:
            return
        sender = str(payload.get("vehicle", "")).strip()
        if sender and sender != self.vehicle.name and sender in self._peer_names:
            self._shared_remote_message_counts[sender] = (
                self._shared_remote_message_counts.get(sender, 0) + 1
            )

    def _status_payload(
        self, *, state: str, disconnected_peers: tuple[str, ...]
    ) -> dict[str, object]:
        return {
            "vehicle": self.vehicle.name,
            "sequence": self._sequence,
            "state": state,
            "peer_names": list(self._peer_names),
            "disconnected_peers": list(disconnected_peers),
            "peer_received_total": int(sum(self._peer_message_counts.values())),
            "peer_received_by_peer": dict(sorted(self._peer_message_counts.items())),
            "shared_received_total": int(self._shared_message_count),
            "shared_remote_received_total": int(
                sum(self._shared_remote_message_counts.values())
            ),
            "shared_remote_received_by_peer": dict(
                sorted(self._shared_remote_message_counts.items())
            ),
        }

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
        status = json.dumps(
            self._status_payload(state=state, disconnected_peers=disconnected_peers),
            sort_keys=True,
        )

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
        self.log(
            f"{state}: peer_total={sum(self._peer_message_counts.values())} "
            f"shared_total={self._shared_message_count} "
            f"shared_remote_total={sum(self._shared_remote_message_counts.values())}"
        )

    def on_enter(self) -> None:
        self._last_publish_time = 0.0
        self._last_log_time = 0.0
        self._sequence = 0
        self._shared_message_count = 0
        for peer_name in self._peer_names:
            self._peer_message_counts[peer_name] = 0
            self._shared_remote_message_counts[peer_name] = 0
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
        self, time_delta: float, connection_status: Mapping[str, bool]
    ) -> None:
        del time_delta
        now = self._now()
        disconnected_peers = tuple(
            peer_name
            for peer_name, is_connected in sorted(connection_status.items())
            if not bool(is_connected)
        )
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
