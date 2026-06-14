"""PayloadIdleMode — does nothing. Used to keep the payload stack alive for
infra-level testing (UDP bridge, heartbeats, sensors) without running any
mission logic."""

from rclpy.node import Node

from payload.payload import Payload
from vehicle_common.mode import Mode
from vehicle_common.runtime.plugin_loader import register_plugin

_FOREVER = 0.0


@register_plugin(name="payload.PayloadIdleMode", base_cls=Mode)
class PayloadIdleMode(Mode):
    mission_target = "payload"
    required_vision_nodes = ()
    requires_camera = False
    transition_labels = ("complete",)

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        wait_seconds: float = _FOREVER,
    ) -> None:
        super().__init__(node, vehicle)
        self._wait_seconds = float(wait_seconds)
        self._elapsed = 0.0

    def on_enter(self) -> None:
        self._elapsed = 0.0
        if self._wait_seconds > 0:
            self.log(
                f"idle for {self._wait_seconds}s — UDP bridge and heartbeat running"
            )
        else:
            self.log("idle indefinitely — UDP bridge and heartbeat running")

    def on_update(self, time_delta: float) -> None:
        self._elapsed += time_delta

    def check_status(self) -> str:
        if self._wait_seconds > 0 and self._elapsed >= self._wait_seconds:
            return "complete"
        return "continue"
