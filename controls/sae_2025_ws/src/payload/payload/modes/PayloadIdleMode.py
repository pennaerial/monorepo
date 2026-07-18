"""PayloadIdleMode — does nothing. Used to keep the payload stack alive for
infra-level testing (UDP bridge, heartbeats, sensors) without running any
mission logic."""

from typing import override

from pydantic import BaseModel
from rclpy.node import Node

from payload.payload import Payload
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import register_mode

_FOREVER = 0.0


class PayloadIdleParams(BaseModel):
    wait_seconds: float = _FOREVER


@register_mode(
    id="payload.PayloadIdleMode", targets=[Payload], transition_labels=["complete"]
)
class PayloadIdleMode(Mode):
    mission_target = "payload"
    required_vision_nodes = ()
    requires_camera = False
    transition_labels = ("complete",)

    @override
    def initialize(
        self, node: Node, vehicle: Payload, params: PayloadIdleParams
    ) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params
        self._elapsed = 0.0

    def on_enter(self) -> None:
        self._elapsed = 0.0
        if self.p.wait_seconds > 0:
            self.log(
                f"idle for {self.p.wait_seconds}s — UDP bridge and heartbeat running"
            )
        else:
            self.log("idle indefinitely — UDP bridge and heartbeat running")

    def on_update(self, time_delta: float) -> None:
        self._elapsed += time_delta

    def check_status(self) -> str:
        if self.p.wait_seconds > 0 and self._elapsed >= self.p.wait_seconds:
            return "complete"
        return "continue"

    @classmethod
    @override
    def get_params_cls(cls) -> type[BaseModel]:
        return PayloadIdleParams
