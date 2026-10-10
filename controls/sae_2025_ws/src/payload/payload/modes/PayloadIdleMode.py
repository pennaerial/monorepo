from typing import override

from rclpy.node import Node
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from payload.payload import Payload


@register_mode(id="payload.PayloadIdleMode", targets=[Payload])
class PayloadIdleMode(Mode):
    """Keep the payload stopped until the mission is shut down."""

    @override
    def initialize(self, node: Node, vehicle: Payload, params: ParamsBase) -> None:
        self.node = node
        self.vehicle = vehicle

    def on_enter(self) -> None:
        self.vehicle.stop()

    def on_update(self, time_delta: float) -> None:
        self.vehicle.stop()

    def check_status(self) -> str:
        return "continue"

    def on_exit(self) -> None:
        self.vehicle.stop()
