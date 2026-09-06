from typing import override

from rclpy.node import Node

from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode


class FlyToPointParams(ParamsBase):
    target: tuple[float, float, float]
    margin: float = 0.5


@register_mode(
    id="uav.FlyToPointMode",
    params_cls=FlyToPointParams,
    targets=[UAV],
    transition_labels=["complete"],
)
class FlyToPointMode(Mode[UAV, FlyToPointParams]):
    """Fly a UAV to one position in its local NED frame."""

    @override
    def initialize(
        self,
        node: Node,
        vehicle: UAV,
        params: FlyToPointParams,
    ) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params

    @override
    def on_update(self, time_delta: float) -> None:
        if self.vehicle.local_position is None:
            return

        self.vehicle.publish_position_setpoint(self.p.target)

    @override
    def check_status(self) -> str:
        if self.vehicle.local_position is None:
            return "continue"

        distance = self.vehicle.distance_to_waypoint("LOCAL", self.p.target)
        if distance <= self.p.margin:
            return "complete"

        return "continue"