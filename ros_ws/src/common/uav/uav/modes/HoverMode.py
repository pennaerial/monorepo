from time import monotonic
from typing import override

from px4_msgs.msg import VehicleStatus
from pydantic import Field
from rclpy.node import Node
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from uav.vehicles.UAV import UAV


class HoverParams(ParamsBase):
    """Duration is measured in steady, real-time seconds from mode entry."""

    duration: float = Field(default=5.0, gt=0, allow_inf_nan=False)


@register_mode(
    id="uav.HoverMode",
    params_cls=HoverParams,
    targets=[UAV],
    transition_labels=["complete"],
)
class HoverMode(Mode[UAV, HoverParams]):
    """Hold the entry position and heading until a fixed deadline expires."""

    @override
    def initialize(self, node: Node, vehicle: UAV, params: HoverParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params

    @override
    def on_enter(self) -> None:
        # Takeoff establishes position and Offboard control before entering this mode.
        position = self.vehicle.local_position
        self.target_position = (float(position.x), float(position.y), float(position.z))
        self.target_yaw = float(self.vehicle.yaw)
        self.deadline = monotonic() + self.p.duration
        self._publish_hold()
        self.log(f"Holding {self.target_position} for {self.p.duration:g}s")

    def _publish_hold(self) -> None:
        if self.vehicle.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return
        # Leave velocity/acceleration feedforward unset; PX4 corrects position error.
        unused = (float("nan"),) * 3
        self.vehicle.publish_trajectory_setpoint(
            self.target_position, unused, unused, self.target_yaw
        )

    @override
    def on_update(self, time_delta: float) -> None:
        self._publish_hold()

    @override
    def check_status(self) -> str:
        if self.vehicle.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return "error"
        if monotonic() >= self.deadline:
            self.log("Hover complete")
            return "complete"
        return "continue"
