from math import ceil, isfinite
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
        self.target_position: tuple[float, float, float] | None = None
        self.target_yaw: float | None = None
        self.deadline: float | None = None
        self.last_countdown_seconds: int | None = None
        self.error = False

    @override
    def on_enter(self) -> None:
        # Modes are constructed at mission load, so capture and reset on every entry.
        self.target_position = None
        self.target_yaw = None
        self.deadline = monotonic() + self.p.duration
        self.last_countdown_seconds = None
        self.error = False
        if not self._ready_to_hold():
            return

        position = self.vehicle.local_position
        self.target_position = (float(position.x), float(position.y), float(position.z))
        self.target_yaw = float(self.vehicle.yaw)
        self._publish_hold()
        self.log(f"Holding {self.target_position} for {self.p.duration:g}s")
        self._log_countdown(monotonic())

    def _ready_to_hold(self) -> bool:
        if self.error:
            return False
        position = self.vehicle.local_position
        if self.vehicle.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            reason = "Hover requires Offboard control."
        elif (
            position is None
            or not position.xy_valid
            or not position.z_valid
            or not all(isfinite(value) for value in (position.x, position.y, position.z))
            or self.vehicle.yaw is None
            or not isfinite(self.vehicle.yaw)
        ):
            reason = "Hover requires a valid local position and heading."
        else:
            return True

        self.node.get_logger().error(reason)
        self.error = True
        return False

    def _publish_hold(self) -> None:
        # Leave velocity/acceleration feedforward unset; PX4 corrects position error.
        unused = (float("nan"),) * 3
        self.vehicle.publish_trajectory_setpoint(
            self.target_position, unused, unused, self.target_yaw
        )

    def _log_countdown(self, now: float) -> None:
        remaining = max(0, ceil(self.deadline - now))
        if remaining != self.last_countdown_seconds:
            self.last_countdown_seconds = remaining
            self.log(f"Hover: {remaining}s remaining" if remaining else "Hover complete")

    @override
    def on_update(self, time_delta: float) -> None:
        if self.target_position is not None and self._ready_to_hold():
            self._publish_hold()
            self._log_countdown(monotonic())

    @override
    def check_status(self) -> str:
        if self.error:
            return "error"
        # Position error and update gaps never pause or restart the deadline.
        if self.deadline is not None:
            now = monotonic()
            if now >= self.deadline:
                self._log_countdown(now)
                return "complete"
        return "continue"
