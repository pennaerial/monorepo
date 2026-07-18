from typing import override

from rclpy.node import Node
from px4_msgs.msg import VehicleStatus
from pydantic import BaseModel

from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import register_mode


class LandingParams(BaseModel):
    pass


@register_mode(id="uav.LandingMode", targets=[UAV])
class LandingMode(Mode):
    """
    A mode for landing vertically.
    """

    @override
    def initialize(self, node: Node, vehicle: UAV, params: LandingParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for landing.
        """
        # Let PX4 own the setpoint stream in AUTO_LAND so its land detector can auto-disarm.
        if self.vehicle.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            if self.vehicle.local_position:
                self.vehicle.publish_position_setpoint(
                    (
                        self.vehicle.local_position.x,
                        self.vehicle.local_position.y,
                        self.vehicle.local_position.z,
                    ),
                    lock_yaw=True,
                )
            self.vehicle.land()

    def check_status(self) -> str:
        """
        Check the status of the mode.

        Returns:
            str: "continue" while landing, "terminate" when fully landed
        """
        # Landing is complete when vehicle has both:
        # 1. Auto-disarmed after touchdown (arm_state == DISARMED)
        # 2. Exited AUTO_LAND mode (nav_state != AUTO_LAND)
        # This ensures the full landing sequence completes before terminating
        if (
            self.vehicle.arm_state == VehicleStatus.ARMING_STATE_DISARMED
            and self.vehicle.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        ):
            return "terminate"  # Mission complete - shut down
        return "continue"

    @classmethod
    @override
    def get_params_cls(cls) -> type[BaseModel]:
        return LandingParams
