from rclpy.node import Node
from px4_msgs.msg import VehicleStatus

from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.runtime.plugin_loader import register_plugin

@register_plugin(name="uav.LandingMode", base_cls=Mode)
class LandingMode(Mode):
    """
    A mode for landing vertically.
    """

    def __init__(self, node: Node, vehicle: UAV):
        """
        Initialize the LandingMode

        Args:
            node (Node): ROS 2 node managing the UAV.
            vehicle (UAV): The UAV instance to control.
        """
        super().__init__(node, vehicle)

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for landing.
        """
        # Maintain current position setpoints until AUTO_LAND mode is engaged
        # This prevents losing offboard connection during the transition
        # Lock yaw to prevent spinning while hovering before landing
        if self.vehicle.local_position:
            self.vehicle.publish_position_setpoint(
                (
                    self.vehicle.local_position.x,
                    self.vehicle.local_position.y,
                    self.vehicle.local_position.z,
                ),
                lock_yaw=True,
            )

        # Only send land command if not already in AUTO_LAND mode
        if self.vehicle.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
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
