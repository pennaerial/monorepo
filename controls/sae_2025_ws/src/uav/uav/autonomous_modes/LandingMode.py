from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV
from px4_msgs.msg import VehicleStatus


class LandingMode(Mode):
    """
    A mode for taking off vertically.
    """

    def __init__(self, node: Node, uav: UAV):
        """
        Initialize the LandingMode

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
        """
        super().__init__(node, uav)

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for landing.
        """
        # Maintain current position setpoints until AUTO_LAND mode is engaged
        # This prevents losing offboard connection during the transition
        # Lock yaw to prevent spinning while hovering before landing
        if self.uav.local_position:
            self.uav.publish_position_setpoint(
                (self.uav.local_position.x, self.uav.local_position.y, self.uav.local_position.z),
                lock_yaw=True
            )
        
        # Only send land command if not already in AUTO_LAND mode
        if self.uav.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.uav.land()
    
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
        if (self.uav.arm_state == VehicleStatus.ARMING_STATE_DISARMED and
            self.uav.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND):
            return "terminate"  # Mission complete - shut down
        return "continue"