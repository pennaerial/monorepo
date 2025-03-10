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
        Periodic logic for taking off vertically.
        """
        self.uav.land()
    
    def check_status(self) -> str:
        """
        Check the status of the mode.

        Returns:
            str: The status of the mode.
        """
        if self.uav.vehicle_status == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            return "continue"
        else:
            return "terminate"