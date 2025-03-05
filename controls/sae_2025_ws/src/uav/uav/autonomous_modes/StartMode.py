from uav.autonomous_modes import Mode
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node
from uav import UAV


class StartMode(Mode):
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

        # self.command_sent = False

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for taking off vertically.
        """
        # if not self.command_sent:
            # self.command_sent = True
        self.uav.arm()
        self.node.get_logger().info("arming vehicle")
    
    def check_status(self) -> str:
        """
        Check the status of the mode.

        Returns:
            str: The status of the mode.
        """
        if self.uav.arm_state == VehicleStatus.ARMING_STATE_ARMED:
            return "complete"
        else:
            return "continue"