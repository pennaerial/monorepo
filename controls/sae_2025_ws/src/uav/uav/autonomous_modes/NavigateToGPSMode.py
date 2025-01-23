from uav import Mode
from rclpy.node import Node

class NavigateToGPSMode(Mode):
    """
    A mode for navigating to a GPS coordinate
    """

    def __init__(self, node: Node, coordinate: tuple[float, float, float]):
        """
        Initialize the NavigateToGPSMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            coordinate (tuple[float, float, float]): The coordinate (x, y, z).
        """
        super().__init__(node)
        self.target_pose = None

    def pose_callback(self):
        """
        Periodic logic for setting gps coord.
        """

        self.target_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.direction)
        uav.set_target_position(self.target_pose)