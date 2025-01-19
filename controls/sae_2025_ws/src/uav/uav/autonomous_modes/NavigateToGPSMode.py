import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from uav import Mode
from rclpy.node import Node

class NavigateToGPSMode(Mode):
    """
    A mode for navigating to a GPS coordinate while avoiding obstacles.
    """

    def __init__(self, node: Node, coordinate: tuple[float, float, float], avoidance_radius: float = 2.0):
        """
        Initialize the NavigateToGPSMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            coordinate (tuple[float, float, float]): The coordinate (x, y, z).
            avoidance_radius (float): Minimum safe distance from obstacles.
        """
        super().__init__(node)
        self.target_pose = None

        self.subscription = self.create_subscription(
            PoseStamped,
            '/uav/target_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self):
        """
        Periodic logic for setting gps coord.
        """

        self.target_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        uav.set_target_position(self.target_pose)