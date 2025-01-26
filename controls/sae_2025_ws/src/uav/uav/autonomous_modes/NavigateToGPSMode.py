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
    
    def set_target(self, target_pose: tuple[float, float, float]):
        """
        Set the target GPS coordinate.

        Args:
            target_pose (tuple[float, float, float]): The target GPS coordinate.
        """
        self.target_pose = target_pose

    def on_update(self):
        """
        Periodic logic for setting gps coord.
        """
        if self.target_pose is not None:
            self.uav.go_to_target(self.target_pose)