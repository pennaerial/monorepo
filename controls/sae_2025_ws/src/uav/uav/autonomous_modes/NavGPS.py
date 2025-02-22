import random
from typing import List
from uav import Mode
from rclpy.node import Node
from uav import UAV

class NavGPS(Mode):
    """
    A mode for navigating to a GPS coordinate
    """

    def __init__(self, node: Node, uav: UAV, coordinates: List[tuple[float, float, float]]):
        """
        Initialize the NavigateToGPSMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            coordinate (tuple[float, float, float]): The coordinate (x, y, z).
        """
        super().__init__(node, uav, [])
        for coordinate in coordinates:
            super().uav.add_waypoint(coordinate, "Local")
    
    def set_target(self, target_pos: tuple[float, float, float]):
        """
        Set the target GPS coordinate.

        Args:
            target_pos (tuple[float, float, float]): The target GPS coordinate.
        """
        self.target_pos = target_pos

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for setting gps coord.
        """
        if self.target_pos is not None:
            self.uav.set_target_position(self.target_pos)

    def check_status(self) -> str:
        """
        Check the status of the mode.
        
        Returns:
            str: The status of the mode.
        """
        # range = 1
        # if gps['latitude']- self.target_pos[0] < range and gps['longitude'] - self.target_pos[1] < range and gps['altitude'] - self.target_pos[2] < range: 
        #     return 'continue'

        if random.random() < 0.1:
            return 'continue'