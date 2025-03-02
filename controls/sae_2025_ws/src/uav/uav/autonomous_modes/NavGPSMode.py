import random
from typing import List
from uav import Mode
from rclpy.node import Node
from uav import UAV

class NavGPSMode(Mode):
    """
    A mode for navigating to a GPS coordinate
    """

    def __init__(self, node: Node, uav: UAV, coordinates: List[tuple[tuple[float, float, float], float]]):
        """
        Initialize the NavigateToGPSMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            coordinate (tuple[float, float, float]): The coordinate (x, y, z).
        """
        super().__init__(node, uav, [])
        self.uav = uav
        
        self.times_between = []
        for coordinate, time_between in coordinates:
            self.uav.add_waypoint(coordinate, "GPS")
            self.times_between.append(time_between)

        self.uav.add_waypoint(None, "END")

        self.index = 0
        self.time_between = 0

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for setting gps coord.
        """

        if self.index >= len(self.times_between):
            return

        self.time_between += time_delta

        if self.time_between > self.times_between[self.index]:
            self.time_between = 0
            self.uav.advance_to_next_waypoint()
            self.index += 1

    def check_status(self) -> str:
        """
        Check the status of the mode.
        
        Returns:
            str: The status of the mode.
        """
        
        if self.uav.coordinate_system == "END":
            return "finshed"
        else:
            return "continue"