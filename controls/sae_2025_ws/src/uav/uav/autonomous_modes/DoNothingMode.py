import math
from typing import List
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV
from uav.vision_nodes import SaveImageNode


class DoNothingMode(Mode):
    """
    A mode for navigating to a GPS coordinate
    """

    def __init__(self, node: Node, uav: UAV, coordinates: List[tuple[tuple[float, float, float], float, str]],
                 margin: float = 0.5):
        """
        Initialize the NavigateToGPSMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            coordinates (List[tuple[tuple[float, float, float], float, str]]): The coordinates to navigate to (x/y/z or lon/lat/alt, wait time, GPS/LOCAL).
                                                                               Local are NED coordinates, relative to the starting position (https://docs.px4.io/main/en/ros2/user_guide.html#ros-2-px4-frame-conventions).
            margin (float): The margin of error for the GPS coordinate.
        """
        super().__init__(node, uav)
        self.coordinates = coordinates
        self.goal = None
        self.margin = margin
        self.circle_pause_time = 2
        self.circle_radius = 0
        self.is_circling = False
        self.index = -1
        self.angle = 0
        self.angle_increment = 15
        self.circle_cycle = 360 / self.angle_increment

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for setting gps coord.
        """
        self.log("I'm on!")

    def check_status(self) -> str:
        """
        Check the status of the mode.

        Returns:
            str: The status of the mode.
        """
        return "continue"