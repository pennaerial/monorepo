from typing import List
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV

class NavGPSMode(Mode):
    """
    A mode for navigating to a GPS coordinate
    """

    def __init__(self, node: Node, uav: UAV, coordinates: List[tuple[tuple[float, float, float], float, str]], margin: float = 0.5):
        """
        Initialize the NavigateToGPSMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            coordinates (List[tuple[tuple[float, float, float], float, str]]): The coordinates to navigate to (x/y/z or lon/lat/alt, wait time, GPS/LOCAL).
            margin (float): The margin of error for the GPS coordinate.
        """
        super().__init__(node, uav)
        self.uav = uav
        self.coordinates = coordinates
        self.margin = margin
        self.coordinates, self.wait_time, self.coordinate_system = coordinates[0]
        self.index = 0

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for setting gps coord.
        """
        if self.uav.distance_to_waypoint(self.coordinate_system, self.coordinates) < self.margin:
            if self.wait_time > 0:
                self.wait_time -= time_delta
            else:
                self.index += 1
                if self.index < len(self.coordinates):
                    self.coordinates, self.wait_time, self.coordinate_system = self.coordinates[self.index]
                    if self.coordinate_system == "GPS":
                        local_target = self.uav.gps_to_local(self.coordinates)
                    elif self.coordinate_system == "LOCAL":
                        local_target = tuple(target - offset for target, offset in zip(self.coordinates, self.uav.local_origin))
                    else: 
                        raise ValueError(f"Invalid coordinate system {self.coordinate_system}")
                    self.uav.publish_position_setpoint(local_target)

    def check_status(self) -> str:
        """
        Check the status of the mode.
        
        Returns:
            str: The status of the mode.
        """
        if self.index >= len(self.coordinates):
            return "finished"
        else:
            return "continue"