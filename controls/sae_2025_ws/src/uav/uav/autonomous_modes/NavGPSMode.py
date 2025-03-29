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
                                                                               Local are NED coordinates, relative to the starting position (https://docs.px4.io/main/en/ros2/user_guide.html#ros-2-px4-frame-conventions).
            margin (float): The margin of error for the GPS coordinate.
        """
        super().__init__(node, uav)
        self.coordinates = coordinates
        self.goal = None
        self.margin = margin
        self.index = -1

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for setting gps coord.
        """
        dist = 0
        if self.index != -1:
            dist = self.uav.distance_to_waypoint(self.coordinate_system, self.goal)
            self.log(f"Distance to waypoint: {dist}, current position: {self.uav.get_local_position()}")
            
        if dist >= self.margin:
            self.uav.publish_position_setpoint(self.target) # PX4 expects stream of setpoints
        elif self.goal is None or dist < self.margin:
            if self.index == -1 or self.wait_time <= 0:
                self.index += 1
                if self.index >= len(self.coordinates):
                    return
                self.goal, self.wait_time, self.coordinate_system = self.coordinates[self.index]
                self.target = self.get_local_target()
                self.uav.publish_position_setpoint(self.target)
            else:
                self.wait_time -= time_delta
                self.log(f"Holding - waiting for {self.wait_time} more seconds")

    def get_local_target(self) -> tuple[float, float, float]:
        """
        Get the local target of the UAV.

        Returns:
            tuple[float, float, float]: The local target of the UAV.
        """
        if self.coordinate_system == "GPS":
            return self.uav.gps_to_local(self.goal)
        elif self.coordinate_system == "LOCAL":
            return tuple(float(x) for x in self.goal)
        else: 
            raise ValueError(f"Invalid coordinate system {self.coordinate_system}")

    def check_status(self) -> str:
        """
        Check the status of the mode.
        
        Returns:
            str: The status of the mode.
        """
        if self.index >= len(self.coordinates):
            return "complete"
        else:
            return "continue"