import math
from typing import List
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV
from uav.vision_nodes import SaveImageNode
class SaveImageMode(Mode):
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
        dist = 0
        if self.index != -1:
            dist = self.uav.distance_to_waypoint(self.coordinate_system, self.goal)
            self.log(f"Distance to waypoint: {dist}, current position: {self.uav.get_local_position()}")
            
        if dist >= self.margin and not self.is_circling:
            self.uav.publish_position_setpoint(self.target) # PX4 expects stream of setpoints
            self.circle_radius = -self.target[2] / 3
        elif self.goal is None or dist < self.margin + self.circle_radius:
            if self.index == -1 or self.wait_time <= 0:
                self.index += 1
                if self.index >= len(self.coordinates):
                    return
                self.goal, self.wait_time, self.coordinate_system = self.coordinates[self.index]
                self.target = self.get_local_target()
                self.uav.publish_position_setpoint(self.target)
            else:
                if self.circle_cycle >= 0:
                    self.is_circling = True
                    circle_target = list(self.target)
                    angle_in_radians = self.angle * math.pi / 180
                    circle_target[0] += self.circle_radius * math.cos(angle_in_radians)
                    circle_target[1] += self.circle_radius * math.sin(angle_in_radians)

                    self.uav.publish_position_setpoint(tuple(circle_target))

                    if self.circle_pause_time > 0:
                        self.circle_pause_time -= time_delta
                        self.log(f"Angle: {self.angle}")
                        self.log(f"Pausing to take photos for: {self.circle_pause_time} more seconds")
                        return

                    self.circle_cycle -= 1
                    self.angle += 5

                    if self.angle % 15 == 0:
                        self.circle_pause_time = 2
                    else:
                        # self.log(f"Angle: {self.angle}")
                        self.log(f"Holding - circling for {self.circle_cycle} more cycles.")

                elif self.wait_time >= 0:
                    self.wait_time -= time_delta
                    if self.wait_time <= 0:
                        self.angle = 0
                        self.is_circling = False
                        self.circle_cycle = 360 / self.angle_increment
                    # self.log(f"Holding - waiting for {self.wait_time} more seconds")

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