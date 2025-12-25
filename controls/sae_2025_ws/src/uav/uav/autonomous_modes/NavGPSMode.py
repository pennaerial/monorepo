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
        self.coordinate_system = None
        self.target = None
        self.wait_time = 0.0
        self._last_log_time = 0.0

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for setting gps coord.
        """
        dist = 0
        if self.index != -1:
            dist = self.uav.distance_to_waypoint(self.coordinate_system, self.goal)
            curr_pos = self.uav.get_local_position()
            curr_gps = self.uav.get_gps()
            
            # Log with 2 decimal places for concise output
            if curr_pos:
                self.log(f"Dist: {dist:.2f}m | Curr LOCAL: ({curr_pos[0]:.2f}, {curr_pos[1]:.2f}, {curr_pos[2]:.2f}) | "
                        f"Target {self.coordinate_system}: {self.goal} | Target LOCAL: ({self.target[0]:.2f}, {self.target[1]:.2f}, {self.target[2]:.2f})")
            if curr_gps:
                self.log(f"GPS: lat={curr_gps[0]:.2f}, lon={curr_gps[1]:.2f}, alt={curr_gps[2]:.2f}m")
        
        # Always publish setpoints to maintain offboard connection (PX4 requires continuous stream)
        if self.target is not None:
            self.uav.publish_position_setpoint(self.target)
            
        if self.goal is None or dist < self.margin:
            if self.index == -1 or self.wait_time <= 0:
                self.index += 1
                if self.index >= len(self.coordinates):
                    self.log("All waypoints completed")
                    return
                self.goal, self.wait_time, self.coordinate_system = self.coordinates[self.index]
                self.target = self.get_local_target()
                self.log(f"New waypoint: {self.coordinate_system} {self.goal} -> LOCAL target: ({self.target[0]:.2f}, {self.target[1]:.2f}, {self.target[2]:.2f})")
            else:
                self.wait_time -= time_delta
                self.log(f"Holding - waiting {self.wait_time:.2f}s")

    def get_local_target(self) -> tuple[float, float, float]:
        """
        Get the local target of the UAV.

        Returns:
            tuple[float, float, float]: The local target of the UAV.
        """
        if self.coordinate_system == "GPS":
            local_target = self.uav.gps_to_local(self.goal)
            self.log(f"GPS {self.goal} -> LOCAL {local_target}")
            return local_target
        elif self.coordinate_system == "LOCAL":
            # LOCAL coordinates are already in NED frame relative to origin
            local_target = tuple(float(x) for x in self.goal)
            if self.uav.local_origin:
                self.log(f"LOCAL waypoint {self.goal} -> LOCAL target {local_target} (origin: {self.uav.local_origin})")
            return local_target
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