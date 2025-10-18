from typing import List
import math
import numpy as np
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV

class CircularFlightMode(Mode):
    """
    A mode for flying in a circular pattern
    """

    def __init__(self, node: Node, uav: UAV, center: tuple[float, float, float], 
                 radius: float, altitude: float, num_points: int = 36, 
                 wait_time_per_point: float = 1.0, coordinate_system: str = "LOCAL"):
        """
        Initialize the CircularFlightMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            center (tuple[float, float, float]): Center point of the circle (x, y, z or lon, lat, alt).
            radius (float): Radius of the circle in meters.
            altitude (float): Altitude to maintain during circular flight.
            num_points (int): Number of waypoints around the circle.
            wait_time_per_point (float): Time to wait at each waypoint in seconds.
            coordinate_system (str): "GPS" or "LOCAL" coordinate system.
        """
        super().__init__(node, uav)
        self.center = center
        self.radius = radius
        self.altitude = altitude
        self.num_points = num_points
        self.wait_time_per_point = wait_time_per_point
        self.coordinate_system = coordinate_system
        
        # Generate circular waypoints
        self.waypoints = self._generate_circular_waypoints()
        self.current_waypoint_index = 0
        self.wait_time_remaining = 0.0
        self.margin = 0.5  # Distance threshold for waypoint completion

    def _generate_circular_waypoints(self) -> List[tuple[float, float, float]]:
        """
        Generate waypoints in a circular pattern.
        
        Returns:
            List[tuple[float, float, float]]: List of waypoint coordinates.
        """
        waypoints = []
        angles = np.linspace(0, 2 * math.pi, self.num_points, endpoint=False)
        
        for angle in angles:
            if self.coordinate_system == "LOCAL":
                # Local coordinates: x is North, y is East, z is Down
                x = self.center[0] + self.radius * math.cos(angle)
                y = self.center[1] + self.radius * math.sin(angle)
                z = self.altitude  # Use specified altitude
                waypoints.append((x, y, z))
            elif self.coordinate_system == "GPS":
                # For GPS coordinates, we'd need to convert radius to lat/lon offsets
                # This is a simplified version - in practice you'd use proper geodetic calculations
                lat_offset = (self.radius / 111000) * math.cos(angle)  # Rough conversion
                lon_offset = (self.radius / (111000 * math.cos(math.radians(self.center[0])))) * math.sin(angle)
                waypoints.append((self.center[0] + lat_offset, self.center[1] + lon_offset, self.altitude))
        
        return waypoints

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for circular flight.
        """
        if self.current_waypoint_index >= len(self.waypoints):
            return  # Mission complete
        
        current_waypoint = self.waypoints[self.current_waypoint_index]
        
        # Calculate distance to current waypoint
        if self.coordinate_system == "GPS":
            distance = self.uav.distance_to_waypoint("GPS", current_waypoint)
            target = self.uav.gps_to_local(current_waypoint)
        else:
            distance = self.uav.distance_to_waypoint("LOCAL", current_waypoint)
            target = current_waypoint
        
        self.log(f"Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: "
                f"Distance: {distance:.2f}m, Position: {self.uav.get_local_position()}")
        
        if distance >= self.margin:
            # Still far from waypoint - keep publishing setpoints
            self.uav.publish_position_setpoint(target)
        else:
            # Reached waypoint
            if self.wait_time_remaining <= 0:
                # Move to next waypoint
                self.current_waypoint_index += 1
                self.wait_time_remaining = self.wait_time_per_point
                self.log(f"Moving to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
            else:
                # Wait at current waypoint
                self.wait_time_remaining -= time_delta
                self.log(f"Holding at waypoint {self.current_waypoint_index + 1} - "
                        f"waiting {self.wait_time_remaining:.1f} more seconds")
                # Continue publishing setpoint to maintain position
                self.uav.publish_position_setpoint(target)

    def check_status(self) -> str:
        """
        Check the status of the circular flight mode.
        
        Returns:
            str: The status of the mode.
        """
        self.log(f"Checking status: current_waypoint_index={self.current_waypoint_index}, len(waypoints)={len(self.waypoints)}")
        if self.current_waypoint_index >= len(self.waypoints):
            return "complete"
        else:
            return "continue"
