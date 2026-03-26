from abc import ABC, abstractmethod
import math
import numpy as np
from rclpy.node import Node
from uav.utils import R_earth


class Vehicle(ABC):
    """
    Abstract base class for all vehicles (UAVs, ground vehicles, etc.).
    """

    def __init__(self, node: Node, DEBUG: bool = False):
        self.node = node
        self.DEBUG = DEBUG

        # Shared state — present on all vehicle types
        self.failsafe = False
        self.failsafe_trigger = False
        self.arm_state = None

        self.local_position = None
        self.global_position = None
        self.gps_origin = None
        self.yaw = None

        self.vision_clients = {}

    @property
    @abstractmethod
    def vehicle_type(self) -> str:
        """Return a string identifying the vehicle type (e.g. 'MULTICOPTER', 'VTOL', 'PAYLOAD')."""
        pass

    # -------------------------
    # Getters / data access
    # -------------------------
    def get_gps(self):
        if self.global_position:
            return (
                self.global_position.lat,
                self.global_position.lon,
                self.global_position.alt,
            )
        else:
            self.node.get_logger().warn("No GPS data available.")
            return None

    def get_local_position(self):
        if self.local_position:
            return (self.local_position.x, self.local_position.y, self.local_position.z)
        else:
            self.node.get_logger().warn("No local position data available.")
            return None

    # -------------------------
    # Navigation helpers
    # -------------------------
    def calculate_yaw(self, x: float, y: float) -> float:
        """Calculate the yaw angle to point towards a target (x, y) in the local frame."""
        dx = x - self.local_position.x
        dy = y - self.local_position.y

        # If very close to target, maintain current yaw to prevent spinning
        if np.linalg.norm([dx, dy]) < 3.0 and self.yaw is not None:
            return self.yaw

        return np.arctan2(dy, dx)

    def vehicle_to_local(self, point, relative=False):
        """
        Converts a point in the vehicle's body frame to the local world frame.

        :param point: A tuple (x, y, z) in the vehicle's body frame.
        :param relative: If True, the result is offset from the current local position.
        :return: (x, y, z) in the local world frame.
        """
        current_pos = self.get_local_position()
        point_x, point_y, point_z = point

        rotated_x = point_x * math.cos(self.yaw) - point_y * math.sin(self.yaw)
        rotated_y = point_x * math.sin(self.yaw) + point_y * math.cos(self.yaw)

        if relative:
            return (
                current_pos[0] + rotated_x,
                current_pos[1] + rotated_y,
                current_pos[2] + point_z,
            )
        else:
            return (rotated_x, rotated_y, point_z)

    def distance_to_waypoint(self, coordinate_system, waypoint) -> float:
        """Calculate the distance to a waypoint. coordinate_system: 'GPS' or 'LOCAL'."""
        if coordinate_system == "GPS":
            curr_gps = self.get_gps()
            return self.gps_distance_3d(
                waypoint[0], waypoint[1], waypoint[2],
                curr_gps[0], curr_gps[1], curr_gps[2],
            )
        elif coordinate_system == "LOCAL":
            return np.sqrt(
                (self.local_position.x - waypoint[0]) ** 2
                + (self.local_position.y - waypoint[1]) ** 2
                + (self.local_position.z - waypoint[2]) ** 2
            )

    def gps_distance_3d(self, lat1, lon1, alt1, lat2, lon2, alt2) -> float:
        """Calculate the 3D distance in meters between two GPS points."""
        curr_x, curr_y, curr_z = self.gps_to_local((lat1, lon1, alt1))
        tar_x, tar_y, tar_z = self.gps_to_local((lat2, lon2, alt2))
        return np.sqrt(
            (curr_x - tar_x) ** 2 + (curr_y - tar_y) ** 2 + (curr_z - tar_z) ** 2
        )

    def gps_to_local(self, target):
        """Convert GPS coordinates (lat, lon, alt) to local NED (x, y, z) in meters."""
        if self.gps_origin is None:
            self.node.get_logger().error(
                "gps_origin not set. Cannot convert GPS to local coordinates."
            )
            return None

        target_lat, target_lon, target_alt = target
        ref_lat, ref_lon, ref_alt = self.gps_origin

        d_lat = math.radians(target_lat - ref_lat)
        d_lon = math.radians(target_lon - ref_lon)

        x = d_lat * R_earth
        y = d_lon * R_earth * math.cos(math.radians(ref_lat))
        z = -(target_alt - ref_alt)

        return (x, y, z)

    def local_to_gps(self, local_pos):
        """Convert local NED (x, y, z) in meters to GPS (lat, lon, alt)."""
        if self.gps_origin is None:
            self.node.get_logger().error(
                "gps_origin not set. Cannot convert local to GPS coordinates."
            )
            return None

        x, y, z = local_pos
        lat0, lon0, alt0 = self.gps_origin

        dlat = (x / R_earth) * (180.0 / math.pi)
        dlon = (y / (R_earth * math.cos(math.radians(lat0)))) * (180.0 / math.pi)

        return (lat0 + dlat, lon0 + dlon, alt0 - z)
