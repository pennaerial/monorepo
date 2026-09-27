import math

import rclpy
from px4_msgs.msg import VehicleGlobalPosition
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sim_interfaces.srv import GetPatchPoints


EARTH_RADIUS_M = 6378137.0
POINT_OFFSET_M = 3.0


class PatchPointsService(Node):
    """ filler, gives 2 points based on uav gps, later to be replaced with cv algo"""
    def __init__(self) -> None:
        super().__init__("patch_points_service")
        self.declare_parameter(
            "global_position_topic", "/uav_0/fmu/out/vehicle_global_position"
        )
        topic = str(self.get_parameter("global_position_topic").value)
        self.latest_position = None
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            VehicleGlobalPosition, topic, self._position_callback, qos
        )
        self.create_service(GetPatchPoints, "/get_patch_points", self._request_callback)
        self.get_logger().info(f"Providing patch points from {topic}")

    def _position_callback(self, message: VehicleGlobalPosition) -> None:
        self.latest_position = message

    def _request_callback(self, request, response):
        del request
        if self.latest_position is None:
            self.get_logger().warning("No UAV GPS position available yet")
            return response

        latitude = math.radians(float(self.latest_position.lat))
        latitude_offset = math.degrees(POINT_OFFSET_M / EARTH_RADIUS_M)
        longitude_offset = math.degrees(
            POINT_OFFSET_M / (EARTH_RADIUS_M * math.cos(latitude))
        )
        response.latitude_deg = [
            float(self.latest_position.lat) + latitude_offset,
            float(self.latest_position.lat),
        ]
        response.longitude_deg = [
            float(self.latest_position.lon),
            float(self.latest_position.lon) + longitude_offset,
        ]
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatchPointsService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()