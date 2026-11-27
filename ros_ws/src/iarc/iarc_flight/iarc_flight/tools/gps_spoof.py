"""Publishes a fake GPS fix type so a TrackMode's rtk_loss stop can be tested in SITL, where PX4
only reports a 3D fix. Point the TrackMode at it with gps_topic: spoof/vehicle_gps_position.

    ros2 run iarc_flight gps_spoof --ros-args -r __ns:=/uav_0
    ros2 param set /uav_0/gps_spoof fix rtk_float    # any FIX_TYPES name, or off to go silent
"""

import rclpy
from px4_msgs.msg import SensorGps
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from iarc_flight.runtime.gps import FIX_TYPES, PX4_QOS


class GpsSpoof(Node):
    def __init__(self) -> None:
        super().__init__("gps_spoof")
        self.fix: int | None = FIX_TYPES["rtk_fixed"]
        self.declare_parameter("fix", "rtk_fixed")
        self.add_on_set_parameters_callback(self.on_set)
        self.pub = self.create_publisher(SensorGps, "spoof/vehicle_gps_position", PX4_QOS)
        self.timer = self.create_timer(0.1, self.publish)
        self.get_logger().info(f"Spoofing GPS fix on {self.pub.topic_name}: rtk_fixed")

    def on_set(self, params) -> SetParametersResult:
        for p in params:
            if p.name != "fix":
                continue
            name = str(p.value).lower()
            if name != "off" and name not in FIX_TYPES:
                return SetParametersResult(
                    successful=False, reason=f"fix is one of {', '.join(FIX_TYPES)} or off"
                )
            self.fix = None if name == "off" else FIX_TYPES[name]
            self.get_logger().info(f"GPS fix: {name}")
        return SetParametersResult(successful=True)

    def publish(self) -> None:
        if self.fix is not None:
            msg = SensorGps()
            msg.timestamp = self.get_clock().now().nanoseconds // 1000  # µs
            msg.fix_type = self.fix
            self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GpsSpoof()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
