import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import SensorGps
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import Bool


class AltitudeNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Create subscriptions
        self.status_sub = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.vehicle_altitude_callback,
            qos_profile)

    #receives and sets vehicle status values 
    def vehicle_altitude_callback(self, msg):
        print(msg.altitude_msl_m)
        
            


def main(args=None):
    rclpy.init(args=args)

    # Create the offboard control node
    node = AltitudeNode()

    # Spin the node so the callback function is called.
    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()