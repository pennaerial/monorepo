import rclpy
from rclpy.node import Node
from uav.UAV_hardware import UAV
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleGlobalPosition,  # <-- Import for the global position topic
    GotoSetpoint
)
from typing import List
import numpy as np
import time
from std_msgs.msg import Bool

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_waypoint_navigation')

        self.uav = UAV(self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.mission_completed = False
        self.armed_message = False
        self.takeoff_message = False
        self.initialized = False

    def timer_callback(self):
        self.uav.publish_offboard_control_heartbeat_signal()
        if self.mission_completed or not self.uav.vehicle_local_position or not self.uav.sensor_gps:
            return

        if self.uav.offboard_setpoint_counter == 10:
            self.uav.arm()
            self.initialized = True
            self.uav.set_origin()
            if not self.armed_message:
                self.get_logger().info("Vehicle armed, Origin Set")
                self.armed_message = True
        elif not self.uav.takeoff_complete:
            if not self.takeoff_message:
                self.get_logger().info("Taking Off")
                self.takeoff_message = True
            self.uav.vtol_takeoff()
        if self.uav.offboard_setpoint_counter < 11:
            self.uav.offboard_setpoint_counter += 1
        
    def get_payload_gps(self):
        return (self.lat, self.lon, 0.0)
    


def main(args=None) -> None:
    print('Starting offboard control node with waypoint navigation...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)