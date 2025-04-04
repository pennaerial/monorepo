import rclpy
from rclpy.node import Node
from uav.UAV import UAV
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
        self.timer1 = 0
        self.lower_time = 1.87
        self.done = False
        self.last_time = time.time()

    def timer_callback(self):
        self.curr_time = time.time()
        time_delta = self.curr_time - self.last_time
        self.last_time = self.curr_time
        if self.timer1 < self.lower_time and self.timer1 >= 0:
            self.get_logger().info(f"Dropping: time passed {self.timer1}")
            self.uav.drop_payload()
            self.timer1 += time_delta
        if self.timer1 >= self.lower_time:
            self.lowering = False
            self.uav.pickup_payload()
            self.timer1 -= time_delta
        if self.timer1 < 0:
            self.uav.disable_servo()
            self.done = True



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