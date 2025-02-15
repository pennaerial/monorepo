<<<<<<< HEAD
#!/usr/bin/env python3
=======
>>>>>>> main
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

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_waypoint_navigation')

        self.uav = UAV(self)
        self.takeoff_complete = False
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.takeoff_height = -5.0
        self.mission_completed = False
        self.hover_time = 0

    def timer_callback(self):
        self.uav.publish_offboard_control_heartbeat_signal()
        if self.mission_completed or not self.uav.vehicle_local_position:
            # self.get_logger().info("No position data yet...")
            return
        if self.uav.offboard_setpoint_counter == 10:
            self.uav.engage_offboard_mode()
            self.uav.arm()
            self.get_logger().info("Vehicle armed")
        if self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info("In Offboard Mode")
            # Handle takeoff first
            if not self.takeoff_complete:
                self.uav.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
                self.check_takeoff_complete()
            else:
                self.uav.land()
                self.mission_completed = True
        if self.uav.offboard_setpoint_counter < 11:
            self.uav.offboard_setpoint_counter += 1
    
    def check_takeoff_complete(self) -> bool:
        """Check if takeoff is complete."""
        height_reached = abs(self.uav.vehicle_local_position.z - self.takeoff_height) < 0.5
        if height_reached and not self.takeoff_complete:
            self.hover_time += 1
            if self.hover_time >= 20:  # Reduced hover time to 2 seconds (20 * 0.1s)
                self.takeoff_complete = True
                self.get_logger().info("Takeoff complete, starting mission")
                return True
        return False
    

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
