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
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.mission_completed = False
        self.waypoints_added = False
        

    def timer_callback(self):
        self.uav.publish_offboard_control_heartbeat_signal()
        if self.mission_completed or not self.uav.vehicle_local_position:
            # self.get_logger().info("No position data yet...")
            return
        elif self.uav.offboard_setpoint_counter == 10:
            self.uav.engage_offboard_mode()
            self.uav.arm()
            self.get_logger().info("Vehicle armed")
        elif self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # self.get_logger().info("In Offboard Mode")
            if not self.waypoints_added:
                self.waypoints_added = True
                self.uav.add_waypoint((5.0, 0.0, -5.0), 'Local')
                self.uav.add_waypoint(None, 'END')
            elif not self.uav.takeoff_complete:
                self.uav.takeoff()
                self.uav.check_takeoff_complete()                
            elif not self.uav.mission_completed:
                self.uav.advance_to_next_waypoint()
            else:
                self.mission_completed = True
        if self.uav.offboard_setpoint_counter < 11:
            self.uav.offboard_setpoint_counter += 1
        
    


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