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
        self.waypoints_added = False
        self.initialized = False

        # Message Clarity
        self.armed_message = False
        self.takeoff_message = False
        self.offboard_message = False
        self.land_message = False

        self.waypoints = self.initialize_waypoints()
        self.waypoints_added = False

        # Failsafe
        self.failsafe = False
        self.create_subscription(Bool, 'failsafe_trigger', self.failsafe_callback, 10)


    def initialize_waypoints(self):
        waypoints = []
        waypoints.append((None, "END"))
        return waypoints
    
    def failsafe_callback(self, msg: Bool):
        # When a manual failsafe command is received, set the failsafe flag.
        if msg.data:
            self.failsafe = True
            self.get_logger().info("Failsafe command received – initiating failsafe landing sequence.")

    def timer_callback(self):
        self.uav.publish_offboard_control_heartbeat_signal()
        if self.mission_completed or not self.uav.vehicle_local_position or not self.uav.sensor_gps:
            return
        if self.failsafe:
            if not self.uav.initiated_landing:
                self.uav.hover()
                self.get_logger().info("Failsafe: Switching to AUTO_LOITER mode.")
                self.uav.initiated_landing = True
            if self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                self.uav.land()  # Initiate the landing procedure.
                self.get_logger().info("Failsafe: Initiating landing.")
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
            self.uav.takeoff()
        elif self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER and not self.uav.initiated_landing and not self.waypoints_added:
            self.get_logger().info("Takeoff Complete. Starting Offboard Mode")
            self.uav.engage_offboard_mode()
        elif self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if not self.offboard_message:
                self.get_logger().info("Entered Offboard Mode")
                self.offboard_message = True
            if not self.waypoints_added:
                for waypoint, coordinate_system in self.waypoints:
                    self.uav.add_waypoint(waypoint, coordinate_system)
                self.waypoints_added = True
            else:
                self.uav.advance_to_next_waypoint()
        elif self.uav.initiated_landing:
            self.uav.advance_to_next_waypoint()
        elif self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            if not self.land_message:
                self.get_logger().info("Landed")
                self.land_message = True
        elif not self.initialized:
            self.get_logger().info("Initializing: Please hold")
        else:
            self.get_logger().info(f"Self.nav_state: {self.uav.nav_state}")
            self.get_logger().info("Why are we here, just to suffer")
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