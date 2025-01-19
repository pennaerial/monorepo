#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, GotoSetpoint
from typing import List
import numpy as np


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_waypoint_navigation')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.goto_setpoint_publisher = self.create_publisher(
            GotoSetpoint, 'fmu/in/goto_setpoint', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.hover_time = 0
        self.takeoff_complete = False
        
        # Generate circular waypoints
        radius = 5.0  # Reduced radius for testing
        num_points = 100  # Number of waypoints along the circle
        angles = np.linspace(0, 2*np.pi, num_points, endpoint=False)
        
        # Create waypoints list starting with takeoff position
        self.waypoints = [[0.0, 0.0, self.takeoff_height]]  # Start at center
        
        # Add circular waypoints
        for angle in angles:
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            self.waypoints.append([x, y, self.takeoff_height])
            
        # Add return to center point
        self.waypoints.append([0.0, 0.0, self.takeoff_height])
        
        self.current_waypoint_index = 0
        self.waypoint_threshold = 2.0  # Increased threshold for testing
        self.mission_completed = False

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.get_logger().info("Switching to offboard mode...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        # self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def calculate_yaw(self, x: float, y: float) -> float:
        """Calculate the yaw angle to point towards the next waypoint."""
        # Calculate relative position
        dx = x - self.vehicle_local_position.x
        dy = y - self.vehicle_local_position.y
        
        # Calculate yaw angle
        yaw = np.arctan2(dy, dx)
        return yaw

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        # Calculate yaw to point towards the waypoint
        msg.yaw = self.calculate_yaw(x, y)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing setpoint: pos={[x, y, z]}, yaw={msg.yaw:.2f}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_goto_setpoint(self, x: float, y: float, z: float):
        """Publish a goto setpoint command."""
        msg = GotoSetpoint()
        msg.position = [x, y, z]
        msg.flag_takeoff = False  # Normal goto, not takeoff
        msg.flag_land = False     # Normal goto, not land
        msg.disable_weather_vane = False  # Allow weather vane mode
        msg.acceptance_radius = self.waypoint_threshold  # Use the same threshold as waypoints
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.goto_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing goto setpoint: pos={[x, y, z]}")

    def distance_to_waypoint(self) -> float:
        """Calculate distance to current waypoint."""
        current_waypoint = self.waypoints[self.current_waypoint_index]
        return np.sqrt(
            (self.vehicle_local_position.x - current_waypoint[0]) ** 2 +
            (self.vehicle_local_position.y - current_waypoint[1]) ** 2 +
            (self.vehicle_local_position.z - current_waypoint[2]) ** 2
        )

    def advance_to_next_waypoint(self) -> None:
        """Advance to the next waypoint if available."""
        if self.current_waypoint_index < len(self.waypoints) - 1:
            self.current_waypoint_index += 1
            self.get_logger().info(f"Advancing to waypoint {self.current_waypoint_index} at position {self.waypoints[self.current_waypoint_index]}")
        else:
            self.mission_completed = True
            self.get_logger().info("Mission completed, preparing to land")

    def check_takeoff_complete(self) -> bool:
        """Check if takeoff is complete."""
        height_reached = abs(self.vehicle_local_position.z - self.takeoff_height) < 0.5
        if height_reached and not self.takeoff_complete:
            self.hover_time += 1
            if self.hover_time >= 20:  # Reduced hover time to 2 seconds (20 * 0.1s)
                self.takeoff_complete = True
                self.get_logger().info("Takeoff complete, starting mission")
                return True
        return False

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if not self.vehicle_local_position:
            self.get_logger().info("No position data yet...")
            return

        # Initial takeoff sequence
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.get_logger().info("Vehicle armed")

        # Main control logic
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # Handle takeoff first
            if not self.takeoff_complete:
                self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
                self.check_takeoff_complete()
            # Then handle waypoint navigation
            elif not self.mission_completed:
                current_waypoint = self.waypoints[self.current_waypoint_index]
                self.publish_position_setpoint(*current_waypoint)

                # Check if we've reached the current waypoint
                if self.distance_to_waypoint() < self.waypoint_threshold:
                    self.advance_to_next_waypoint()
            else:
                # Mission is complete, initiate landing
                self.land()
                self.get_logger().info("Landing initiated")

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


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