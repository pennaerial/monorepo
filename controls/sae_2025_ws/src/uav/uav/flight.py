#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
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

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --------------------------
        # Publishers
        # --------------------------
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        self.goto_setpoint_publisher = self.create_publisher(
            GotoSetpoint, 'fmu/in/goto_setpoint', qos_profile
        )

        # --------------------------
        # Subscribers
        # --------------------------
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )
        # NEW: Subscribe to the global position topic
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.vehicle_global_position_callback,
            qos_profile
        )

        # --------------------------
        # Internal State
        # --------------------------
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.vehicle_global_position = VehicleGlobalPosition()
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None

        # Desired GPS target to fly to (example)
        self.target_lat = 47.398170327054  # Replace with your desired lat
        self.target_lon = 8.545649021863965  # Replace with your desired lon
        self.target_alt = 530.0  # Replace with your desired alt

        # How high above ground we want to fly once we reach area
        # (Below, you'll see we convert alt to a local -Z; 
        #  you might want to unify these ideas if your local frame uses NED.)
        self.takeoff_height = -5.0

        self.takeoff_complete = False
        self.hover_time = 0

        # If you only want to go to *one* waypoint, you might track that waypoint
        # as your final step. We'll store it once we can compute it.
        self.local_target_x = 0.0
        self.local_target_y = 0.0
        self.local_target_z = 0.0

        self.mission_completed = False

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    # ----------------------------------------------------------------------------
    # Subscriptions
    # ----------------------------------------------------------------------------
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback for local position data."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback for vehicle status data."""
        self.vehicle_status = vehicle_status

    def vehicle_global_position_callback(self, msg):
        """Callback for global position data (lat, lon, alt)."""
        self.vehicle_global_position = msg

        # On the first message, record the home lat/lon/alt
        # so we can compute local offsets relative to that point.
        if self.home_lat is None:
            self.home_lat = msg.lat  # degrees
            self.home_lon = msg.lon  # degrees
            self.home_alt = msg.alt  # meters above MSL
            self.get_logger().info(f"Home position lat={self.home_lat}, lon={self.home_lon}, alt={self.home_alt}")

    # ----------------------------------------------------------------------------
    # Arming, Mode, Landing
    # ----------------------------------------------------------------------------
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.get_logger().info("Switching to offboard mode...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    # ----------------------------------------------------------------------------
    # Offboard Control
    # ----------------------------------------------------------------------------
    def publish_offboard_control_heartbeat_signal(self):
        """Keep offboard mode alive with repeated messages."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def calculate_yaw(self, x: float, y: float) -> float:
        """Calculate the yaw angle to face the waypoint (x,y)."""
        dx = x - self.vehicle_local_position.x
        dy = y - self.vehicle_local_position.y
        yaw = np.arctan2(dy, dx)
        return yaw

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish local position setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
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

    # ----------------------------------------------------------------------------
    # GPS -> Local Conversion
    # ----------------------------------------------------------------------------
    def gps_to_local_waypoint(self, target_lat, target_lon, target_alt):
        """
        Convert a desired lat/lon/alt to a local (x, y, z).
        This uses a simple equirectangular approximation for small distances.
        For high accuracy or large distances, consider a proper geodesy library.
        """
        if self.home_lat is None or self.home_lon is None or self.home_alt is None:
            self.get_logger().warning("Home position not yet set; cannot convert GPS to local.")
            return 0.0, 0.0, 0.0

        # Earth radius in meters
        R = 6378137.0

        # Convert degrees -> radians
        rad_home_lat = np.radians(self.home_lat)
        rad_target_lat = np.radians(target_lat)
        rad_dlat = rad_target_lat - rad_home_lat
        rad_dlon = np.radians(target_lon) - np.radians(self.home_lon)

        # Approximate local planar offsets
        dNorth = rad_dlat * R
        dEast = rad_dlon * R * np.cos(rad_home_lat)

        # x, y in local frame (assuming x=East, y=North, z=Down)
        x_local = dEast
        y_local = dNorth

        # If your local Z is "down," then a positive difference in altitude
        # means you want a lower (more negative) z setpoint.
        # E.g. local_z = -(target_alt - home_alt).
        z_local = -(target_alt - self.home_alt)

        return x_local, y_local, z_local

    # ----------------------------------------------------------------------------
    # Mission / Timer Logic
    # ----------------------------------------------------------------------------
    def check_takeoff_complete(self) -> bool:
        """Check if we have reached takeoff altitude and hovered briefly."""
        height_reached = abs(self.vehicle_local_position.z - self.takeoff_height) < 0.5
        if height_reached and not self.takeoff_complete:
            self.hover_time += 1
            if self.hover_time >= 20:  # ~2 seconds if timer is 0.1s
                self.takeoff_complete = True
                self.get_logger().info("Takeoff complete, starting mission")
                return True
        return False

    def timer_callback(self) -> None:
        """Main control loop."""
        self.publish_offboard_control_heartbeat_signal()

        # Ensure we have local position data
        if not self.vehicle_local_position:
            self.get_logger().info("No local position data yet...")
            return

        # Ensure we have at least one global position fix (to set home)
        if self.home_lat is None:
            self.get_logger().info("No global position fix yet...")
            return

        # On the 10th iteration, engage offboard and arm
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.get_logger().info("Vehicle armed")

        # If in offboard mode, proceed
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # 1) Handle takeoff
            if not self.takeoff_complete:
                self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
                self.check_takeoff_complete()

            # 2) Navigate to your desired GPS waypoint
            elif not self.mission_completed:
                # Convert your desired lat/lon/alt to local (x,y,z) once:
                if (self.local_target_x == 0.0 and
                    self.local_target_y == 0.0 and
                    self.local_target_z == 0.0):
                    (self.local_target_x,
                     self.local_target_y,
                     self.local_target_z) = self.gps_to_local_waypoint(
                         self.target_lat, self.target_lon, self.target_alt
                     )
                    self.get_logger().info(
                        f"Target local waypoint: x={self.local_target_x:.1f}, "
                        f"y={self.local_target_y:.1f}, z={self.local_target_z:.1f}"
                    )

                # Now publish that position setpoint
                self.publish_position_setpoint(
                    self.local_target_x,
                    self.local_target_y,
                    self.local_target_z
                )

                # Check if we are close to the waypoint
                dist = np.sqrt(
                    (self.vehicle_local_position.x - self.local_target_x)**2 +
                    (self.vehicle_local_position.y - self.local_target_y)**2 +
                    (self.vehicle_local_position.z - self.local_target_z)**2
                )
                threshold = 2.0
                if dist < threshold:
                    self.mission_completed = True
                    self.get_logger().info("Reached target GPS location; landing next.")

            else:
                # 3) Land when mission is complete
                self.land()
                self.get_logger().info("Landing initiated")

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting offboard control node, flying to a GPS waypoint...')
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
