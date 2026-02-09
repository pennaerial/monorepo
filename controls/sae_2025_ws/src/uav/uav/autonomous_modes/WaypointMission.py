#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
import math
import time
from typing import List
from .Mode import Mode


class WaypointMission(Mode):
    """
    Simple waypoint mission for testing scoring node.
    Flies through a series of waypoints defined in YAML.
    """

    def __init__(
        self,
        node: Node,
        uav,
        waypoints: List[List[float]] = None,
        waypoint_tolerance: float = 0.5,
        speed: float = 1.0,
        loiter_time: float = 1.0,
        altitude: float = 2.0,
    ):
        super().__init__(node, uav)

        # Mission parameters
        self.waypoints = waypoints or []
        self.current_waypoint = 0
        self.waypoint_tolerance = waypoint_tolerance
        self.speed = speed
        self.loiter_time = loiter_time
        self.altitude = altitude
        self.mission_active = False
        self.waypoint_start_time = None

        # UAV state
        self.offboard_setpoint_counter = 0
        # Mission timer
        self.start_timer = self.node.create_timer(5.0, self.start_mission)

        self.node.get_logger().info("Waypoint Mission initialized")

    def on_enter(self):
        """Called when mode is activated."""
        self.node.get_logger().info("Waypoint Mission activated")
        self.mission_active = True

        # Enable offboard mode
        self.publish_offboard_control_mode()

        # Arm the vehicle
        self.arm_vehicle()

    def on_exit(self):
        """Called when mode is deactivated."""
        self.node.get_logger().info("Waypoint Mission deactivated")
        self.mission_active = False

    def on_update(self, time_delta: float):
        """Periodic update called by ModeManager."""
        if not self.mission_active or not self.uav.local_position:
            return

        if self.current_waypoint >= len(self.waypoints):
            self.node.get_logger().info("Mission completed!")
            self.mission_active = False
            return

        target = self.waypoints[self.current_waypoint]
        target_x, target_y, target_z = target

        current_x, current_y, current_z = (
            self.uav.local_position.x,
            self.uav.local_position.y,
            self.uav.local_position.z,
        )
        distance = math.sqrt(
            (current_x - target_x) ** 2
            + (current_y - target_y) ** 2
            + (current_z - target_z) ** 2
        )

        # Check if we've reached the current waypoint
        if distance < self.waypoint_tolerance:
            if self.waypoint_start_time is None:
                self.waypoint_start_time = time.time()
                self.node.get_logger().info(
                    f"Reached waypoint {self.current_waypoint}: {target}"
                )
            elif time.time() - self.waypoint_start_time >= self.loiter_time:
                self.node.get_logger().info(
                    f"Completed waypoint {self.current_waypoint}"
                )
                self.current_waypoint += 1
                self.waypoint_start_time = None
        else:
            # Reset waypoint timer if we're not at the waypoint
            self.waypoint_start_time = None

        # Publish trajectory setpoint
        setpoint = TrajectorySetpoint()
        setpoint.position = [float(target_x), float(target_y), float(target_z)]
        setpoint.yaw = 0.0  # Keep yaw at 0
        self.uav.publish_position_setpoint((target_x, target_y, target_z))

        # Log progress every 10 iterations
        if self.current_waypoint % 10 == 0:
            self.node.get_logger().info(
                f"Moving to waypoint {self.current_waypoint}: {target}, distance: {distance:.2f}"
            )

    def check_status(self) -> str:
        """Check if the mode should deactivate."""
        if not self.mission_active:
            return "complete"
        if self.current_waypoint >= len(self.waypoints):
            return "complete"
        return "continue"

    def start_mission(self):
        """Start the mission after initial delay."""
        self.mission_active = True
        self.start_timer.cancel()
        self.node.get_logger().info("Starting waypoint mission!")

        # Enable offboard mode
        self.publish_offboard_control_mode()

        # Arm the vehicle
        self.arm_vehicle()

    def publish_offboard_control_mode(self):
        """Publish offboard control mode."""
        self.uav.publish_offboard_control_heartbeat_signal()

    def arm_vehicle(self):
        """Arm the vehicle."""
        self.uav.arm()
        self.node.get_logger().info("Arming vehicle...")


def main(args=None):
    rclpy.init(args=args)

    # This is for standalone testing - not used in ModeManager
    node = Node("waypoint_mission_standalone")

    # Create a dummy UAV for testing
    class DummyUAV:
        def publish_position_setpoint(self, pos):
            pass

        def publish_offboard_control_mode(self):
            pass

        def arm(self):
            pass

    uav = DummyUAV()

    # Test waypoints
    test_waypoints = [
        [0, 0, 2],  # Takeoff
        [2, 0, 2],  # Hoop 1
        [4, 0, 2],  # Hoop 2
        [6, 0, 2],  # Hoop 3
        [8, 0, 2],  # Hoop 4
        [10, 0, 2],  # Hoop 5
        [0, 0, 2],  # Return
        [0, 0, 0],  # Land
    ]

    WaypointMission(node, uav, waypoints=test_waypoints)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
