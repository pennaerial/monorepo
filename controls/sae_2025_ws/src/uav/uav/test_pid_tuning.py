#!/usr/bin/env python3
"""
Ziegler-Nichols PID Tuning Test Script for DLZ Landing Mode.

This script helps tune the PID gains for horizontal position control
by allowing you to test different Kp values and observe the response.

Usage:
    ros2 run uav test_pid_tuning --ros-args -p kp:=0.1

Procedure:
    1. Run with initial Kp=0.1
    2. Observe response when UAV attempts to center over DLZ
    3. Increase Kp until sustained oscillations occur (that's Ku)
    4. Measure oscillation period (Tu)
    5. Calculate final gains using Ziegler-Nichols formulas
"""

import rclpy
from rclpy.node import Node
from uav.UAV import UAV
from uav.autonomous_modes.DLZLandingMode import DLZLandingMode, PIDController
from px4_msgs.msg import VehicleStatus
import numpy as np
import time
from datetime import datetime


class PIDTuningNode(Node):
    """Node for testing PID tuning with Ziegler-Nichols method."""

    # Test configuration
    DLZ_POSITION = (-5.0, 0.0)  # DLZ marker position in local frame (x, y)
    TEST_ALTITUDE = 3.0         # Altitude for tuning tests (ALIGN phase)
    OFFSET_DISTANCE = 1.5       # How far to offset from DLZ for testing (meters)

    def __init__(self) -> None:
        super().__init__('pid_tuning_test')

        # Declare parameters
        self.declare_parameter('kp', 0.1)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('log_interval', 0.5)  # Log every N seconds

        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.log_interval = self.get_parameter('log_interval').value

        self.get_logger().info(f"=== PID Tuning Test ===")
        self.get_logger().info(f"Gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")

        # Initialize UAV
        self.uav = UAV(self, takeoff_amount=self.TEST_ALTITUDE, DEBUG=True)

        # State machine
        self.state = "INIT"
        self.offboard_counter = 0
        self.dlz_mode = None

        # Timing and logging
        self.last_log_time = 0.0
        self.test_start_time = None
        self.position_history = []  # For oscillation detection
        self.peak_times = []        # For measuring Tu
        self.last_error_x = 0.0
        self.last_error_sign = 0

        # Create timer (10 Hz - matches ModeManager)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_update_time = time.time()

        self.get_logger().info("Waiting for UAV initialization...")

    def timer_callback(self) -> None:
        """Main control loop."""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Always send heartbeat
        self.uav.publish_offboard_control_heartbeat_signal()

        # Wait for vehicle data
        if not self.uav.local_position or not self.uav.global_position:
            return

        # State machine
        if self.state == "INIT":
            self._handle_init_state()
        elif self.state == "ARMING":
            self._handle_arming_state()
        elif self.state == "TAKEOFF":
            self._handle_takeoff_state()
        elif self.state == "NAVIGATE_TO_TEST":
            self._handle_navigate_state()
        elif self.state == "OFFSET":
            self._handle_offset_state()
        elif self.state == "TESTING":
            self._handle_testing_state(dt)
        elif self.state == "COMPLETE":
            self._handle_complete_state()

    def _handle_init_state(self) -> None:
        """Initialize and prepare for arming."""
        self.offboard_counter += 1
        if self.offboard_counter >= 10:
            self.state = "ARMING"
            self.get_logger().info("Initialization complete, arming...")

    def _handle_arming_state(self) -> None:
        """Arm the vehicle and set origin."""
        if not self.uav.origin_set:
            self.uav.arm()
            self.uav.set_origin()
        elif self.uav.arm_state == VehicleStatus.ARMING_STATE_ARMED:
            self.state = "TAKEOFF"
            self.get_logger().info("Armed! Taking off...")

    def _handle_takeoff_state(self) -> None:
        """Execute takeoff."""
        if not self.uav.attempted_takeoff:
            self.uav.takeoff()
        elif self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            # Takeoff complete, switch to offboard
            self.uav.engage_offboard_mode()
            self.state = "NAVIGATE_TO_TEST"
            self.get_logger().info(f"Takeoff complete! Navigating to test position above DLZ...")

    def _handle_navigate_state(self) -> None:
        """Navigate to position above DLZ marker."""
        if self.uav.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return

        # Target: above DLZ at test altitude
        target = [self.DLZ_POSITION[0], self.DLZ_POSITION[1], -self.TEST_ALTITUDE]
        self.uav.publish_position_setpoint(target, relative=False)

        # Check if we're at the target
        local_pos = self.uav.get_local_position()
        if local_pos:
            dist = np.sqrt(
                (local_pos[0] - target[0])**2 +
                (local_pos[1] - target[1])**2
            )
            if dist < 0.5:  # Within 0.5m horizontally
                self.state = "OFFSET"
                self.get_logger().info(f"Above DLZ! Now offsetting {self.OFFSET_DISTANCE}m for test...")

    def _handle_offset_state(self) -> None:
        """Offset from DLZ to create initial error for testing."""
        # Offset in +X direction from DLZ
        offset_target = [
            self.DLZ_POSITION[0] + self.OFFSET_DISTANCE,
            self.DLZ_POSITION[1],
            -self.TEST_ALTITUDE
        ]
        self.uav.publish_position_setpoint(offset_target, relative=False)

        local_pos = self.uav.get_local_position()
        if local_pos:
            dist = np.sqrt(
                (local_pos[0] - offset_target[0])**2 +
                (local_pos[1] - offset_target[1])**2
            )
            if dist < 0.3:
                self._start_pid_test()

    def _start_pid_test(self) -> None:
        """Initialize DLZ landing mode for PID testing."""
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"STARTING PID TEST - Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Watch for oscillations as UAV centers over DLZ marker")
        self.get_logger().info("")

        # Create DLZ landing mode with test gains
        self.dlz_mode = DLZLandingMode(
            node=self,
            uav=self.uav,
            pid_gains_xy=(self.kp, self.ki, self.kd),
            centering_threshold=0.15,
            detection_timeout=5.0,
            use_auto_land=False  # Don't actually land during tuning
        )
        self.dlz_mode.on_enter()

        self.test_start_time = time.time()
        self.state = "TESTING"

    def _handle_testing_state(self, dt: float) -> None:
        """Run the PID test and log response."""
        if self.dlz_mode is None:
            return

        # Update DLZ mode
        self.dlz_mode.on_update(dt)

        # Get current position for analysis
        local_pos = self.uav.get_local_position()
        if local_pos is None:
            return

        # Calculate error from DLZ center
        error_x = local_pos[0] - self.DLZ_POSITION[0]
        error_y = local_pos[1] - self.DLZ_POSITION[1]
        horizontal_error = np.sqrt(error_x**2 + error_y**2)

        # Track position for oscillation analysis
        elapsed = time.time() - self.test_start_time
        self.position_history.append((elapsed, error_x, error_y))

        # Detect zero crossings (for measuring oscillation period)
        current_sign = 1 if error_x > 0 else -1
        if self.last_error_sign != 0 and current_sign != self.last_error_sign:
            self.peak_times.append(elapsed)
            if len(self.peak_times) >= 2:
                period = (self.peak_times[-1] - self.peak_times[-2]) * 2  # Full cycle
                self.get_logger().info(f"  -> Zero crossing detected! Half-period: {period/2:.3f}s")
        self.last_error_sign = current_sign

        # Periodic logging
        if elapsed - self.last_log_time >= self.log_interval:
            self.last_log_time = elapsed
            self._log_status(elapsed, error_x, error_y, horizontal_error)

        # Check for test completion (centered for 3 seconds or 30 seconds elapsed)
        if horizontal_error < 0.1 and elapsed > 5.0:
            self.get_logger().info("UAV centered - no sustained oscillation at this Kp")
            self._finish_test("CENTERED")
        elif elapsed > 30.0:
            self._finish_test("TIMEOUT")

    def _log_status(self, elapsed: float, error_x: float, error_y: float,
                    horizontal_error: float) -> None:
        """Log current test status."""
        # Get PID internal state
        pid_integral_x = self.dlz_mode.pid_x.integral if self.dlz_mode else 0
        pid_integral_y = self.dlz_mode.pid_y.integral if self.dlz_mode else 0

        self.get_logger().info(
            f"[{elapsed:5.1f}s] Error: ({error_x:+.3f}, {error_y:+.3f}) | "
            f"Dist: {horizontal_error:.3f}m | "
            f"Integral: ({pid_integral_x:+.3f}, {pid_integral_y:+.3f})"
        )

    def _finish_test(self, reason: str) -> None:
        """Finish the test and report results."""
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"TEST COMPLETE - Reason: {reason}")
        self.get_logger().info("=" * 60)

        # Analyze oscillations
        if len(self.peak_times) >= 4:
            # Calculate average period from zero crossings
            periods = []
            for i in range(2, len(self.peak_times)):
                periods.append((self.peak_times[i] - self.peak_times[i-2]))
            avg_period = np.mean(periods) if periods else 0
            self.get_logger().info(f"Detected {len(self.peak_times)} zero crossings")
            self.get_logger().info(f"Estimated oscillation period (Tu): {avg_period:.3f}s")
            self.get_logger().info("")
            self.get_logger().info("If oscillations were SUSTAINED (constant amplitude):")
            self.get_logger().info(f"  Ku = {self.kp}")
            self.get_logger().info(f"  Tu = {avg_period:.3f}s")
            self._calculate_zn_gains(self.kp, avg_period)
        else:
            self.get_logger().info("Not enough oscillations detected")
            self.get_logger().info(f"Try increasing Kp (current: {self.kp})")
            self.get_logger().info(f"Suggested next value: Kp={self.kp + 0.1:.1f}")

        self.get_logger().info("")
        self.state = "COMPLETE"

    def _calculate_zn_gains(self, ku: float, tu: float) -> None:
        """Calculate and display Ziegler-Nichols gains."""
        if tu <= 0:
            return

        # No-overshoot Ziegler-Nichols formulas
        kp = 0.2 * ku
        ki = 0.4 * ku / tu
        kd = 0.066 * ku * tu

        self.get_logger().info("")
        self.get_logger().info("Ziegler-Nichols No-Overshoot Gains:")
        self.get_logger().info(f"  Kp = 0.2 × {ku} = {kp:.4f}")
        self.get_logger().info(f"  Ki = 0.4 × {ku} / {tu:.3f} = {ki:.4f}")
        self.get_logger().info(f"  Kd = 0.066 × {ku} × {tu:.3f} = {kd:.4f}")
        self.get_logger().info("")
        self.get_logger().info("To apply these gains, update DLZLandingMode.py line 110:")
        self.get_logger().info(f"  pid_gains_xy: Tuple[float, float, float] = ({kp:.4f}, {ki:.4f}, {kd:.4f}),")

    def _handle_complete_state(self) -> None:
        """Hover after test completion."""
        self.uav.hover()


def main(args=None) -> None:
    print("=" * 60)
    print("Ziegler-Nichols PID Tuning Test for DLZ Landing")
    print("=" * 60)
    print()
    print("Usage: ros2 run uav test_pid_tuning --ros-args -p kp:=<value>")
    print()

    rclpy.init(args=args)
    node = PIDTuningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
