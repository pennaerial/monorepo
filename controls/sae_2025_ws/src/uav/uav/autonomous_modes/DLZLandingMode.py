"""
DLZ Landing Mode - Precision landing on Drop/Landing Zone.

Implements a multi-phase precision landing algorithm with PID control
for smooth centering over the DLZ marker before descent.
"""

from enum import Enum
from typing import Dict, Optional, Tuple
import numpy as np
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus

from uav import UAV
from uav.autonomous_modes import Mode
from uav_interfaces.srv import PayloadTracking
from uav.vision_nodes import PayloadTrackingNode


class LandingPhase(Enum):
    """Landing phase state machine states."""
    APPROACH = "approach"   # High altitude (>3m) - fast approach to DLZ area
    ALIGN = "align"         # Medium altitude (1-3m) - precise centering over DLZ
    DESCEND = "descend"     # Low altitude (0.5-1m) - slow descent while centered
    FINAL = "final"         # Very low (<0.5m) - trigger auto-land or touchdown
    LANDED = "landed"       # Monitor for landing complete


class PIDController:
    """PID controller for smooth position control with anti-windup."""

    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float = 1.0):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            integral_limit: Anti-windup limit for integral term
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error: float, dt: float) -> float:
        """
        Compute PID output for given error.

        Args:
            error: Current error value
            dt: Time delta since last update

        Returns:
            Control output value
        """
        # Accumulate error over time (with anti-windup)
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        # Rate of change of error
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        # Combined output
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0


class DLZLandingMode(Mode):
    """
    Precision landing mode for landing in the center of the DLZ.

    Implements:
    - PID control for smooth horizontal positioning
    - Multi-phase state machine for controlled descent
    - Altitude-dependent descent rates
    - Robust detection loss handling
    """

    # Default descent rates for each phase (m/s)
    DEFAULT_DESCENT_RATES = {
        'approach': 1.0,   # Fast descent while far
        'align': 0.3,      # Slow, focusing on centering
        'descend': 0.2,    # Careful descent
        'final': 0.1       # Very slow final approach
    }

    # Phase altitude thresholds (meters)
    ALTITUDE_APPROACH = 3.0    # Above this: approach phase
    ALTITUDE_ALIGN = 1.0       # Above this (below approach): align phase
    ALTITUDE_DESCEND = 0.5     # Above this (below align): descend phase
    ALTITUDE_LANDED = 0.1      # Below this: consider landed

    # Stability thresholds (radians)
    MAX_ROLL = 0.1
    MAX_PITCH = 0.1

    def __init__(
        self,
        node: Node,
        uav: UAV,
        pid_gains_xy: Tuple[float, float, float] = (0.5, 0.01, 0.1),
        descent_rates: Optional[Dict[str, float]] = None,
        centering_threshold: float = 0.15,
        detection_timeout: float = 2.0,
        use_auto_land: bool = True
    ):
        """
        Initialize DLZ Landing Mode.

        Args:
            node: ROS 2 node managing the UAV
            uav: The UAV instance to control
            pid_gains_xy: Tuple of (Kp, Ki, Kd) gains for X/Y PID controllers
            descent_rates: Dict mapping phase names to descent rates (m/s)
            centering_threshold: Distance threshold (m) to consider centered
            detection_timeout: Seconds to wait before aborting on lost detection
            use_auto_land: Whether to use PX4 auto-land for final phase
        """
        super().__init__(node, uav)

        # PID controllers for X and Y positioning
        kp, ki, kd = pid_gains_xy
        self.pid_x = PIDController(kp, ki, kd)
        self.pid_y = PIDController(kp, ki, kd)

        # Configuration
        self.descent_rates = descent_rates or self.DEFAULT_DESCENT_RATES.copy()
        self.centering_threshold = centering_threshold
        self.detection_timeout = detection_timeout
        self.use_auto_land = use_auto_land

        # State tracking
        self.phase = LandingPhase.APPROACH
        self.last_valid_detection_time: Optional[float] = None
        self.last_valid_direction: Optional[Tuple[float, float, float]] = None
        self.elapsed_time = 0.0
        self.error_condition = False
        self.landing_triggered = False

    def on_enter(self) -> None:
        """Initialize state when mode is activated."""
        self.phase = LandingPhase.APPROACH
        self.pid_x.reset()
        self.pid_y.reset()
        self.last_valid_detection_time = None
        self.last_valid_direction = None
        self.elapsed_time = 0.0
        self.error_condition = False
        self.landing_triggered = False
        self.log(f"DLZ Landing Mode activated - starting in {self.phase.value} phase")

    def on_update(self, time_delta: float) -> None:
        """
        Main update loop for precision landing.

        Args:
            time_delta: Time in seconds since last update
        """
        self.elapsed_time += time_delta

        # Handle post-landing states
        if self.phase == LandingPhase.LANDED:
            return

        if self.phase == LandingPhase.FINAL and self.landing_triggered:
            self._handle_final_phase()
            return

        # Check UAV stability - skip update if unstable
        if abs(self.uav.roll) > self.MAX_ROLL or abs(self.uav.pitch) > self.MAX_PITCH:
            self.log("Roll or pitch exceeded threshold. Waiting for stabilization.")
            return

        # Get current altitude (negative z in NED frame)
        local_pos = self.uav.get_local_position()
        if local_pos is None:
            self.log("No local position available")
            return
        altitude = -local_pos[2]

        # Request vision tracking
        request = PayloadTracking.Request()
        request.altitude = altitude
        request.yaw = float(self.uav.yaw)
        request.payload_color = 'pink'  # DLZ marker color
        response = self.send_request(PayloadTrackingNode, request)

        # Handle response (async - may be None if still processing)
        if response is None:
            return

        # Process detection result
        detection_valid = self._is_detection_valid(response, altitude)

        if detection_valid:
            self._handle_valid_detection(response, altitude, time_delta)
        else:
            self._handle_lost_detection(altitude)

    def _is_detection_valid(self, response, altitude: float) -> bool:
        """
        Check if the detection response is valid.

        A detection is invalid if the direction vector magnitude is very small
        or if the response indicates no detection was made.
        """
        direction = response.direction
        # Check for valid direction (non-zero magnitude)
        magnitude = np.sqrt(direction[0]**2 + direction[1]**2)
        return magnitude > 0.001

    def _handle_valid_detection(
        self,
        response,
        altitude: float,
        time_delta: float
    ) -> None:
        """Process a valid detection and compute control outputs."""
        # Update detection tracking
        self.last_valid_detection_time = self.elapsed_time

        # Transform direction from camera to local frame
        # Camera: x right, y down -> Local NED: x north, y east
        direction = [-response.direction[1], response.direction[0], response.direction[2]]

        # Apply camera offset correction
        camera_offsets = self.uav.camera_offsets
        if altitude > 1:
            camera_offsets = tuple(x / altitude for x in camera_offsets)
        local_camera_offsets = self.uav.uav_to_local(camera_offsets)
        direction = [d + c for d, c in zip(direction, local_camera_offsets)]

        self.last_valid_direction = tuple(direction)

        # Compute PID outputs for horizontal position
        error_x = direction[0]
        error_y = direction[1]

        velocity_x = self.pid_x.update(error_x, time_delta)
        velocity_y = self.pid_y.update(error_y, time_delta)

        # Check if centered
        horizontal_error = np.sqrt(error_x**2 + error_y**2)
        is_centered = horizontal_error < self.centering_threshold

        # Update phase based on altitude and centering
        self._update_phase(altitude, is_centered)

        # Determine descent rate based on phase and centering
        if is_centered:
            descent_rate = self.descent_rates.get(self.phase.value, 0.0)
        else:
            descent_rate = 0.0  # Hold altitude until centered

        # Compute position setpoint
        # Scale PID outputs for reasonable movement (clamp to prevent large jumps)
        max_horizontal_step = 0.5  # meters per update
        velocity_x = np.clip(velocity_x, -max_horizontal_step, max_horizontal_step)
        velocity_y = np.clip(velocity_y, -max_horizontal_step, max_horizontal_step)

        # Create relative setpoint (descent is positive z in our direction convention)
        setpoint = [velocity_x, velocity_y, descent_rate * time_delta]

        self.log(
            f"Phase: {self.phase.value} | Error: ({error_x:.3f}, {error_y:.3f}) | "
            f"Centered: {is_centered} | Alt: {altitude:.2f}m | "
            f"PID out: ({velocity_x:.3f}, {velocity_y:.3f})"
        )

        # Publish setpoint or trigger landing
        if self.phase == LandingPhase.FINAL and is_centered:
            self._trigger_final_landing()
        else:
            self.uav.publish_position_setpoint(setpoint, relative=True)

    def _handle_lost_detection(self, altitude: float) -> None:
        """Handle case when DLZ detection is lost."""
        if self.last_valid_detection_time is None:
            # Never had a valid detection - just hover
            self.uav.hover()
            self.log("No detection yet, hovering...")
            return

        time_since_detection = self.elapsed_time - self.last_valid_detection_time

        if time_since_detection < self.detection_timeout:
            # Within timeout - hold position and wait
            self.uav.hover()
            self.log(
                f"Detection lost, holding position ({time_since_detection:.1f}s / "
                f"{self.detection_timeout:.1f}s timeout)"
            )
        else:
            # Timeout exceeded - set error condition
            self.log(f"Detection lost for {time_since_detection:.1f}s - aborting landing")
            self.error_condition = True

    def _update_phase(self, altitude: float, is_centered: bool) -> None:
        """Update landing phase based on altitude and centering status."""
        previous_phase = self.phase

        if altitude > self.ALTITUDE_APPROACH:
            self.phase = LandingPhase.APPROACH
        elif altitude > self.ALTITUDE_ALIGN:
            # Can only transition to align if coming from approach
            if self.phase == LandingPhase.APPROACH:
                self.phase = LandingPhase.ALIGN
        elif altitude > self.ALTITUDE_DESCEND:
            # Can only transition to descend if centered and coming from align
            if self.phase == LandingPhase.ALIGN and is_centered:
                self.phase = LandingPhase.DESCEND
        else:
            # Below descend altitude - go to final if centered
            if self.phase in (LandingPhase.ALIGN, LandingPhase.DESCEND) and is_centered:
                self.phase = LandingPhase.FINAL

        if self.phase != previous_phase:
            self.log(f"Phase transition: {previous_phase.value} -> {self.phase.value}")
            # Reset PID on phase transition for smoother control
            self.pid_x.reset()
            self.pid_y.reset()

    def _trigger_final_landing(self) -> None:
        """Trigger the final landing sequence."""
        if self.landing_triggered:
            return

        self.log("Triggering final landing sequence")
        self.landing_triggered = True

        if self.use_auto_land:
            self.uav.land()
        else:
            # Continue manual descent
            pass

    def _handle_final_phase(self) -> None:
        """Monitor final landing phase for completion."""
        # Check if auto-land is complete
        if self.use_auto_land:
            # Check if no longer in auto-land state (landing complete)
            if self.uav.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self.phase = LandingPhase.LANDED
                self.log("Landing complete")
                return

        # Alternative: check altitude for touchdown
        local_pos = self.uav.get_local_position()
        if local_pos is not None:
            altitude = -local_pos[2]
            if altitude < self.ALTITUDE_LANDED:
                self.phase = LandingPhase.LANDED
                self.log("Landing complete (altitude threshold)")

    def check_status(self) -> str:
        """
        Check the status of the landing.

        Returns:
            'complete' if landed successfully
            'error' if an error occurred
            'continue' if still in progress
        """
        if self.phase == LandingPhase.LANDED:
            return 'complete'
        if self.error_condition:
            return 'error'
        return 'continue'

    def on_exit(self) -> None:
        """Cleanup when mode is deactivated."""
        self.log(f"DLZ Landing Mode deactivated in phase: {self.phase.value}")
