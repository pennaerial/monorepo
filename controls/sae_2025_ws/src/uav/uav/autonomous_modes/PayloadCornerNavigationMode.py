"""
PayloadCornerNavigationMode - Navigate payload to a corner of the DLZ.

After being dropped off, the payload searches for an available corner by:
1. (Future) Detecting AprilTag on existing payload to identify occupied corners
2. Sampling pink/green color ratios in multiple directions
3. Moving in the direction with the least pink (most green/grass)
4. Stopping when pink ratio exceeds threshold (corner reached)
"""

import numpy as np
import cv2
from enum import Enum
from typing import Optional
from rclpy.node import Node
from sensor_msgs.msg import Image
from payload_interfaces.msg import DriveCommand

from uav import UAV
from uav.autonomous_modes import Mode
from uav.utils import pink

try:
    import apriltag
except ImportError:
    apriltag = None


class CornerNavigationPhase(Enum):
    """State machine phases for corner navigation."""

    INITIAL_SCAN = "initial_scan"  # Detect obstacles (AprilTags) ahead
    ORBIT_OBSTACLE = (
        "orbit_obstacle"  # Navigate around detected obstacle using hardcoded path
    )
    NAVIGATE_TO_EDGE = "navigate_to_edge"  # Move toward nearest edge
    RESCAN_TO_EDGE = "rescan_to_edge"  # Periodically rescan while moving to edge
    CENTER_ON_EDGE = "center_on_edge"  # Align perpendicular to edge
    SCAN_FOR_CORNERS = "scan_for_corners"  # Turn left/right to find nearest corner
    NAVIGATE_TO_CORNER = "navigate_to_corner"  # Move along edge to corner
    TURN_TO_CENTER = "turn_to_center"  # Turn 135° to face center
    CORNER_REACHED = "corner_reached"  # At corner, done


class PayloadCornerNavigationMode(Mode):
    """
    Navigate payload from center of DLZ to an available corner.

    Uses color ratio detection (pink DLZ vs green grass) to find the corner
    by moving in the direction with the least pink (most grass), indicating
    movement toward the edge/corner of the DLZ.
    """

    def __init__(
        self,
        node: Node,
        uav: UAV,
        payload_name: str = "payload_0",
        corner_threshold: float = 0.25,
        linear_velocity: float = 0.3,
        angular_velocity: float = 0.5,
        scan_span: float = np.radians(30),
        scan_interval: float = np.radians(5),
        forward_time: float = 2.0,
        rescan_interval: float = 3.0,
        min_color_pixels: int = 100,
        centering_tolerance: float = 0.05,
        force_obstacle_detected: bool = False,
    ):
        """
        Initialize the PayloadCornerNavigationMode.

        Args:
            node: ROS 2 node managing the mode
            uav: The UAV instance (not directly used for payload control)
            payload_name: Name of the payload for topic namespacing
            corner_threshold: Pink ratio below which edge/corner is considered reached
            linear_velocity: Forward velocity for the payload in m/s
            angular_velocity: Turn velocity in rad/s
            scan_span: Angular span to scan on each side (radians, default 30°)
            scan_interval: Angular interval between scan points (radians, default 5°)
            forward_time: Time to move forward between rescans (seconds)
            rescan_interval: Time between directional rescans (seconds)
            min_color_pixels: Minimum combined pink+green pixels for valid detection
            centering_tolerance: Max pink ratio difference for being centered on edge
            force_obstacle_detected: Force obstacle detection to test orbiting behavior
        """
        super().__init__(node, uav)

        self.payload_name = payload_name
        self.corner_threshold = float(corner_threshold)
        self.linear_velocity = float(linear_velocity)
        self.angular_velocity = float(angular_velocity)
        self.scan_span = float(scan_span)
        self.scan_interval = float(scan_interval)
        self.force_obstacle_detected = bool(force_obstacle_detected)

        # Calculate number of scan points based on span and interval
        # For span=30° and interval=5°: (-30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30) = 13 points
        self.scan_points = int((2 * self.scan_span / self.scan_interval) + 1)

        self.forward_time = float(forward_time)
        self.rescan_interval = float(rescan_interval)
        self.min_color_pixels = int(min_color_pixels)
        self.centering_tolerance = float(centering_tolerance)

        # HSV color ranges (same as PayloadEdgeNavigationMode)
        self.pink_lower = np.array(pink[0])
        self.pink_upper = np.array(pink[1])
        # Wider green range for Gazebo grass
        self.green_lower = np.array([25, 20, 20])
        self.green_upper = np.array([90, 255, 255])

        # State tracking
        self.current_image: Optional[Image] = None
        self.phase = CornerNavigationPhase.INITIAL_SCAN
        self.error_state = False

        # Scanning state
        self.scan_index = 0
        self.scan_ratios = []  # List of (angle, pink_ratio) tuples
        self.best_direction_angle = 0.0  # Angle (radians) of best direction
        self.current_heading = 0.0  # Track current heading
        self.corner_direction = (
            0.0  # Which direction we turned to go to corner (for final turn)
        )

        # Navigation state
        self.forward_elapsed = 0.0
        self.rescan_elapsed = 0.0
        self.corner_movement_elapsed = 0.0  # Time spent moving to corner
        self.is_turning = False
        self.turn_target = 0.0
        self.turn_elapsed = 0.0
        self.turn_duration = 0.0

        # Coast compensation for turn accuracy
        self.coast_angle_left = np.radians(4.0)  # 4° coast for left turns
        self.coast_angle_right = np.radians(
            10.0
        )  # 10° coast for right turns (more overshoot)
        self.settling_time = 0.15  # 150ms to settle after stopping
        self.turn_settling_elapsed = 0.0

        # Centering state
        self.centering_scan_index = 0
        self.centering_measurements = []  # List of (angle, balance_diff) tuples

        # Detection tracking
        self.frames_without_valid_detection = 0
        self.max_frames_without_detection = 50  # ~5 seconds at 10Hz

        # Obstacle detection and orbiting
        self.obstacle_detected = False
        self.orbit_direction = 0  # +1 for left (CCW), -1 for right (CW)
        self.orbit_step = 0  # Current step in orbit sequence
        # Aircraft dimensions: width=0.343m, length=1.080m
        self.aircraft_width = 0.343
        self.aircraft_length = 1.080

        # Publishers and subscribers (initialized in on_enter)
        self.drive_publisher = None
        self.camera_subscriber = None

        # AprilTag detection for detecting existing payloads
        self.apriltag_detector = None
        if apriltag is not None:
            try:
                options = apriltag.DetectorOptions(
                    families="tag36h11",
                    refine_edges=True,
                )
                self.apriltag_detector = apriltag.Detector(options)
            except (AttributeError, TypeError):
                # Fallback for older apriltag versions
                self.apriltag_detector = apriltag.Detector()

        self.occupied_corners = []  # List of corner positions with existing payloads
        self.selected_corner = None  # Chosen target corner

    def on_enter(self) -> None:
        """Initialize mode - set up publishers and subscribers."""
        # Create publisher for payload drive commands
        drive_topic = f"/{self.payload_name}/cmd_drive"
        self.drive_publisher = self.node.create_publisher(DriveCommand, drive_topic, 10)
        self.log(f"Created drive publisher on {drive_topic}")

        # Subscribe to payload camera
        camera_topic = f"/{self.payload_name}/camera"
        self.camera_subscriber = self.node.create_subscription(
            Image, camera_topic, self._camera_callback, 10
        )
        self.log(f"Subscribed to camera on {camera_topic}")

        # Phase already set to INITIAL_SCAN in __init__
        self.scan_index = 0
        self.scan_ratios = []
        self.current_heading = 0.0

        # Log AprilTag availability
        if self.apriltag_detector is not None:
            self.log("PayloadCornerNavigationMode activated with AprilTag detection")
        else:
            if apriltag is None:
                self.log(
                    "PayloadCornerNavigationMode activated - apriltag not installed"
                )
            else:
                self.log(
                    "PayloadCornerNavigationMode activated - apriltag failed to initialize"
                )

        self.log("PayloadCornerNavigationMode activated - starting corner search")

    def _camera_callback(self, msg: Image) -> None:
        """Store the latest camera image."""
        self.current_image = msg

    def _convert_image_msg_to_frame(self, msg: Image) -> np.ndarray:
        """Convert ROS Image message to OpenCV BGR frame."""
        img_data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = img_data.reshape((msg.height, msg.width, 3))
        # Gazebo sends RGB, OpenCV expects BGR
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    def _calculate_color_ratio(self, frame: np.ndarray) -> Optional[float]:
        """
        Calculate the ratio of pink (DLZ) to total colored pixels (pink + green).

        Returns:
            float: Ratio of pink pixels, or None if insufficient color detected
        """
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create masks for pink and green
        pink_mask = cv2.inRange(hsv, self.pink_lower, self.pink_upper)
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)

        # Count pixels
        pink_pixels = cv2.countNonZero(pink_mask)
        green_pixels = cv2.countNonZero(green_mask)
        total_colored = pink_pixels + green_pixels

        # Check if we have enough colored pixels for valid detection
        if total_colored < self.min_color_pixels:
            return None

        # Return pink ratio
        return pink_pixels / total_colored

    def _detect_edge_position(self, frame: np.ndarray) -> Optional[float]:
        """
        Detect the horizontal position of the pink/green edge in the frame.

        Returns:
            float: Normalized edge position (0.0 = left edge, 1.0 = right edge), or None if not detected
        """
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create mask for pink
        pink_mask = cv2.inRange(hsv, self.pink_lower, self.pink_upper)

        # Divide frame into vertical strips and count pink pixels in each
        num_strips = 20
        strip_width = width // num_strips
        pink_counts = []

        for i in range(num_strips):
            x_start = i * strip_width
            x_end = (i + 1) * strip_width if i < num_strips - 1 else width
            strip = pink_mask[:, x_start:x_end]
            pink_count = np.sum(strip)
            pink_counts.append(pink_count)

        # Find where pink count drops significantly (edge location)
        max_count = max(pink_counts) if pink_counts else 0
        if max_count < 100:  # Not enough pink detected
            return None

        # Find the rightmost strip with significant pink (edge is just after this)
        threshold = max_count * 0.3  # 30% of max
        edge_strip = 0
        for i in range(num_strips):
            if pink_counts[i] > threshold:
                edge_strip = i

        # Convert strip index to normalized position
        edge_position = (edge_strip + 0.5) / num_strips
        return edge_position

    def _calculate_left_right_pink(self, frame: np.ndarray) -> tuple:
        """
        Calculate pink pixel count for left and right halves of the frame.

        Returns:
            tuple: (left_pink_count, right_pink_count) or (0, 0) if detection fails
        """
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create pink mask
        pink_mask = cv2.inRange(hsv, self.pink_lower, self.pink_upper)

        # Split frame into left and right halves
        mid = width // 2
        left_pink = np.sum(pink_mask[:, :mid])
        right_pink = np.sum(pink_mask[:, mid:])

        return (left_pink, right_pink)

    def _detect_apriltags(self, frame: np.ndarray) -> list:
        """
        Detect AprilTags in the current camera frame.

        Returns:
            list: Detected AprilTag objects, or empty list if none found
        """
        if self.apriltag_detector is None:
            return []

        # Convert to grayscale for AprilTag detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect tags
        detections = self.apriltag_detector.detect(gray)

        return detections

    def _determine_free_corner(self, apriltag_detections: list) -> Optional[float]:
        """
        Determine which corner is free based on AprilTag detections.

        Case I: If no AprilTag detected, the corner in view is free
        Case II: If AprilTag detected (other payload visible), navigate to opposite side

        Args:
            apriltag_detections: List of detected AprilTag objects

        Returns:
            float: Target angle in radians to face free corner, or None if undetermined
        """
        if len(apriltag_detections) == 0:
            # Case I: No other payload visible - current view is free
            self.log("No other payload detected - corner in current view is free")
            return 0.0  # Continue in current direction

        # Case II: Other payload detected - navigate to opposite side
        # Get position of detected payload from AprilTag
        tag = apriltag_detections[0]  # Use first detection
        tag_center_x = tag.center[0]

        # Determine if tag is on left or right side of frame
        # If tag on right, turn left (negative angle)
        # If tag on left, turn right (positive angle)
        if self.current_image is None:
            return None

        frame = self._convert_image_msg_to_frame(self.current_image)
        frame_center_x = frame.shape[1] / 2

        if tag_center_x > frame_center_x:
            # Tag on right, turn left (opposite direction)
            target_angle = np.pi  # 180 degrees
            self.log(
                f"Payload detected on right (tag_id={tag.tag_id}) - navigating to opposite side"
            )
        else:
            # Tag on left, turn right (opposite direction)
            target_angle = np.pi  # 180 degrees
            self.log(
                f"Payload detected on left (tag_id={tag.tag_id}) - navigating to opposite side"
            )

        return target_angle

    def _publish_drive_command(self, linear: float, angular: float) -> None:
        """Publish a drive command to the payload."""
        if self.drive_publisher is None:
            return

        msg = DriveCommand()
        msg.linear = linear
        msg.angular = angular
        self.drive_publisher.publish(msg)

    def _stop_payload(self) -> None:
        """Stop the payload by sending zero velocity."""
        self._publish_drive_command(0.0, 0.0)

    def _start_turn(self, target_angle: float) -> None:
        """
        Start a turn with coast compensation awareness.

        Args:
            target_angle: Angle in radians to turn (relative)
        """
        self.is_turning = True
        self.turn_target = target_angle
        self.turn_elapsed = 0.0
        self.turn_settling_elapsed = 0.0  # Reset settling timer
        self.turn_duration = abs(target_angle) / self.angular_velocity

        turn_dir = "RIGHT" if target_angle < 0 else "LEFT"
        coast = self.coast_angle_right if target_angle < 0 else self.coast_angle_left
        self.log(
            f"Starting {turn_dir} turn: {np.degrees(target_angle):.1f}° "
            f"(coast={np.degrees(coast):.1f}°, duration={self.turn_duration:.2f}s)"
        )

    def _update_turn(self, time_delta: float) -> bool:
        """
        Update turning state with early stopping to compensate for coast momentum.

        Returns:
            bool: True if turn is complete, False otherwise
        """
        # Determine turn direction and coast compensation
        is_right_turn = self.turn_target < 0
        coast_angle = self.coast_angle_right if is_right_turn else self.coast_angle_left

        # Calculate how much we've turned
        angle_turned_so_far = (self.turn_elapsed / self.turn_duration) * abs(
            self.turn_target
        )
        angle_remaining = abs(self.turn_target) - angle_turned_so_far

        # Check if we should stop commanding (coast zone)
        if angle_remaining <= coast_angle:
            # Stop commanding velocity - let it coast
            self._stop_payload()

            # Wait for settling
            self.turn_settling_elapsed += time_delta

            if self.turn_settling_elapsed >= self.settling_time:
                # Mark turn complete
                self.is_turning = False
                self.current_heading += self.turn_target
                turn_dir = "RIGHT" if is_right_turn else "LEFT"
                self.log(
                    f"{turn_dir} turn complete after {self.settling_time:.2f}s settling"
                )
                return True
            return False

        # Reset settling timer while still turning
        self.turn_settling_elapsed = 0.0

        # Smooth deceleration parameters
        decel_start_angle = np.radians(20)  # Start decel at 20° remaining
        min_angular_vel = self.angular_velocity * 0.20  # Min 20% speed

        # Velocity control with smooth ramp
        if angle_remaining > decel_start_angle:
            angular_vel = self.angular_velocity
        else:
            # Proportional deceleration
            vel_range = self.angular_velocity - min_angular_vel
            angle_range = decel_start_angle - coast_angle
            progress = (angle_remaining - coast_angle) / angle_range
            angular_vel = min_angular_vel + (vel_range * progress)
            angular_vel = max(angular_vel, min_angular_vel)

        # Apply direction
        if is_right_turn:
            angular_vel = -angular_vel

        self._publish_drive_command(0.0, angular_vel)
        self.turn_elapsed += time_delta
        return False

    def _perform_directional_scan(self, time_delta: float) -> None:
        """
        Scan a span around current heading at configured intervals.
        Default: ±30° at 5° intervals = 13 positions: -30°, -25°, ..., 0°, ..., +25°, +30°
        """
        # Handle turning between scan positions
        if self.is_turning:
            if self._update_turn(time_delta):
                # Turn complete, measure ratio at this angle
                if self.current_image is None:
                    self.log("Waiting for camera image during scan...")
                    return

                frame = self._convert_image_msg_to_frame(self.current_image)
                pink_ratio = self._calculate_color_ratio(frame)

                if pink_ratio is not None:
                    # Record relative angle from scan start
                    relative_angle = self._get_scan_angle(self.scan_index)
                    self.scan_ratios.append((relative_angle, pink_ratio))
                    angle_deg = np.degrees(relative_angle)
                    self.log(
                        f"Scan {self.scan_index + 1}/{self.scan_points}: angle={angle_deg:+.1f}°, pink_ratio={pink_ratio:.3f}"
                    )
                    self.scan_index += 1
                else:
                    self.log("Warning: Invalid color detection during scan")

                # Check if scan complete
                if self.scan_index >= self.scan_points:
                    self._complete_scan()
            return

        # Start next scan position
        if self.scan_index < self.scan_points:
            # Get target angle for this scan point
            target_angle = self._get_scan_angle(self.scan_index)

            # If this is the first scan point (left), turn left from start
            if self.scan_index == 0:
                self._start_turn(-self.scan_span)  # Turn left
            else:
                # Calculate incremental turn from previous position
                prev_angle = self._get_scan_angle(self.scan_index - 1)
                angle_diff = target_angle - prev_angle
                self._start_turn(angle_diff)
        else:
            self._complete_scan()

    def _get_scan_angle(self, index: int) -> float:
        """
        Get the relative angle for a given scan index.

        Maps index to evenly-spaced angles from -scan_span to +scan_span.
        Example: span=30°, interval=5° → [-30°, -25°, -20°, ..., 0°, ..., +25°, +30°]

        Args:
            index: Scan point index (0 to scan_points-1)

        Returns:
            float: Relative angle in radians
        """
        if self.scan_points == 1:
            return 0.0

        # Map index to angle using the configured interval
        # Index 0 = -scan_span, Index (scan_points-1) = +scan_span
        return -self.scan_span + (index * self.scan_interval)

    def _complete_scan(self) -> None:
        """Process scan results and select best direction."""
        if len(self.scan_ratios) == 0:
            self.log("Error: No valid scan data collected")
            self.error_state = True
            return

        # Find direction with minimum pink ratio (most green/grass)
        best_relative_angle, best_ratio = min(self.scan_ratios, key=lambda x: x[1])

        # Calculate absolute angle in heading space
        # We're currently at the rightmost scan position (+scan_span)
        # Need to return to center (0) then turn to best angle
        current_scan_offset = self.scan_span  # We ended at right side
        turn_needed = best_relative_angle - current_scan_offset

        self.log(
            f"Scan complete! Best direction: {np.degrees(best_relative_angle):+.1f}° (pink_ratio={best_ratio:.3f})"
        )

        # Turn to face best direction (if not already facing it)
        if abs(turn_needed) > 0.01:  # Small threshold to avoid tiny turns
            self._start_turn(turn_needed)

        self.phase = CornerNavigationPhase.NAVIGATE_TO_EDGE
        self.forward_elapsed = 0.0
        self.rescan_elapsed = 0.0

    def _center_on_edge(self, time_delta: float) -> None:
        """
        Center the payload on the edge by scanning 3 angles to find perpendicular orientation.
        Scans at 0°, +30°, -30° and chooses angle with best left/right balance.
        """
        if self.centering_scan_index == 0:
            # Initial measurement at current angle (0°)
            self.log("=" * 50)
            self.log("EDGE REACHED - Scanning for perpendicular orientation")
            self.log("=" * 50)

            frame = self._convert_image_msg_to_frame(self.current_image)
            left_pink, right_pink = self._calculate_left_right_pink(frame)

            # Calculate balance difference
            total_pink = left_pink + right_pink
            if total_pink > 0:
                left_ratio = float(left_pink) / float(total_pink)
                right_ratio = float(right_pink) / float(total_pink)
                balance_diff = abs(left_ratio - right_ratio)
            else:
                balance_diff = 0.0

            # Store measurement for current angle (0°)
            self.centering_measurements.append((0.0, balance_diff))
            self.log(f"Center scan 1/3 (0°): balance_diff={balance_diff:.3f}")

            # Turn left 30° for next measurement
            self.log("Center scan: turning left 30°")
            self._start_turn(np.radians(30))
            self.centering_scan_index = 1
            return

        elif self.centering_scan_index == 1:
            # Turning to +30°
            if self.is_turning:
                if self._update_turn(time_delta):
                    # Measure balance at +30°
                    frame = self._convert_image_msg_to_frame(self.current_image)
                    left_pink, right_pink = self._calculate_left_right_pink(frame)

                    total_pink = left_pink + right_pink
                    if total_pink > 0:
                        left_ratio = float(left_pink) / float(total_pink)
                        right_ratio = float(right_pink) / float(total_pink)
                        balance_diff = abs(left_ratio - right_ratio)
                    else:
                        balance_diff = 0.0

                    # Store measurement for +30°
                    self.centering_measurements.append((np.radians(30), balance_diff))
                    self.log(f"Center scan 2/3 (+30°): balance_diff={balance_diff:.3f}")

                    # Turn right 60° to get to -30°
                    self.log("Center scan: turning right 60°")
                    self._start_turn(np.radians(-60))
                    self.centering_scan_index = 2
            return

        elif self.centering_scan_index == 2:
            # Turning to -30°
            if self.is_turning:
                if self._update_turn(time_delta):
                    # Measure balance at -30°
                    frame = self._convert_image_msg_to_frame(self.current_image)
                    left_pink, right_pink = self._calculate_left_right_pink(frame)

                    total_pink = left_pink + right_pink
                    if total_pink > 0:
                        left_ratio = float(left_pink) / float(total_pink)
                        right_ratio = float(right_pink) / float(total_pink)
                        balance_diff = abs(left_ratio - right_ratio)
                    else:
                        balance_diff = 0.0

                    # Store measurement for -30°
                    self.centering_measurements.append((np.radians(-30), balance_diff))
                    self.log(f"Center scan 3/3 (-30°): balance_diff={balance_diff:.3f}")

                    # Find angle with best balance (minimum balance_diff)
                    best_angle, best_balance = min(
                        self.centering_measurements, key=lambda x: x[1]
                    )
                    self.log(
                        f"Best perpendicular orientation: {np.degrees(best_angle):+.0f}° (balance={best_balance:.3f})"
                    )

                    # Calculate turn needed to reach best angle from current position (-30°)
                    current_offset = np.radians(-30)
                    turn_needed = best_angle - current_offset

                    if abs(turn_needed) > 0.01:
                        self.log(
                            f"Turning to best angle: {np.degrees(turn_needed):+.1f}°"
                        )
                        self._start_turn(turn_needed)
                        self.centering_scan_index = 3
                    else:
                        # Already at best angle
                        self.log("Already at best angle")
                        self.log("=" * 50)
                        self.log("CENTERING COMPLETE - Starting corner scan")
                        self.log("=" * 50)
                        self.phase = CornerNavigationPhase.SCAN_FOR_CORNERS
                        self.scan_index = 0
                        self.scan_ratios = []
            return

        elif self.centering_scan_index == 3:
            # Turning to best angle
            if self.is_turning:
                if self._update_turn(time_delta):
                    self.log("Turn to best angle complete")
                    self.log("=" * 50)
                    self.log("CENTERING COMPLETE - Starting corner scan")
                    self.log("=" * 50)
                    self.phase = CornerNavigationPhase.SCAN_FOR_CORNERS
                    self.scan_index = 0
                    self.scan_ratios = []
            return

    def _scan_for_corners(self, time_delta: float) -> None:
        """
        Scan left (90°) and right (90°) to find which corner is closer.
        Chooses the direction with lower pink ratio (closer corner).
        """
        if self.scan_index == 0:
            # Start scan: turn left 90°
            self.log("Scanning for corners: turning left 90°")
            self._start_turn(np.radians(90))
            self.scan_index = 1
            return

        if self.is_turning:
            if self._update_turn(time_delta):
                # Turn complete, measure pink ratio
                frame = self._convert_image_msg_to_frame(self.current_image)
                pink_ratio = self._calculate_color_ratio(frame)

                if pink_ratio is not None:
                    if self.scan_index == 1:
                        # Just finished left turn, record left corner distance
                        # After turning left 90°, current_heading = π/2
                        self.scan_ratios.append((self.current_heading, pink_ratio))
                        self.log(
                            f"Left corner (heading={np.degrees(self.current_heading):.0f}°): pink_ratio={pink_ratio:.3f}"
                        )
                        # Now turn right 90° (first of two 90° turns to check right corner)
                        self.log("Scanning for corners: turning right 90° (1/2)")
                        self._start_turn(np.radians(-90))
                        self.scan_index = 2
                    elif self.scan_index == 2:
                        # Just finished first 90° right turn, now turn another 90° right
                        # After first 90° right turn, current_heading = 0
                        self.log("Scanning for corners: turning right 90° (2/2)")
                        self._start_turn(np.radians(-90))
                        self.scan_index = 3
                    elif self.scan_index == 3:
                        # Just finished second 90° right turn, record right corner distance
                        # After turning right 180° total, current_heading = -π/2
                        self.scan_ratios.append((self.current_heading, pink_ratio))
                        self.log(
                            f"Right corner (heading={np.degrees(self.current_heading):.0f}°): pink_ratio={pink_ratio:.3f}"
                        )

                        # Choose closest corner (min pink ratio)
                        best_angle, best_ratio = min(
                            self.scan_ratios, key=lambda x: x[1]
                        )
                        self.log(
                            f"Selected corner at {np.degrees(best_angle):+.0f}° (pink={best_ratio:.3f})"
                        )

                        self.corner_direction = best_angle  # Store for final 135° turn

                        # Turn to face corner direction
                        # Use actual current_heading instead of hardcoded value
                        turn_needed = best_angle - self.current_heading
                        self.log("=" * 50)
                        self.log(
                            f"CORNER SCAN COMPLETE - Moving to corner at {np.degrees(best_angle):+.0f}°"
                        )
                        self.log("=" * 50)

                        # Split large turns (>90°) into two 90° turns
                        if abs(turn_needed) > np.pi / 2:
                            # First turn: 90° in the direction of the turn
                            first_turn = (
                                np.radians(90) if turn_needed > 0 else np.radians(-90)
                            )
                            self.log("Splitting large turn: First 90° turn (1/2)")
                            self._start_turn(first_turn)
                            self.scan_index = 4  # Go to intermediate state
                        elif abs(turn_needed) > 0.01:
                            self.log(
                                f"Turning to corner direction: {np.degrees(turn_needed):+.1f}°"
                            )
                            self._start_turn(turn_needed)
                            self.phase = CornerNavigationPhase.NAVIGATE_TO_CORNER
                            self.scan_index = 0
                            self.scan_ratios = []
                            self.corner_movement_elapsed = 0.0
                        else:
                            self.log("Already facing corner direction")
                            self.phase = CornerNavigationPhase.NAVIGATE_TO_CORNER
                            self.scan_index = 0
                            self.scan_ratios = []
                            self.corner_movement_elapsed = 0.0
                    elif self.scan_index == 4:
                        # Intermediate state: first 90° turn complete, do second 90° turn
                        if self.is_turning:
                            if self._update_turn(time_delta):
                                # First turn complete, do second turn (compensated)
                                turn_needed = (
                                    self.corner_direction - self.current_heading
                                )
                                self.log("Splitting large turn: Second turn (2/2)")
                                self._start_turn(turn_needed)
                                self.scan_index = 5
                        return
                    elif self.scan_index == 5:
                        # Wait for second turn to complete
                        if self.is_turning:
                            if self._update_turn(time_delta):
                                # Both turns complete, proceed to navigate to corner
                                self.phase = CornerNavigationPhase.NAVIGATE_TO_CORNER
                                self.scan_index = 0
                                self.scan_ratios = []
                                self.corner_movement_elapsed = 0.0
                        return
            return

    def _navigate_to_corner(self, time_delta: float) -> None:
        """
        Move along the edge toward the corner.
        Stops when pink ratio drops very low (corner reached).
        """
        # Handle turn to corner direction
        if self.is_turning:
            turn_complete = self._update_turn(time_delta)
            if turn_complete:
                self.log("Turn to corner direction complete - ready to move along edge")
            return

        # Move forward along edge
        frame = self._convert_image_msg_to_frame(self.current_image)
        pink_ratio = self._calculate_color_ratio(frame)

        if pink_ratio is None:
            self.log("WARNING: No valid color detection in NAVIGATE_TO_CORNER")
            self.frames_without_valid_detection += 1
            if self.frames_without_valid_detection >= self.max_frames_without_detection:
                self.log("Error: Too many frames without valid color detection")
                self._stop_payload()
                self.error_state = True
            return

        self.frames_without_valid_detection = 0

        # Update movement timer
        self.corner_movement_elapsed += time_delta

        # Check if corner reached
        # Use a higher threshold than edge detection to stop before getting too close to corner
        # This prevents the back of payload from going off DLZ when turning
        corner_final_threshold = (
            self.corner_threshold * 0.92
        )  # Slightly lower than edge threshold (0.25 * 0.92 = 0.23)

        if pink_ratio <= corner_final_threshold:
            self.log(
                f"Corner reached! Pink ratio: {pink_ratio:.3f} <= {corner_final_threshold:.3f} after {self.corner_movement_elapsed:.2f}s"
            )
            self._stop_payload()
            self.phase = CornerNavigationPhase.TURN_TO_CENTER
            return

        # Edge correction disabled for now
        # edge_position = self._detect_edge_position(frame)
        # angular_velocity = 0.0
        # if edge_position is not None:
        #     target_position = 0.33
        #     position_error = edge_position - target_position
        #     if abs(position_error) > 0.1:
        #         correction_gain = 2.0
        #         angular_velocity = -position_error * correction_gain
        #         max_correction = self.angular_velocity * 0.4
        #         angular_velocity = np.clip(angular_velocity, -max_correction, max_correction)
        #         self.log(f"NAVIGATE_TO_CORNER: pink={pink_ratio:.3f}, edge_pos={edge_position:.2f}, target={target_position:.2f}, ang_vel={angular_velocity:+.3f}")
        #     else:
        #         self.log(f"NAVIGATE_TO_CORNER: pink={pink_ratio:.3f}, edge_pos={edge_position:.2f} (on track)")
        # else:
        #     self.log(f"NAVIGATE_TO_CORNER: pink={pink_ratio:.3f}, edge not detected")

        self.log(
            f"NAVIGATE_TO_CORNER: pink={pink_ratio:.3f}, time={self.corner_movement_elapsed:.2f}s"
        )

        # Continue moving toward corner (no correction)
        self._publish_drive_command(self.linear_velocity, 0.0)

    def on_update(self, time_delta: float) -> None:
        """Main update loop for corner navigation."""
        # Wait for camera data
        if self.current_image is None:
            self.log("Waiting for camera image...")
            return

        # State machine
        if self.phase == CornerNavigationPhase.INITIAL_SCAN:
            # scan_index: 0=detect obstacle ahead, 1=scan left, 2=measure left, 3=scan right, 4=measure right, 5=decide direction

            frame = self._convert_image_msg_to_frame(self.current_image)

            if self.scan_index == 0:
                # Check for obstacle ahead
                apriltag_detections = self._detect_apriltags(frame)
                has_obstacle = (
                    len(apriltag_detections) > 0 or self.force_obstacle_detected
                )

                if has_obstacle:
                    self.log(
                        f"Obstacle detected ahead! {'(FORCED for testing)' if self.force_obstacle_detected else f'AprilTag {apriltag_detections[0].tag_id}'}"
                    )
                    self.obstacle_detected = True
                    self.scan_index = 1  # Continue to direction scan
                else:
                    # No obstacle, proceed to edge navigation
                    self.log("No obstacle detected, proceeding to edge navigation")
                    self.phase = CornerNavigationPhase.NAVIGATE_TO_EDGE
                return

            # If obstacle detected, scan left and right to determine orbit direction
            if self.obstacle_detected:
                if self.scan_index == 1:
                    self.log("Scanning left to check pink ratio...")
                    self._start_turn(np.radians(90))
                    self.scan_index = 2
                    return
                elif self.scan_index == 2:
                    if self.is_turning:
                        if self._update_turn(time_delta):
                            # Measure pink on left side
                            pink_ratio = self._calculate_color_ratio(frame)
                            if pink_ratio is not None:
                                self.scan_ratios = [
                                    (self.current_heading, pink_ratio)
                                ]  # Store left measurement
                                self.log(
                                    f"Left side pink ratio: {pink_ratio:.3f} at heading={np.degrees(self.current_heading):.1f}°"
                                )
                                self.log("Scanning right: turning right 90° (1/2)...")
                                self._start_turn(np.radians(-90))
                                self.scan_index = 3
                        return
                elif self.scan_index == 3:
                    if self.is_turning:
                        if self._update_turn(time_delta):
                            # First 90° right turn complete, turn another 90° right
                            self.log("Scanning right: turning right 90° (2/2)...")
                            self._start_turn(np.radians(-90))
                            self.scan_index = 4
                        return
                elif self.scan_index == 4:
                    if self.is_turning:
                        if self._update_turn(time_delta):
                            # Measure pink on right side
                            pink_ratio = self._calculate_color_ratio(frame)
                            if pink_ratio is not None:
                                self.scan_ratios.append(
                                    (self.current_heading, pink_ratio)
                                )  # Store right measurement
                                self.log(
                                    f"Right side pink ratio: {pink_ratio:.3f} at heading={np.degrees(self.current_heading):.1f}°"
                                )
                                # Decide direction: go towards side with MORE pink (towards DLZ)
                                left_pink = self.scan_ratios[0][1]
                                right_pink = self.scan_ratios[1][1]
                                if left_pink > right_pink:
                                    self.orbit_direction = 1  # Go left (CCW)
                                    self.log(
                                        f"Choosing LEFT orbit (left_pink={left_pink:.3f} > right_pink={right_pink:.3f})"
                                    )
                                else:
                                    self.orbit_direction = -1  # Go right (CW)
                                    self.log(
                                        f"Choosing RIGHT orbit (right_pink={right_pink:.3f} >= left_pink={left_pink:.3f})"
                                    )
                                # Turn back to face forward
                                self.log("Turning back to face forward...")
                                self._start_turn(np.radians(90))
                                self.scan_index = 5
                        return
                elif self.scan_index == 5:
                    # Turn should already be complete from scan_index 4
                    # Wait for turn to finish if still turning
                    if self.is_turning:
                        if self._update_turn(time_delta):
                            # Turn just finished, transition to orbit next frame
                            pass
                        return
                    else:
                        # Turn complete, ready to start orbiting
                        self.log(
                            f"Starting orbit around obstacle (direction={'LEFT' if self.orbit_direction > 0 else 'RIGHT'})"
                        )
                        self.phase = CornerNavigationPhase.ORBIT_OBSTACLE
                        self.orbit_step = 0
                        self.forward_elapsed = 0.0
                        return

        elif self.phase == CornerNavigationPhase.ORBIT_OBSTACLE:
            # Execute hardcoded orbit path around aircraft
            # Aircraft dimensions: 0.343m wide, 1.080m long
            # Payload starts 0.2m behind aircraft
            # Orbit path: rectangular around the aircraft

            clearance = 0.25  # 25cm clearance around aircraft for safety
            initial_offset = 0.3  # Initial distance behind aircraft

            if self.orbit_step == 0:
                # Turn 90° sideways in chosen orbit direction
                base_angle = 90
                turn_angle = self.orbit_direction * np.radians(base_angle)
                self.log(
                    f"Orbit step 0: Turning {np.degrees(turn_angle):.0f}° sideways"
                )
                self._start_turn(turn_angle)
                self.orbit_step = 1
                return

            elif self.orbit_step == 1:
                # Wait for turn to complete
                if self.is_turning:
                    if self._update_turn(time_delta):
                        # Turn complete, prepare to move sideways
                        self.forward_elapsed = 0.0
                        # Move past half aircraft length (from centerline to edge) + clearance
                        distance = self.aircraft_length / 2 + clearance
                        self.orbit_move_duration = distance / self.linear_velocity
                        self.log(
                            f"Orbit step 1: Moving {distance:.2f}m sideways past aircraft"
                        )
                        self.orbit_step = 2  # Advance to movement step
                return

            elif self.orbit_step == 2:
                # Move sideways past aircraft width
                self._publish_drive_command(self.linear_velocity, 0.0)
                self.forward_elapsed += time_delta

                # Debug logging every ~1 second
                if int(self.forward_elapsed * 10) % 10 == 0:
                    self.log(
                        f"Orbit step 2: Moving sideways {self.forward_elapsed:.2f}s / {self.orbit_move_duration:.2f}s"
                    )

                if self.forward_elapsed >= self.orbit_move_duration:
                    self._stop_payload()
                    self.log("Sideways movement complete")
                    self.orbit_step = 3
                return

            elif self.orbit_step == 3:
                # Second turn in same direction (90° to complete 180° total)
                base_angle = 90
                turn_angle = self.orbit_direction * np.radians(base_angle)
                self.log(
                    f"Orbit step 3: Turning {np.degrees(turn_angle):.0f}° (second turn)"
                )
                self._start_turn(turn_angle)
                self.orbit_step = 4
                return

            elif self.orbit_step == 4:
                # Wait for turn to complete
                if self.is_turning:
                    if self._update_turn(time_delta):
                        # Turn complete, prepare to move forward
                        self.forward_elapsed = 0.0
                        # Move past: initial offset + aircraft width + clearance
                        distance = initial_offset + self.aircraft_width + clearance
                        self.orbit_move_duration = distance / self.linear_velocity
                        self.log(
                            f"Orbit step 4: Moving {distance:.2f}m forward past aircraft"
                        )
                        self.orbit_step = 5  # Advance to movement step
                return

            elif self.orbit_step == 5:
                # Move forward past aircraft
                self._publish_drive_command(self.linear_velocity, 0.0)
                self.forward_elapsed += time_delta
                if self.forward_elapsed >= self.orbit_move_duration:
                    self._stop_payload()
                    self.log("Forward movement complete")
                    self.orbit_step = 6
                return

            elif self.orbit_step == 6:
                # Third turn in same direction
                base_angle = 90
                turn_angle = self.orbit_direction * np.radians(base_angle)
                self.log(
                    f"Orbit step 6: Turning {np.degrees(turn_angle):.0f}° (third turn)"
                )
                self._start_turn(turn_angle)
                self.orbit_step = 7
                return

            elif self.orbit_step == 7:
                # Wait for turn to complete
                if self.is_turning:
                    if self._update_turn(time_delta):
                        # Turn complete, prepare to move back to centerline
                        self.forward_elapsed = 0.0
                        # Move back the same distance we moved sideways (half length + clearance)
                        distance = self.aircraft_length / 2 + clearance
                        self.orbit_move_duration = distance / self.linear_velocity
                        self.log(
                            f"Orbit step 7: Moving {distance:.2f}m back to centerline"
                        )
                        self.orbit_step = 8  # Advance to movement step
                return

            elif self.orbit_step == 8:
                # Move back to centerline
                self._publish_drive_command(self.linear_velocity, 0.0)
                self.forward_elapsed += time_delta
                if self.forward_elapsed >= self.orbit_move_duration:
                    self._stop_payload()
                    self.log("Return to centerline complete")
                    self.orbit_step = 9
                return

            elif self.orbit_step == 9:
                # Final turn in OPPOSITE direction to face 180° from original
                base_angle = 90
                turn_angle = -self.orbit_direction * np.radians(base_angle)
                self.log(
                    f"Orbit step 9: Turning {np.degrees(turn_angle):.0f}° to face opposite direction"
                )
                self._start_turn(turn_angle)
                self.orbit_step = 10
                return

            elif self.orbit_step == 10:
                # Wait for final turn to complete
                if self.is_turning:
                    if self._update_turn(time_delta):
                        # Orbit complete
                        self.log("=" * 50)
                        self.log("ORBIT COMPLETE - Proceeding to edge navigation")
                        self.log("=" * 50)
                        self.phase = CornerNavigationPhase.NAVIGATE_TO_EDGE
                        self.scan_ratios = []
                        self.scan_index = 0
                        self.forward_elapsed = 0.0
                return

        elif self.phase == CornerNavigationPhase.NAVIGATE_TO_EDGE:
            # Move forward until edge is reached, with course correction based on left/right ratios
            frame = self._convert_image_msg_to_frame(self.current_image)
            pink_ratio = self._calculate_color_ratio(frame)

            if pink_ratio is None:
                self.frames_without_valid_detection += 1
                if (
                    self.frames_without_valid_detection
                    >= self.max_frames_without_detection
                ):
                    self.log("Error: Too many frames without valid color detection")
                    self._stop_payload()
                    self.error_state = True
                return

            self.frames_without_valid_detection = 0

            # Check if edge reached (low pink ratio - mostly grass)
            if pink_ratio <= self.corner_threshold:
                self.log(
                    f"Edge reached! Pink ratio: {pink_ratio:.3f} <= threshold {self.corner_threshold}"
                )
                self._stop_payload()
                self.phase = CornerNavigationPhase.CENTER_ON_EDGE
                self.centering_scan_index = 0  # Reset centering state
                self.centering_measurements = []  # Reset measurements
                return

            # Move straight forward to edge (no course correction)
            self.forward_elapsed += time_delta
            self.log(
                f"Moving to edge. Pink ratio: {pink_ratio:.3f}, forward: {self.forward_elapsed:.1f}s"
            )
            self._publish_drive_command(self.linear_velocity, 0.0)

        elif self.phase == CornerNavigationPhase.CENTER_ON_EDGE:
            self._center_on_edge(time_delta)

        elif self.phase == CornerNavigationPhase.SCAN_FOR_CORNERS:
            self._scan_for_corners(time_delta)

        elif self.phase == CornerNavigationPhase.NAVIGATE_TO_CORNER:
            self._navigate_to_corner(time_delta)

        elif self.phase == CornerNavigationPhase.TURN_TO_CENTER:
            # Turn 135° to face back toward center
            # Direction depends on which corner we went to
            if not self.is_turning:
                # If we went to left corner (heading = π/2), turn right (+135°)
                # If we went to right corner (heading = -π/2), turn left (-135°)
                # Since corner_direction now stores actual heading, use it directly
                turn_direction = (
                    np.sign(self.corner_direction) if self.corner_direction != 0 else 1
                )
                turn_angle = turn_direction * np.radians(135)
                self.log(
                    f"Turning {np.degrees(turn_angle):+.1f}° to face center (from heading {np.degrees(self.corner_direction):.0f}°)..."
                )
                self._start_turn(turn_angle)
            else:
                if self._update_turn(time_delta):
                    self.log("Turn to center complete!")
                    self.phase = CornerNavigationPhase.CORNER_REACHED

        elif self.phase == CornerNavigationPhase.CORNER_REACHED:
            # Stay stopped
            self._stop_payload()

    def check_status(self) -> str:
        """
        Check the status of corner navigation.

        Returns:
            'complete' when corner is reached
            'error' on failure
            'continue' otherwise
        """
        if self.error_state:
            return "error"
        if self.phase == CornerNavigationPhase.CORNER_REACHED:
            return "complete"
        return "continue"

    def on_exit(self) -> None:
        """Cleanup when mode is deactivated."""
        self._stop_payload()

        # Destroy subscriber
        if self.camera_subscriber is not None:
            self.node.destroy_subscription(self.camera_subscriber)
            self.camera_subscriber = None

        self.log("PayloadCornerNavigationMode exited, payload stopped")
