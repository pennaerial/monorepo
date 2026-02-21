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


class CornerNavigationPhase(Enum):
    """State machine phases for corner navigation."""
    # APRILTAG_DETECTION = "apriltag_detection"  # (Future) Check for existing payload
    # CORNER_SELECTION = "corner_selection"      # (Future) Choose free corner
    INITIAL_SCAN = "initial_scan"              # Sample ratios in multiple directions
    NAVIGATE_TO_EDGE = "navigate_to_edge"      # Move toward nearest edge
    RESCAN_TO_EDGE = "rescan_to_edge"          # Periodically rescan while moving to edge
    CENTER_ON_EDGE = "center_on_edge"          # Align perpendicular to edge
    SCAN_FOR_CORNERS = "scan_for_corners"      # Turn left/right to find nearest corner
    NAVIGATE_TO_CORNER = "navigate_to_corner"  # Move along edge to corner
    TURN_TO_CENTER = "turn_to_center"          # Turn 135° to face center
    CORNER_REACHED = "corner_reached"          # At corner, done


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
        """
        super().__init__(node, uav)

        self.payload_name = payload_name
        self.corner_threshold = float(corner_threshold)
        self.linear_velocity = float(linear_velocity)
        self.angular_velocity = float(angular_velocity)
        self.scan_span = float(scan_span)
        self.scan_interval = float(scan_interval)

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
        self.corner_direction = 0.0  # Which direction we turned to go to corner (for final turn)

        # Navigation state
        self.forward_elapsed = 0.0
        self.rescan_elapsed = 0.0
        self.corner_movement_elapsed = 0.0  # Time spent moving to corner
        self.is_turning = False
        self.turn_target = 0.0
        self.turn_elapsed = 0.0
        self.turn_duration = 0.0

        # Centering state
        self.centering_scan_index = 0
        self.centering_measurements = []  # List of (angle, balance_diff) tuples

        # Detection tracking
        self.frames_without_valid_detection = 0
        self.max_frames_without_detection = 50  # ~5 seconds at 10Hz

        # Publishers and subscribers (initialized in on_enter)
        self.drive_publisher = None
        self.camera_subscriber = None

        # === APRILTAG DETECTION (COMMENTED OUT FOR NOW) ===
        # When enabled, this will detect AprilTags on existing payloads
        # to determine which corners are already occupied
        # self.apriltag_detector = None
        # self.occupied_corners = []  # List of corner positions with existing payloads
        # self.selected_corner = None  # Chosen target corner

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

        # Initialize state - start by moving straight forward to edge
        self.phase = CornerNavigationPhase.NAVIGATE_TO_EDGE
        self.scan_index = 0
        self.scan_ratios = []
        self.current_heading = 0.0

        # === APRILTAG DETECTION (COMMENTED OUT FOR NOW) ===
        # Initialize AprilTag detector
        # try:
        #     import apriltag
        #     self.apriltag_detector = apriltag.Detector()
        #     self.log("AprilTag detector initialized")
        # except ImportError:
        #     self.log("Warning: apriltag library not available, skipping AprilTag detection")
        #     self.apriltag_detector = None

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

    # === APRILTAG DETECTION METHODS (COMMENTED OUT FOR NOW) ===
    # def _detect_apriltags(self, frame: np.ndarray) -> list:
    #     """
    #     Detect AprilTags in the current camera frame.
    #
    #     Returns:
    #         list: Detected AprilTag objects, or empty list if none found
    #     """
    #     if self.apriltag_detector is None:
    #         return []
    #
    #     # Convert to grayscale for AprilTag detection
    #     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #
    #     # Detect tags
    #     detections = self.apriltag_detector.detect(gray)
    #
    #     return detections
    #
    # def _determine_free_corner(self, apriltag_detections: list) -> Optional[float]:
    #     """
    #     Determine which corner is free based on AprilTag detections.
    #
    #     Case I: If no AprilTag detected, the corner in view is free
    #     Case II: If AprilTag detected (other payload visible), navigate to opposite side
    #
    #     Args:
    #         apriltag_detections: List of detected AprilTag objects
    #
    #     Returns:
    #         float: Target angle in radians to face free corner, or None if undetermined
    #     """
    #     if len(apriltag_detections) == 0:
    #         # Case I: No other payload visible - current view is free
    #         self.log("No other payload detected - corner in current view is free")
    #         return 0.0  # Continue in current direction
    #
    #     # Case II: Other payload detected - navigate to opposite side
    #     # Get position of detected payload from AprilTag
    #     tag = apriltag_detections[0]  # Use first detection
    #     tag_center_x = tag.center[0]
    #
    #     # Determine if tag is on left or right side of frame
    #     # If tag on right, turn left (negative angle)
    #     # If tag on left, turn right (positive angle)
    #     frame_center_x = self.current_image.width / 2
    #
    #     if tag_center_x > frame_center_x:
    #         # Tag on right, turn left (opposite direction)
    #         target_angle = np.pi  # 180 degrees
    #         self.log("Payload detected on right - navigating to opposite side (left)")
    #     else:
    #         # Tag on left, turn right (opposite direction)
    #         target_angle = np.pi  # 180 degrees
    #         self.log("Payload detected on left - navigating to opposite side (right)")
    #
    #     return target_angle

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
        Start a turn to face a target angle.

        Args:
            target_angle: Angle in radians to turn (relative)
        """
        self.is_turning = True
        self.turn_target = target_angle
        self.turn_elapsed = 0.0
        self.turn_duration = abs(target_angle) / self.angular_velocity

        turn_deg = np.degrees(target_angle)
        self.log(f"Starting turn: {turn_deg:.1f} degrees ({self.turn_duration:.2f}s)")

    def _update_turn(self, time_delta: float) -> bool:
        """
        Update turning state with precise angle control using predictive lookahead.

        Returns:
            bool: True if turn is complete, False otherwise
        """
        # Calculate how much angle we've turned so far
        angle_turned_so_far = (self.turn_elapsed / self.turn_duration) * abs(self.turn_target)

        # Predict how much we'll have turned AFTER this timestep at full speed
        angle_if_full_speed = angle_turned_so_far + (self.angular_velocity * time_delta)
        angle_remaining_now = abs(self.turn_target) - angle_turned_so_far

        # Check if we're already done
        if angle_remaining_now <= 0.0001:  # Within 0.0001 radians (~0.006 degrees)
            self._stop_payload()
            self.is_turning = False
            self.current_heading += self.turn_target
            self.log(f"Turn complete (error: {np.degrees(angle_remaining_now):.4f}°)")
            return True

        # Determine angular velocity for this timestep
        # If full speed would overshoot, reduce velocity proportionally
        if angle_if_full_speed > abs(self.turn_target):
            # We're on the final approach - slow down to hit target exactly
            # Calculate exact velocity needed to reach target in this timestep
            if time_delta > 0.001:
                angular_vel = angle_remaining_now / time_delta
                # Apply direction
                if self.turn_target < 0:
                    angular_vel = -angular_vel
                self.log(f"Final approach: {np.degrees(angle_remaining_now):.3f}° remaining, vel={angular_vel:.4f} rad/s")
            else:
                # Safety: timestep too small, stop
                self._stop_payload()
                self.is_turning = False
                self.current_heading += self.turn_target
                return True
        else:
            # Normal turning - still far from target
            angular_vel = self.angular_velocity if self.turn_target > 0 else -self.angular_velocity

        # Send command and update elapsed time
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
                    self.log(f"Scan {self.scan_index + 1}/{self.scan_points}: angle={angle_deg:+.1f}°, pink_ratio={pink_ratio:.3f}")
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

        self.log(f"Scan complete! Best direction: {np.degrees(best_relative_angle):+.1f}° (pink_ratio={best_ratio:.3f})")

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
                    best_angle, best_balance = min(self.centering_measurements, key=lambda x: x[1])
                    self.log(f"Best perpendicular orientation: {np.degrees(best_angle):+.0f}° (balance={best_balance:.3f})")

                    # Calculate turn needed to reach best angle from current position (-30°)
                    current_offset = np.radians(-30)
                    turn_needed = best_angle - current_offset

                    if abs(turn_needed) > 0.01:
                        self.log(f"Turning to best angle: {np.degrees(turn_needed):+.1f}°")
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
            self._start_turn(np.pi / 2)  # Turn left
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
                        self.log(f"Left corner (heading={np.degrees(self.current_heading):.0f}°): pink_ratio={pink_ratio:.3f}")
                        # Now turn right 180° to check right corner
                        self.log("Scanning for corners: turning right 180°")
                        self._start_turn(-np.pi)  # Turn right (180° from left)
                        self.scan_index = 2
                    elif self.scan_index == 2:
                        # Just finished right turn, record right corner distance
                        # After turning right 180°, current_heading = -π/2
                        self.scan_ratios.append((self.current_heading, pink_ratio))
                        self.log(f"Right corner (heading={np.degrees(self.current_heading):.0f}°): pink_ratio={pink_ratio:.3f}")
                        # Choose closest corner (min pink = closer)
                        best_angle, best_ratio = min(self.scan_ratios, key=lambda x: x[1])
                        self.corner_direction = best_angle  # Store for final 135° turn
                        self.log(f"Nearest corner is at {np.degrees(best_angle):+.0f}° (pink={best_ratio:.3f})")
                        # Turn to face corner direction
                        # Use actual current_heading instead of hardcoded value
                        turn_needed = best_angle - self.current_heading
                        self.log("=" * 50)
                        self.log(f"CORNER SCAN COMPLETE - Moving to corner at {np.degrees(best_angle):+.0f}°")
                        self.log("=" * 50)
                        if abs(turn_needed) > 0.01:
                            self.log(f"Turning to corner direction: {np.degrees(turn_needed):+.1f}°")
                            self._start_turn(turn_needed)
                        else:
                            self.log("Already facing corner direction")
                        self.phase = CornerNavigationPhase.NAVIGATE_TO_CORNER
                        self.scan_index = 0
                        self.scan_ratios = []
                        self.corner_movement_elapsed = 0.0  # Reset corner movement timer
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
        corner_final_threshold = self.corner_threshold * 0.92  # Slightly lower than edge threshold (0.25 * 0.92 = 0.23)

        if pink_ratio <= corner_final_threshold:
            self.log(f"Corner reached! Pink ratio: {pink_ratio:.3f} <= {corner_final_threshold:.3f} after {self.corner_movement_elapsed:.2f}s")
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

        self.log(f"NAVIGATE_TO_CORNER: pink={pink_ratio:.3f}, time={self.corner_movement_elapsed:.2f}s")

        # Continue moving toward corner (no correction)
        self._publish_drive_command(self.linear_velocity, 0.0)

    def on_update(self, time_delta: float) -> None:
        """Main update loop for corner navigation."""
        # Wait for camera data
        if self.current_image is None:
            self.log("Waiting for camera image...")
            return

        # State machine
        if self.phase == CornerNavigationPhase.NAVIGATE_TO_EDGE:
            # Move forward until edge is reached, with course correction based on left/right ratios
            frame = self._convert_image_msg_to_frame(self.current_image)
            pink_ratio = self._calculate_color_ratio(frame)

            if pink_ratio is None:
                self.frames_without_valid_detection += 1
                if self.frames_without_valid_detection >= self.max_frames_without_detection:
                    self.log("Error: Too many frames without valid color detection")
                    self._stop_payload()
                    self.error_state = True
                return

            self.frames_without_valid_detection = 0

            # Check if edge reached (low pink ratio - mostly grass)
            if pink_ratio <= self.corner_threshold:
                self.log(f"Edge reached! Pink ratio: {pink_ratio:.3f} <= threshold {self.corner_threshold}")
                self._stop_payload()
                self.phase = CornerNavigationPhase.CENTER_ON_EDGE
                self.centering_scan_index = 0  # Reset centering state
                self.centering_measurements = []  # Reset measurements
                return

            # Move straight forward to edge (no course correction)
            self.forward_elapsed += time_delta
            self.log(f"Moving to edge. Pink ratio: {pink_ratio:.3f}, forward: {self.forward_elapsed:.1f}s")
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
                turn_direction = np.sign(self.corner_direction) if self.corner_direction != 0 else 1
                turn_angle = turn_direction * np.radians(135)
                self.log(f"Turning {np.degrees(turn_angle):+.1f}° to face center (from heading {np.degrees(self.corner_direction):.0f}°)...")
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
