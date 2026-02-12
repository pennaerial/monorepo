import numpy as np
import cv2
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from sensor_msgs.msg import Image
from payload_interfaces.msg import DriveCommand
from uav.utils import pink, green
from typing import Optional


class PayloadEdgeNavigationMode(Mode):
    """
    A mode for the payload to navigate from the center of the DLZ to its edge.
    Uses color ratio detection (pink DLZ vs green grass) to detect the edge.
    """

    def __init__(
        self,
        node: Node,
        uav: UAV,
        payload_name: str = 'payload_0',
        edge_threshold: float = 0.15,
        linear_velocity: float = 0.3,
        angular_velocity: float = 0.5,
        turn_angle: float = np.pi,
        min_color_pixels: int = 100
    ):
        """
        Initialize the PayloadEdgeNavigationMode.

        Args:
            node (Node): ROS 2 node managing the mode.
            uav (UAV): The UAV instance (not directly used for payload control).
            payload_name (str): Name of the payload for topic namespacing.
            edge_threshold (float): Pink ratio below which edge is considered detected.
            linear_velocity (float): Forward velocity for the payload in m/s.
            angular_velocity (float): Turn velocity in rad/s.
            turn_angle (float): Angle to turn in radians (default: π = 180 degrees).
            min_color_pixels (int): Minimum combined pink+green pixels for valid detection.
        """
        super().__init__(node, uav)

        self.payload_name = payload_name
        self.edge_threshold = float(edge_threshold)
        self.linear_velocity = float(linear_velocity)
        self.angular_velocity = float(angular_velocity)
        self.turn_angle = float(turn_angle)
        self.min_color_pixels = int(min_color_pixels)

        # HSV color ranges from utils.py
        self.pink_lower = np.array(pink[0])
        self.pink_upper = np.array(pink[1])
        # Use wider green range for Gazebo ground (original is too narrow)
        # Original: H:30-40, S:110-255, V:20-255
        # Wider: H:25-90, S:20-255, V:20-255 (captures more green shades)
        self.green_lower = np.array([25, 20, 20])
        self.green_upper = np.array([90, 255, 255])

        # State tracking
        self.current_image: Optional[Image] = None
        self.edge_detected = False
        self.turning = False
        self.turn_complete = False
        self.turn_elapsed = 0.0
        # Time to turn: angle (radians) / angular_velocity (rad/s)
        self.turn_duration = self.turn_angle / self.angular_velocity
        self.error_state = False
        self.frames_without_valid_detection = 0
        self.max_frames_without_detection = 50  # ~5 seconds at 10Hz

        # Publishers and subscribers (initialized in on_enter)
        self.drive_publisher = None
        self.camera_subscriber = None

    def on_enter(self) -> None:
        """
        Called when the mode is activated.
        Sets up camera subscription and drive command publisher.
        """
        # Create publisher for payload drive commands
        drive_topic = f'/{self.payload_name}/cmd_drive'
        self.drive_publisher = self.node.create_publisher(
            DriveCommand,
            drive_topic,
            10
        )
        self.log(f"Created drive publisher on {drive_topic}")

        # Subscribe to payload camera
        camera_topic = f'/{self.payload_name}/camera'
        self.camera_subscriber = self.node.create_subscription(
            Image,
            camera_topic,
            self._camera_callback,
            10
        )
        self.log(f"Subscribed to camera on {camera_topic}")

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
            float: Ratio of pink pixels, or None if insufficient color detected.
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

    def on_update(self, time_delta: float) -> None:
        """
        Periodic update logic. Analyzes camera feed and controls payload movement.
        """
        # Handle turning phase (after edge detected)
        if self.turning:
            self.turn_elapsed += time_delta
            if self.turn_elapsed >= self.turn_duration:
                self.log(f"{np.degrees(self.turn_angle):.0f} degree turn complete!")
                self._stop_payload()
                self.turning = False
                self.turn_complete = True
            else:
                remaining = self.turn_duration - self.turn_elapsed
                self.log(f"Turning... {remaining:.1f}s remaining")
                self._publish_drive_command(0.0, self.angular_velocity)
            return

        # Wait for camera data
        if self.current_image is None:
            self.log("Waiting for camera image...")
            return

        # Convert image
        frame = self._convert_image_msg_to_frame(self.current_image)

        # Calculate color ratio
        pink_ratio = self._calculate_color_ratio(frame)

        if pink_ratio is None:
            # Not enough colored pixels detected - check raw pixel counts
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            pink_mask = cv2.inRange(hsv, self.pink_lower, self.pink_upper)
            green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
            pink_px = cv2.countNonZero(pink_mask)
            green_px = cv2.countNonZero(green_mask)

            self.frames_without_valid_detection += 1
            self.log(f"Insufficient color detection ({self.frames_without_valid_detection}/{self.max_frames_without_detection}) - pink: {pink_px}, green: {green_px}")

            if self.frames_without_valid_detection >= self.max_frames_without_detection:
                self.log("Error: Too many frames without valid color detection")
                self._stop_payload()
                self.error_state = True
            return

        # Reset detection counter on valid detection
        self.frames_without_valid_detection = 0

        # Check if we've reached the edge
        if pink_ratio < self.edge_threshold:
            self.log(f"Edge detected! Pink ratio: {pink_ratio:.3f} < threshold {self.edge_threshold}")
            turn_degrees = np.degrees(self.turn_angle)
            self.log(f"Starting {turn_degrees:.0f} degree turn ({self.turn_duration:.1f}s at {self.angular_velocity} rad/s)")
            self.edge_detected = True
            self.turning = True
            self.turn_elapsed = 0.0
            return

        # Continue driving forward
        self.log(f"Driving forward. Pink ratio: {pink_ratio:.3f}")
        self._publish_drive_command(self.linear_velocity, 0.0)

    def check_status(self) -> str:
        """
        Check the status of the edge navigation.

        Returns:
            str: 'complete' when edge is reached and turn is done, 'error' on failure, 'continue' otherwise.
        """
        if self.error_state:
            return 'error'
        if self.turn_complete:
            return 'complete'
        return 'continue'

    def on_exit(self) -> None:
        """
        Called when the mode is deactivated.
        Ensures payload is stopped and cleans up subscribers.
        """
        self._stop_payload()

        # Destroy subscriber (cleanup)
        if self.camera_subscriber is not None:
            self.node.destroy_subscription(self.camera_subscriber)
            self.camera_subscriber = None

        self.log("PayloadEdgeNavigationMode exited, payload stopped")
