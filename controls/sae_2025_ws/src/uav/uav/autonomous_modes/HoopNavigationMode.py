import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.msg import HoopTracking
from typing import Optional, Tuple
import cv2

class HoopNavigationMode(Mode):
    """
    A mode for navigating through a hoop using vision tracking.
    """

    def __init__(self, node: Node, uav: UAV, approach_distance: float = 5.0, 
                 pass_distance: float = 2.0, speed_factor: float = 1.5):
        """
        Initialize the HoopNavigationMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            approach_distance (float): Distance at which to start centering on hoop (meters).
            pass_distance (float): Distance to travel past the hoop center (meters).
            speed_factor (float): Speed multiplier for forward movement.
        """
        super().__init__(node, uav)

        self.approach_distance = approach_distance
        self.pass_distance = pass_distance
        self.speed_factor = speed_factor
        self.done = False
        self.hoop_centered = False
        self.passing_through = False
        self.hoop_pass_target = None
        self.initial_hoop_detection = False
        
        # Subscribe to hoop tracking data
        self.subscribe_to_vision('/vision/hoop_tracking', HoopTracking, 'hoop')

    def on_enter(self) -> None:
        """
        Called when this mode is activated.
        """
        self.log("HoopNavigationMode activated")
        self.done = False
        self.hoop_centered = False
        self.passing_through = False
        self.hoop_pass_target = None
        self.initial_hoop_detection = False

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for navigating through a hoop.
        """
        # If UAV is unstable, skip the update
        if self.uav.roll > 0.1 or self.uav.pitch > 0.1:
            self.log("Roll or pitch detected. Waiting for stabilization.")
            return

        # Get current altitude and position
        current_pos = self.uav.get_local_position()
        current_altitude = -current_pos[2]
        
        # Get latest hoop tracking data
        tracking_data = self.get_vision_data('hoop')
        
        # If no tracking data received yet, exit early
        if tracking_data is None:
            self.log("No tracking data received yet")
            return
        
        # Debug: Log current state
        self.log(f"Current Position: ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}), Altitude: {current_altitude:.2f}m")

        # If we're already passing through, just go straight
        if self.passing_through:
            if self.hoop_pass_target:
                dist_to_target = self.uav.distance_to_waypoint('LOCAL', self.hoop_pass_target)
                self.log(f"Passing through... Distance to target: {dist_to_target:.2f}m")
                self.uav.publish_position_setpoint(self.hoop_pass_target)
                if dist_to_target <= 0.3:
                    self.done = True
                    self.log("✓✓✓ Successfully passed through hoop! ✓✓✓")
            return

        # Check if hoop is detected
        if not tracking_data.hoop_detected:
            self.log("No hoop detected. Holding position.")
            # If we never detected a hoop, just hold position
            if not self.initial_hoop_detection:
                return
            # If we lost the hoop but detected it before, move forward slowly
            else:
                direction = [0, 0, 0]
                direction[2] = 0
                self.log("Lost hoop tracking. Moving forward slowly.")
                # Move forward in body frame
                forward_body = self.uav.uav_to_local((self.speed_factor * 0.5, 0, 0))
                self.uav.publish_position_setpoint(forward_body, relative=True)
                return
        
        # Mark that we've detected the hoop at least once
        self.initial_hoop_detection = True

        # Debug: Log raw camera data
        self.log(f"Raw Camera Direction: [{tracking_data.direction[0]:.3f}, {tracking_data.direction[1]:.3f}, {tracking_data.direction[2]:.3f}]")
        self.log(f"Pixel coords: x={tracking_data.x:.1f}, y={tracking_data.y:.1f}")

        # Convert direction vector from camera frame to UAV frame
        # tracking_data.direction is [x, y, z] in camera frame
        # Camera frame: x right, y down, z forward
        # UAV frame: x forward (North), y left (West), z up
        direction_camera = tracking_data.direction
        direction_uav_raw = [-direction_camera[1], direction_camera[0], -direction_camera[2]]
        self.log(f"Direction after camera->UAV conversion: [{direction_uav_raw[0]:.3f}, {direction_uav_raw[1]:.3f}, {direction_uav_raw[2]:.3f}]")
        
        # Apply camera offsets
        camera_offsets = tuple(x / current_altitude for x in self.uav.camera_offsets) if current_altitude > 1 else self.uav.camera_offsets
        camera_offsets_rotated = self.uav.uav_to_local(camera_offsets)
        self.log(f"Camera offsets (raw): {self.uav.camera_offsets}, scaled: {camera_offsets}, rotated: {camera_offsets_rotated}")
        
        direction = [x + y for x, y in zip(direction_uav_raw, camera_offsets_rotated)]
        self.log(f"Direction after camera offset: [{direction[0]:.3f}, {direction[1]:.3f}, {direction[2]:.3f}]")

        # Normalize direction
        direction_magnitude = np.linalg.norm(direction)
        if direction_magnitude > 0:
            direction_normalized = [d / direction_magnitude for d in direction]
        else:
            direction_normalized = [0, 0, 0]

        # Estimate distance to hoop (rough approximation based on altitude and direction)
        estimated_distance = current_altitude * direction_magnitude

        self.log(f"Direction magnitude: {direction_magnitude:.3f}, Normalized: [{direction_normalized[0]:.3f}, {direction_normalized[1]:.3f}, {direction_normalized[2]:.3f}]")
        self.log(f"Estimated distance to hoop: {estimated_distance:.2f}m")

        # Check if hoop is centered (within threshold)
        # Check x and y components of normalized direction
        centering_threshold = 0.05  # Adjust as needed
        x_error = abs(direction_normalized[0])
        y_error = abs(direction_normalized[1])
        is_centered = (x_error < centering_threshold and y_error < centering_threshold)
        
        self.log(f"Centering errors - X: {x_error:.3f}, Y: {y_error:.3f}, Threshold: {centering_threshold}, Centered: {is_centered}")

        if is_centered:
            self.log("✓ Hoop centered! Moving forward to pass through.")
            # Calculate target position to pass through hoop
            # Move forward in the direction of the hoop
            forward_distance = self.pass_distance
            forward_direction = self.uav.uav_to_local((forward_distance, 0, 0))
            self.hoop_pass_target = tuple(current_pos[i] + forward_direction[i] for i in range(3))
            self.log(f"Pass target set: ({self.hoop_pass_target[0]:.2f}, {self.hoop_pass_target[1]:.2f}, {self.hoop_pass_target[2]:.2f})")
            self.passing_through = True
        else:
            # Not centered yet - adjust position to center on hoop
            # Scale movement based on error
            movement_scale = min(1.0, direction_magnitude)
            adjusted_direction = [d * movement_scale for d in direction]
            
            # Slow down vertical movement for stability
            adjusted_direction[2] *= 0.5
            
            # Calculate target position
            target_pos = tuple(current_pos[i] + adjusted_direction[i] for i in range(3))
            distance_to_move = np.linalg.norm(adjusted_direction)
            
            self.log(f"→ Centering adjustment: [{adjusted_direction[0]:.3f}, {adjusted_direction[1]:.3f}, {adjusted_direction[2]:.3f}] (magnitude: {distance_to_move:.3f}m)")
            self.log(f"→ Target position: ({target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f})")
            self.uav.publish_position_setpoint(adjusted_direction, relative=True)

    def check_status(self) -> str:
        """
        Check the status of the hoop navigation.

        Returns:
            str: The status of the hoop navigation.
        """
        if self.done:
            return 'complete'
        return 'continue'
    
    def log(self, message: str):
        """
        Log a message with mode context.

        Args:
            message (str): The message to log.
        """
        self.node.get_logger().info(f"[HoopNavigationMode] {message}")

