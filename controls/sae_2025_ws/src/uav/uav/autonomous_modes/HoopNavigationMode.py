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

        # Get current altitude
        current_altitude = -self.uav.get_local_position()[2]
        
        # Get latest hoop tracking data
        tracking_data = self.get_vision_data('hoop')
        
        # If no tracking data received yet, exit early
        if tracking_data is None:
            return

        # If we're already passing through, just go straight
        if self.passing_through:
            if self.hoop_pass_target:
                self.uav.publish_position_setpoint(self.hoop_pass_target)
                if self.uav.distance_to_waypoint('LOCAL', self.hoop_pass_target) <= 0.3:
                    self.done = True
                    self.log("Successfully passed through hoop!")
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

        # Convert direction vector from camera frame to UAV frame
        # tracking_data.direction is [x, y, z] in camera frame
        # Camera frame: x right, y down, z forward
        # UAV frame: x forward (North), y left (West), z up
        direction = [-tracking_data.direction[1], tracking_data.direction[0], -tracking_data.direction[2]]
        
        # Apply camera offsets
        camera_offsets = tuple(x / current_altitude for x in self.uav.camera_offsets) if current_altitude > 1 else self.uav.camera_offsets
        direction = [x + y for x, y in zip(direction, self.uav.uav_to_local(camera_offsets))]

        # Normalize direction
        direction_magnitude = np.linalg.norm(direction)
        if direction_magnitude > 0:
            direction_normalized = [d / direction_magnitude for d in direction]
        else:
            direction_normalized = [0, 0, 0]

        # Estimate distance to hoop (rough approximation based on altitude and direction)
        estimated_distance = current_altitude * direction_magnitude

        self.log(f"Hoop detected. Direction: {direction}, Est. distance: {estimated_distance:.2f}m")

        # Check if hoop is centered (within threshold)
        # Check x and y components of normalized direction
        centering_threshold = 0.05  # Adjust as needed
        is_centered = (abs(direction_normalized[0]) < centering_threshold and 
                      abs(direction_normalized[1]) < centering_threshold)

        if is_centered:
            self.log("Hoop centered! Moving forward to pass through.")
            # Calculate target position to pass through hoop
            current_pos = self.uav.get_local_position()
            # Move forward in the direction of the hoop
            forward_distance = self.pass_distance
            forward_direction = self.uav.uav_to_local((forward_distance, 0, 0))
            self.hoop_pass_target = tuple(current_pos[i] + forward_direction[i] for i in range(3))
            self.passing_through = True
        else:
            # Not centered yet - adjust position to center on hoop
            # Scale movement based on error
            movement_scale = min(1.0, direction_magnitude)
            adjusted_direction = [d * movement_scale for d in direction]
            
            # Slow down vertical movement for stability
            adjusted_direction[2] *= 0.5
            
            self.log(f"Centering on hoop. Adjusted direction: {adjusted_direction}")
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

