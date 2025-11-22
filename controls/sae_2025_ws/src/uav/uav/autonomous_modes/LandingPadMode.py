import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
import math
import rclpy
from std_msgs.msg import Float64MultiArray
from typing import Optional
from uav.vision_nodes import LandingPadTrackingNode  # Import for launch system detection

class LandingPadMode(Mode):
    """
    Mode for navigating to and landing on a colored landing pad.
    Listens to /landing_pad_tracking and moves the UAV to center over the pad,
    then descends and lands.
    """

    def __init__(self, node: Node, uav: UAV, color: str = 'red', approach_altitude: float = 3.0, landing_threshold: float = 0.15):
        """
        Initialize the LandingPadMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            color (str): Color of the landing pad ('red', 'green', or 'blue').
            approach_altitude (float): Altitude to maintain while approaching the pad (meters).
            landing_threshold (float): Area ratio threshold to start landing (0-1).
        """
        super().__init__(node, uav)
        self.color = color
        self.approach_altitude = approach_altitude
        self.landing_threshold = landing_threshold
        self.latest_msg = None
        self.STATE = 'searching'  # searching, aligning, descending, landing
        self.alignment_start_time = None
        self.alignment_duration = 1.0  # seconds to maintain alignment before descending
        self.descent_start_time = None
        self.descent_duration = 2.0  # seconds to descend before final landing
        
        # Set the color parameter for the vision node
        node.declare_parameter('landing_pad_color', color)
        node.set_parameters([rclpy.parameter.Parameter('landing_pad_color', rclpy.Parameter.Type.STRING, color)])
        
        self.sub = node.create_subscription(
            Float64MultiArray,
            '/landing_pad_tracking',
            self._pad_cb,
            10
        )

    def _pad_cb(self, msg):
        """Callback for landing pad tracking messages."""
        # msg.data = [cx, cy, dir_x, dir_y, dir_z, area_ratio, detected_flag]
        if len(msg.data) >= 7:
            self.latest_msg = msg.data

    def on_update(self, dt: float):
        """Periodic update logic."""
        if self.latest_msg is None:
            self.log("Waiting for landing pad detection...")
            # Hover in place while searching
            self.uav.hover()
            return
        
        detected_flag = self.latest_msg[6] if len(self.latest_msg) > 6 else 0.0
        
        if detected_flag < 0.5:
            # No pad detected
            self.log("Landing pad not detected. Searching...")
            self.STATE = 'searching'
            self.uav.hover()
            return
        
        # Pad detected - extract data
        cx, cy = self.latest_msg[0], self.latest_msg[1]
        dir_x, dir_y, dir_z = self.latest_msg[2], self.latest_msg[3], self.latest_msg[4]
        area_ratio = self.latest_msg[5] if len(self.latest_msg) > 5 else 0.0
        
        current_pos = self.uav.get_local_position()
        current_altitude = -current_pos[2]  # Convert NED to altitude
        
        self.log(f"State: {self.STATE}, Altitude: {current_altitude:.2f}m, Area ratio: {area_ratio:.3f}")
        
        # State machine logic
        if self.STATE == 'searching':
            # Transition to aligning when pad is detected
            self.STATE = 'aligning'
            self.alignment_start_time = None
        
        if self.STATE == 'aligning':
            # Align laterally (x and z axes) and maintain approach altitude
            if self.alignment_start_time is None:
                self.alignment_start_time = self.node.get_clock().now().nanoseconds
            
            # Check if aligned (centered over pad)
            lateral_error = math.sqrt(dir_x**2 + dir_z**2)
            
            if lateral_error < 0.1:  # Centered
                elapsed = (self.node.get_clock().now().nanoseconds - self.alignment_start_time) / 1e9
                if elapsed >= self.alignment_duration:
                    # Maintained alignment long enough, start descending
                    self.STATE = 'descending'
                    self.descent_start_time = None
                    self.log("Aligned! Starting descent.")
                else:
                    # Hold position while maintaining alignment
                    target_pos = (current_pos[0], current_pos[1], -self.approach_altitude)
                    self.uav.publish_position_setpoint(target_pos)
            else:
                # Not aligned yet - adjust position
                # Reset alignment timer if we lose alignment
                self.alignment_start_time = self.node.get_clock().now().nanoseconds
                
                # Move laterally to center over pad
                # Scale direction vector for smooth movement
                lateral_vel = np.array([dir_x * 0.5, 0.0, dir_z * 0.5]).astype('float32')
                self.uav.publish_velocity(lateral_vel)
                
                # Maintain approach altitude
                if abs(current_altitude - self.approach_altitude) > 0.2:
                    target_pos = (current_pos[0], current_pos[1], -self.approach_altitude)
                    self.uav.publish_position_setpoint(target_pos)
        
        elif self.STATE == 'descending':
            # Descend while maintaining lateral alignment
            if self.descent_start_time is None:
                self.descent_start_time = self.node.get_clock().now().nanoseconds
            
            # Check if we're close enough (large area ratio) or have descended enough
            elapsed = (self.node.get_clock().now().nanoseconds - self.descent_start_time) / 1e9
            
            if area_ratio > self.landing_threshold or elapsed >= self.descent_duration:
                # Close enough or time elapsed - initiate landing
                self.STATE = 'landing'
                self.log("Initiating landing sequence.")
            else:
                # Continue descending slowly
                target_altitude = max(0.5, current_altitude - 0.2 * dt)  # Descend at 0.2 m/s
                target_pos = (current_pos[0], current_pos[1], -target_altitude)
                self.uav.publish_position_setpoint(target_pos)
                
                # Fine-tune lateral position
                lateral_error = math.sqrt(dir_x**2 + dir_z**2)
                if lateral_error > 0.05:
                    lateral_vel = np.array([dir_x * 0.3, 0.0, dir_z * 0.3]).astype('float32')
                    self.uav.publish_velocity(lateral_vel)
        
        elif self.STATE == 'landing':
            # Final landing - use PX4 landing command
            self.uav.land()
            self.done = True

    def check_status(self):
        """Check if mode is complete."""
        if getattr(self, 'done', False):
            return 'complete'
        return 'continue'

