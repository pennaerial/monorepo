import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import HoopTracking
from uav.vision_nodes import HoopTrackingNode
from typing import Optional, Tuple
import cv2

class HoopTrackingMode(Mode):
    """
    A mode for tracking and flying through hoops.
    """

    def __init__(self, node: Node, uav: UAV, hoop_tolerance: float = 1.5):
        """
        Initialize the HoopTrackingMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            hoop_tolerance (float): Distance threshold to consider hoop as reached (in meters).
        """
        super().__init__(node, uav)

        self.response = None
        self.altitude_constant = 3
        self.done = False
        self.hoop_tolerance = hoop_tolerance
        self.goal_pos = None

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for tracking and navigating through hoops.
        """
        # If UAV is unstable, skip the update
        if self.uav.roll > 0.1 or self.uav.pitch > 0.1:
            self.log("Roll or pitch detected. Waiting for stabilization.")
            return
          
        self.log("HoopTrackingMode: Requesting hoop detection...")
        
        # Create and send tracking request
        request = HoopTracking.Request()
        request.altitude = -self.uav.get_local_position()[2]
        request.yaw = float(self.uav.yaw)
        response = self.send_request(HoopTrackingNode, request)
        
        # If no response received, exit early
        if response is None:
            self.log("HoopTrackingMode: No response from HoopTrackingNode!")
            return
        
        self.log(f"HoopTrackingMode: Response received - detected={response.detected}, x={response.x}, y={response.y}")

        # If we've already set a goal position (passed through hoop), hold position
        if self.goal_pos:
            self.uav.publish_position_setpoint(self.goal_pos)
            if self.uav.distance_to_waypoint('LOCAL', self.goal_pos) <= 0.05:
                self.done = True
            return
        
        # Transform direction vector to UAV local frame
        direction = [-response.direction[1], response.direction[0],
                        response.direction[2] / self.altitude_constant]
        
        # Apply camera offset transformation
        camera_offsets = tuple(x / request.altitude for x in self.uav.camera_offsets) if request.altitude > 1 else self.uav.camera_offsets
        direction = [x + y for x, y in zip(direction, self.uav.uav_to_local(camera_offsets))]

        # Altitude-based approach logic
        if request.altitude < self.hoop_tolerance:
            # If we're centered on the hoop, fly through it
            if (np.abs(direction[0]) < self.hoop_tolerance / 10 and
                np.abs(direction[1]) < self.hoop_tolerance / 10):
                # Close enough - set goal to current position (stop and mark complete)
                self.goal_pos = self.uav.get_local_position()
                self.log("Hoop reached! Stopping.")
                return
            else:
                # Align horizontally, don't change altitude
                direction[2] = 0
        
        # If hoop not detected, try to search by moving forward slowly
        if not response.detected:
            self.log("No hoop detected. Searching...")
            # Move forward slowly to search
            direction = [0.5, 0, 0]  # Move forward in body frame
        
        self.log(f"Direction: {direction}, Detected: {response.detected}")
        self.uav.publish_position_setpoint(direction, relative=True)
    
    def check_status(self) -> str:
        """
        Check the status of the hoop tracking.

        Returns:
            str: The status of the hoop tracking.
        """
        if self.done:
            return 'complete'
        return 'continue'