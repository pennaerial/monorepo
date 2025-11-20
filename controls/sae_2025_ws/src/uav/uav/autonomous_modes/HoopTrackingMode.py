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
        self.passing_through = False
        self.forward_push_distance = 7.5  # required distance before we consider the hoop cleared
        self.forward_push_step = 0.3      # meters per update while passing through
        self.max_forward_push = 30.0       # safety cap
        self.push_distance_accum = 0.0
        self.no_hoop_frames = 0
        self.required_no_hoop_frames = 5

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for tracking and navigating through hoops.
        """
        # If UAV is unstable, skip the update
        # if self.uav.roll > 0.1 or self.uav.pitch > 0.1:
        #     self.log("Roll or pitch detected. Waiting for stabilization.")
        #     return
          
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

        align_vector, command_vector = self._compute_direction_vectors(response, request)

        if self.passing_through:
            self._continue_passing(response, command_vector)
            return

        if request.altitude < self.hoop_tolerance and response.detected:
            if (np.abs(align_vector[0]) < self.hoop_tolerance / 10 and
                np.abs(align_vector[1]) < self.hoop_tolerance / 10):
                self._start_passing_through()
                self._continue_passing(response, command_vector)
                return
            else:
                command_vector[2] = 0  # hold altitude when close

        self.log(f"Direction: {command_vector}, Detected: {response.detected}")
        self.uav.publish_position_setpoint(command_vector, relative=True)
    
    def _start_passing_through(self):
        self.log("Hoop centered! Driving forward to clear it.")
        self.passing_through = True
        self.push_distance_accum = 0.0
        self.no_hoop_frames = 0
    
    def _continue_passing(self, response: HoopTracking.Response, correction_vector):
        self.log("Advancing through hoop...")
        step = self.forward_push_step
        self.push_distance_accum += step
        command = list(correction_vector)
        command[0] += step
        self.uav.publish_position_setpoint(command, relative=True)
        
        if response.detected:
            self.no_hoop_frames = 0
        else:
            self.no_hoop_frames += 1
        
        hoop_cleared = (
            self.push_distance_accum >= self.forward_push_distance and
            self.no_hoop_frames >= self.required_no_hoop_frames
        )
        hit_safety_cap = self.push_distance_accum >= self.max_forward_push
        
        if hoop_cleared or hit_safety_cap:
            if hit_safety_cap:
                self.log("Reached forward push safety cap, assuming hoop cleared.")
            else:
                self.log("Hoop cleared (lost detection after pushing forward).")
            self.passing_through = False
            self.done = True

    def _compute_direction_vectors(self, response: HoopTracking.Response, request: HoopTracking.Request):
        direction = [
            -response.direction[1],
             response.direction[0],
             response.direction[2] / self.altitude_constant,
        ]

        camera_offsets = tuple(x / request.altitude for x in self.uav.camera_offsets) if request.altitude > 1 else self.uav.camera_offsets
        direction = [x + y for x, y in zip(direction, self.uav.uav_to_local(camera_offsets))]

        align_vector = direction.copy()

        if not response.detected:
            self.log("No hoop detected. Searching...")
            direction = [0.5, 0.0, 0.0]

        step_gain = 0.3
        command_vector = [d * step_gain for d in direction]
        return align_vector, command_vector
    
    def check_status(self) -> str:
        """
        Check the status of the hoop tracking.

        Returns:
            str: The status of the hoop tracking.
        """
        if self.done:
            return 'complete'
        return 'continue'