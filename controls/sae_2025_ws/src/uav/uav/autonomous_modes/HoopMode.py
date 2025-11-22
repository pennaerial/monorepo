import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import HoopTracking 
from uav.vision_nodes import HoopTrackingNode 
from typing import Optional, Tuple
import cv2

class HoopMode(Mode):
    """
    A mode that initiates by flying up, trying to find a hoop, and continues
    attempting to find and look for hoops.
    """

    def __init__(self, node: Node, uav: UAV, num_hoops: int = 1):
        """
        Initialize the LowerPayload.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            color (str): The color of the payload to track.
        """
        super().__init__(node, uav)

        self.response = None 
        self.altitude_constant: float = 0.25
        self.done: bool = False
        self.wait_time: float = 20.0 # delay in s
        self.num_hoops: int = num_hoops
        self.passed_hoops: list[tuple[float, float, float]] = []      # list of coordinates of all passed hoops
        self.goal_pos: tuple[float, float, float] = ()
        self.rotation_vec: tuple[float, float, float] = () 
        self.success: bool = False


    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for lowering payload and handling obstacles.
        """
        request = HoopTracking.Request() # TODO: change this to new vision node

        # fields: t_vec[3], r_vec[3], dlz_empty; returns None if no hoop
        response = self.send_request(HoopTrackingNode, request)

        # Time delta between takeoff and hoop mode running
        if time_delta > 1:
            time_delta = 0
        
        # If no hoop detected, start counting down the wait time
        if not response and len(self.goal_pos) == 0:
            direction = [0, 0, -self.altitude_constant] # start going upwards
            self.uav.publish_position_setpoint(direction, relative=True)
            self.wait_time -= time_delta

            if len(self.passed_hoops) >= self.num_hoops or self.wait_time <= 0: # if 20 seconds has elapsed with no response
                self.done = True
            self.log(f"Looking for hoop - waiting for {self.wait_time} more seconds")
            return
        
        self.wait_time = 20

        # if no goal_pos
        if len(self.goal_pos) == 0:
            self.success = response.success
            self.goal_pos = response.t_vec
            self.rotation_vec = response.r_vec

        if self.success:
            self.uav.publish_position_setpoint(self.goal_pos)
            if self.uav.distance_to_waypoint('LOCAL', self.goal_pos) <= 0.05:
                self.passed_hoops.append(self.goal_pos)
                self.goal_pos = () 
                if len(self.passed_hoops) >= self.num_hoops:
                    self.done = True
                    return
            return
        
        direction = [ -response.t_vec[1], response.t_vec[0], response.t_vec[2] ]
        
        camera_offsets = self.uav.camera_offsets
        direction = [x + y for x, y in zip(direction, self.uav.uav_to_local(camera_offsets))]


        self.log(f"Direction: {self.goal_pos}")
        self.uav.publish_position_setpoint(self.goal_pos, relative=True)
    
    def check_status(self) -> str:
        """
        Check the status of the payload lowering.

        Returns:
            str: The status of the payload lowering.
        """
        if self.done:
            return 'complete'
        return 'continue'