import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import PayloadTracking
from uav.vision_nodes import PayloadTrackingNode
from typing import Optional, Tuple
import cv2

class HoopMode(Mode):
    """
    A mode that initiates by flying up, trying to find a hoop, and continues
    attempting to find and look for hoops.
    """

    def __init__(self, node: Node, uav: UAV, color: str = 'orange'):
        """
        Initialize the LowerPayload.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            color (str): The color of the payload to track.
        """
        super().__init__(node, uav)

        self.response = None
        self.altitude_constant = 1
        self.wait_time = 20_000         # delay in ms
        self.done = False
        self.color = color          # might be useful as a way to confirm
        self.goal_pos = None
        self.passed_hoops = []      # list of coordinates of all passed hoops

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for lowering payload and handling obstacles.
        """
        # If UAV is unstable, skip the update
        if self.uav.roll > 0.1 or self.uav.pitch > 0.1:
            self.log("Roll or pitch detected. Waiting for stabilization.")
            return
        
        request = PayloadTracking.Request() # TODO: change this to new vision node
        request.altitude = -self.uav.get_local_position()[2]
        request.yaw = float(self.uav.yaw)
        response = self.send_request(PayloadTrackingNode, request)
        
        # If no payload pose is received, exit early
        if response is None:
            direction = [0, 0, -self.altitude_constant] # start going upwards
            self.uav.publish_position_setpoint(direction, relative=True)
            self.wait_time -= time_delta

            if self.wait_time <= 0: # if 20 seconds has elapsed with no response
                self.done = True
            return
        
        self.wait_time = 20_000
        self.goal_pos = self.fetch_cv_coordinates()

        if self.goal_pos:
            self.uav.publish_position_setpoint(self.goal_pos)
            if self.uav.distance_to_waypoint('LOCAL', self.goal_pos) <= 0.05:
                # why is there an "or True" here? what is dlz?
                if response.dlz_empty or True:
                    self.done = True
                else:
                    pass # TODO: Extend servo
            return
        
        direction = [ -response.direction[1], response.direction[0], response.direction[2] ]
        
        camera_offsets = tuple(x / request.altitude for x in self.uav.camera_offsets) if request.altitude > 1 else self.uav.camera_offsets
        direction = [x + y for x, y in zip(direction, self.uav.uav_to_local(camera_offsets))]

        self.log(f"Direction: {direction}")
        self.uav.publish_position_setpoint(direction, relative=True)
    
    def fetch_cv_coordinates(self) -> tuple[float, float, float] | None:
        pass # TODO: implement call to cv once pushed

    def check_status(self) -> str:
        """
        Check the status of the payload lowering.

        Returns:
            str: The status of the payload lowering.
        """
        if self.done:
            return 'complete'
        return 'continue'