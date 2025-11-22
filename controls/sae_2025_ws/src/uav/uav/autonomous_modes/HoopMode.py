import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import HoopTracking 
from uav_interfaces.msg import DroneState
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

        self.hoop_detected: bool = False
        self.hoop_position: Optional[np.ndarray] = None
        
        self.state: str = "searching" #should be either "searching", "centering", or "flying_through"

        self.center_threshold_y: float = 0.1  #modify this if it doesn't work
        self.center_threshold_z: float = 0.1  #modify this if it doesn't work
        self.distance_threshold: float = 0.05  #distance to hoop center

        self.drone_state: Optional[DroneState] = None
        self.drone_position: Optional[np.ndarray] = None
        self.drone_velocity: Optional[np.ndarray] = None

        ##self.goal_pos: tuple[float, float, float] = ()
        self.rotation_vec: tuple[float, float, float] = () 
        self.success: bool = False

        self.state_subscription = self.node.create_subscription(
            DroneState,
            '/drone_state',
            self.state_callback,
            10
        )

    def state_callback(self, msg: DroneState):
        """
        Callback function to update the drone's state.

        Args:
            msg (DroneState): The message containing the drone's state.
        """
        self.drone_state = msg
        self.drone_position = np.array([
            msg.position.x, 
            msg.position.y, 
            msg.position.z])
        self.drone_velocity = np.array([
            msg.velocity.x,
            msg.velocity.y, 
            msg.velocity.z])


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
        
        if self.state == "searching":
            self._handle_searching(response, time_delta)
        elif self.state == "centering":
            self._handle_centering(response)
        elif self.state == "flying_through":
            self._handle_flying_through()

    def _handle_searching(self, response, time_delta: float):
        "fly upward to look for hoops"

        if response and response.success:
            self.state = "centering"
            self.hoop_detected = True
            self.hoop_position = response.t_vec
            self.log("Hoop detected, switching to centering mode")
            return
        
        direction = [0, 0, -self.altitude_constant] # start going upwards
        self.uav.publish_position_setpoint(direction, relative=True)
        self.wait_time -= time_delta

        if len(self.passed_hoops) >= self.num_hoops or self.wait_time <= 0: # if 20 seconds has elapsed with no response
            self.done = True
            self.log("Search timeout or all hoops passed.")
            return
        
        self.log(f"Looking for hoop - waiting for {self.wait_time} more seconds")
        
    def _handle_centering(self, response):
        "center the hoop in y and z and then move forward in x"
        if not response or not response.success:
            self.log("Lost hoop, switching to searching mode")
            self.state = "searching"
            self.wait_time = 20
            return
        
        self.log(self.hoop_position)
            
        hoop_ned = np.array([
            -self.hoop_position[1],
            self.hoop_position[0],
            -self.hoop_position[2]
        ])

        # -response.direction[1], response.direction[0],
        #                 response.direction[2]

        centered_y = abs(hoop_ned[1]) <= self.center_threshold_y
        centered_z = abs(hoop_ned[2]) <= self.center_threshold_z

        if centered_y and centered_z:
            self.state = "flying_through"
            self.log("Hoop centered, switching to flying through mode")
            return
        
        correction = np.array([
            0.0,
            hoop_ned[1],
            hoop_ned[2]
        ])

        # self.log("centering - y offset: {hoop_ned[1]:.3f}, z offset: {hoop_ned[2]:.3f}")
        self.uav.publish_position_setpoint(correction.tolist(), relative=True)
        self.log(f"Centering hoop - y offset: {hoop_ned[1]:.3f}, z offset: {hoop_ned[2]:.3f}")

    def _handle_flying_through(self):
        "fly forward through the centered hoop"
        if self.hoop_position is None:
            self.log("Lost hoop, switching to searching mode")
            self.state = "searching"
            return
        
        forward_distance = self.hoop_position[0] + 0.5  # added buffer of 0.5m but we can change if needed

        #moving forward to hoop center
        self.log("Flying through hoop")
        direction = [0.0, forward_distance, -0.04]
        self.uav.publish_position_setpoint(direction, relative=True)

        if self.drone_position is not None:
            distance_to_hoop = np.linalg.norm(self.drone_position - np.array([
                self.drone_position[0] + forward_distance,
                self.drone_position[1],
                self.drone_position[2]
            ]))

            if distance_to_hoop <= self.distance_threshold:
                self.log("Successfully flew through hoop")
                self.passed_hoops.append((
                    self.drone_position[0] + forward_distance,
                    self.drone_position[1],
                    self.drone_position[2]
                ))
                if len(self.passed_hoops) >= self.num_hoops:
                    self.done = True
                    self.log("All hoops passed, mission complete")
                    return
                else:
                    self.state = "searching"
                    self.wait_time = 20
                    self.hoop_position = None
                return
        

        ##self.wait_time = 20

        # if no goal_pos
        ##if len(self.goal_pos) == 0:
            ##self.success = response.success
            ##self.goal_pos = response.t_vec
           ## self.rotation_vec = response.r_vec

        ##if self.success:
           ## self.uav.publish_position_setpoint(self.goal_pos)
           ## if self.uav.distance_to_waypoint('LOCAL', self.goal_pos) <= 0.05:
           ##     self.passed_hoops.append(self.goal_pos)
           ##     self.goal_pos = () 
           ##     if len(self.passed_hoops) >= self.num_hoops:
         ##           self.done = True
           ##         return
          ##  return
        
       ## direction = [ -response.t_vec[1], response.t_vec[0], response.t_vec[2] ]
        
       ## camera_offsets = self.uav.camera_offsets
       ## direction = [x + y for x, y in zip(direction, self.uav.uav_to_local(camera_offsets))]


       ## self.log(f"Direction: {self.goal_pos}")
       ## self.uav.publish_position_setpoint(self.goal_pos, relative=True)#
    
    def check_status(self) -> str:
        """
        Check the status of the payload lowering.

        Returns:
            str: The status of the payload lowering.
        """
        if self.done:
            return 'complete'
        return 'continue'