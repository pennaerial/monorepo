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
        self.hoop_position: Optional[np.ndarray] = None  # hoop position in camera frame (t_vec)
        self.hoop_local_position: Optional[np.ndarray] = None  # hoop position in local NED frame
        
        self.state: str = "searching" #should be either "searching", "centering", or "flying_through"

        self.center_threshold_y: float = 0.1  #modify this if it doesn't work
        self.center_threshold_z: float = 0.1  #modify this if it doesn't work
        self.distance_threshold: float = 0.3  #distance to hoop center (increased for better detection)
        self.pass_through_distance: float = 0.5  # distance past hoop center to consider passed

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
    
    def _update_hoop_local_position(self):
        """
        Convert hoop position from camera frame (t_vec) to local NED frame.
        Camera frame: X forward, Y left, Z up
        NED frame: X North (forward), Y East (right), Z Down
        """
        if self.hoop_position is None or self.drone_position is None:
            return
        
        # Convert camera frame to NED frame
        # Camera X -> NED X (forward, same)
        # Camera Y -> NED -Y (left to right, flip)
        # Camera Z -> NED -Z (up to down, flip)
        hoop_ned_offset = np.array([
            self.hoop_position[0],   # X forward (same)
            -self.hoop_position[1],  # Y right (flip from left)
            -self.hoop_position[2]   # Z down (flip from up)
        ])
        
        # Hoop position in local frame = current drone position + offset
        self.hoop_local_position = self.drone_position + hoop_ned_offset

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
            self._handle_centering(response, time_delta)
        elif self.state == "flying_through":
            self._handle_flying_through(response, time_delta)

    def _handle_searching(self, response, time_delta: float):
        "fly upward to look for hoops"

        if response and response.success:
            self.state = "centering"
            self.hoop_detected = True
            self.hoop_position = np.array(response.t_vec)
            # Calculate hoop position in local NED frame
            self._update_hoop_local_position()
            self.log("Hoop detected, switching to centering mode")
            return
        
        direction = [0, 0, -self.altitude_constant] # start going upwards
        self.uav.publish_position_setpoint(direction, relative=True)
        self.wait_time -= time_delta

        if len(self.passed_hoops) >= self.num_hoops or self.wait_time <= 0: # if 20 seconds has elapsed with no response
            self.done = True
            self.log("Search timeout or all hoops passed.")
            return
        
        self.log(f"Looking for hoop - waiting for {self.wait_time:.1f} more seconds")
        
    def _handle_centering(self, response, time_delta: float):
        "center the hoop in y and z and then move forward in x"
        if not response or not response.success:
            self.log("Lost hoop, switching to searching mode")
            self.state = "searching"
            self.wait_time = 20.0
            self.hoop_position = None
            self.hoop_local_position = None
            return
            
        self.hoop_position = np.array(response.t_vec)
        # Update hoop position in local frame
        self._update_hoop_local_position()

        hoop_ned = np.array([
            self.hoop_position[0],
            -self.hoop_position[1],
            -self.hoop_position[2]
        ])

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

        self.log(f"Centering - y offset: {hoop_ned[1]:.3f}, z offset: {hoop_ned[2]:.3f}")
        self.uav.publish_position_setpoint(correction.tolist(), relative=True)

    def _handle_flying_through(self, response, time_delta: float):
        "fly forward through the centered hoop"
        # Continue tracking hoop if available
        if response and response.success:
            self.hoop_position = np.array(response.t_vec)
            self._update_hoop_local_position()
        
        # If we have a tracked hoop position, fly towards it
        if self.hoop_local_position is not None and self.drone_position is not None:
            # Calculate direction to hoop in local frame
            direction_to_hoop = self.hoop_local_position - self.drone_position
            
            # Check if we've passed through the hoop
            # We've passed through if we're past the hoop center in the forward (x) direction
            if direction_to_hoop[0] < -self.pass_through_distance:
                # We've passed through!
                self.log("Successfully flew through hoop")
                self.passed_hoops.append(tuple(self.hoop_local_position))
                
                if len(self.passed_hoops) >= self.num_hoops:
                    self.done = True
                    self.log("All hoops passed, mission complete")
                    return
                else:
                    # Reset for next hoop
                    self.state = "searching"
                    self.wait_time = 20.0
                    self.hoop_position = None
                    self.hoop_local_position = None
                    self.log(f"Passed hoop {len(self.passed_hoops)}/{self.num_hoops}, searching for next hoop")
                    return
            
            # Continue flying forward towards hoop
            # Use relative movement: move forward by the x distance to hoop plus a small buffer
            forward_distance = max(direction_to_hoop[0] + 0.2, 0.1)  # Always move forward, at least 0.1m
            direction = [forward_distance, 0.0, 0.0]
            self.uav.publish_position_setpoint(direction, relative=True)
            
            distance_to_hoop = np.linalg.norm(direction_to_hoop)
            self.log(f"Flying through hoop - distance: {distance_to_hoop:.3f}m, x-offset: {direction_to_hoop[0]:.3f}m")
        else:
            # Lost track of hoop - if we had a position before, check if we passed it
            if self.hoop_local_position is not None and self.drone_position is not None:
                # Check if we're past where the hoop was
                direction_to_last_known = self.hoop_local_position - self.drone_position
                if direction_to_last_known[0] < -self.pass_through_distance:
                    # We passed the hoop location
                    self.log("Passed through hoop (lost tracking)")
                    self.passed_hoops.append(tuple(self.hoop_local_position))
                    
                    if len(self.passed_hoops) >= self.num_hoops:
                        self.done = True
                        self.log("All hoops passed, mission complete")
                        return
                    else:
                        self.state = "searching"
                        self.wait_time = 20.0
                        self.hoop_position = None
                        self.hoop_local_position = None
                        self.log(f"Passed hoop {len(self.passed_hoops)}/{self.num_hoops}, searching for next hoop")
                        return
            
            # Continue forward if we don't know if we passed
            direction = [0.3, 0.0, 0.0]  # Move forward 0.3m
            self.uav.publish_position_setpoint(direction, relative=True)
            self.log("Flying forward - hoop position unknown")


    def check_status(self) -> str:
        """
        Check the status of the payload lowering.

        Returns:
            str: The status of the payload lowering.
        """
        if self.done:
            return 'complete'
        return 'continue'