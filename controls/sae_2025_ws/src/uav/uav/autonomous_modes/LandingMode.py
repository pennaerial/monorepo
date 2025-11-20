import cv2
import numpy as np
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV
from px4_msgs.msg import VehicleStatus
from uav_interfaces.srv import Landing
from uav.vision_nodes import LandingNode


class LandingMode(Mode):
    """
    Landing mode that tilts until we detect the landing pad, and then goes and lands.
    """

    def __init__(self, node: Node, uav: UAV, color: str = 'red'):
        """
        Initialize the LandingMode

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            color (str): The color of the landing pad to detect.
        """
        super().__init__(node, uav)
        self.color = color
        self.done = False
        self.landing_initiated = False
        self.search_pitch = 0.4  # pitch angle to search in radians - can adjust this if we end up needing to

    def on_update(self, time_delta: float) -> None:
        """
        Tilt and search for pad, then follow translation vector to land
        """
        # check if landed
        if self.landing_initiated:
            if self.uav.vehicle_status != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self.done = True
            return
        
        # ask CV for landing pad detection (change this with exact names when i know them)
        altitude = -self.uav.get_local_position()[2]
        request = Landing.Request()
        
        # if no pad detected, tilt and search
        response = self.send_request(LandingNode, request)

        if response is None:
            self._search_for_pad()
            return
        
        # get ouputted translation & rotation vectors
        tvec = np.array(response.t_vec)  
        rvec = np.array(response.r_vec)     
        
        # use rotation vector to get pitch needed for pad
        pitch_from_rvec = self._get_pitch_from_rotation(rvec)
        
        # convert camera translational vector to ned & modify translational vector using pitch
        ned_direction = self._transform_to_ned(tvec, pitch_from_rvec)
        
        # get the horizontal distance
        horizontal_dist = np.sqrt(ned_direction[0]**2 + ned_direction[1]**2)
        
        self.log(f"Pad at {ned_direction}, dist: {horizontal_dist:.2f}m") #added a couple log statements in case we need to debug
        
        # if close & aligned at low altitutde, land
        if altitude < 1.0 and horizontal_dist < 0.15:
            self.log("Landing!")
            self.uav.land()
            self.landing_initiated = True
            return
        
        # if too high, move toward pad but maintain pitch to see it
        if altitude < 1.0:
            #altitude is good, move horizontally
            ned_direction[2] = 0
        else:
            # altitude is higher, go down slowly
            ned_direction[2] = 0.3 * time_delta
        
        # keep pitch from rotation vector to keep pad in view
        self.uav.publish_attitude_setpoint(
            roll=0.0,
            pitch=pitch_from_rvec,
            yaw=self.uav.yaw,
            thrust=0.5
        )
        
        self.uav.publish_position_setpoint(ned_direction, relative=True)

    def _search_for_pad(self) -> None:
        """
        pitch forward and rotate slowly to search for landing pad.
        """
        
        # pitch forward to look at ground and rotate slowly to scan the area
        self.uav.publish_attitude_setpoint(
            roll=0.0,
            pitch=self.search_pitch,
            yaw=self.uav.yaw + 0.05,  
            thrust=0.5
        )

    def _get_pitch_from_rotation(self, rvec: np.ndarray) -> float:
        """
        get pitch angle from rotation vector to keep pad in camera view.
        
        args:
            rvec: Rotation vector from CV
            
        returns:
            float: Pitch angle in radians
        """
        rot_mat, _ = cv2.Rodrigues(rvec)
        
        pitch = np.arcsin(-rot_mat[2, 0])
        
        return np.clip(pitch, 0.2, 1.0)

    def _transform_to_ned(self, tvec: np.ndarray, pitch: float) -> list:
        """
        Change camera translation vector to NED frame.
        
        Args:
            tvec: Translation [x, y, z] in camera frame
            pitch: Current pitch angle
            
        Returns:
            list: [forward, right, down] in NED frame
        """
        x_cam, y_cam, z_cam = tvec
        
        # rotate from camera frame to NED accounting for pitch
        ned_forward = z_cam * np.cos(pitch) - y_cam * np.sin(pitch)
        ned_right = x_cam
        ned_down = z_cam * np.sin(pitch) + y_cam * np.cos(pitch)
        
        return [ned_forward, ned_right, ned_down]

    def check_status(self) -> str:
        """Check status of landing."""
        if self.done:
            return "landed"
        else:
            return "continue"
