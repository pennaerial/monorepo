import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import HoopTracking
from uav.vision_nodes import HoopTrackingNode
from typing import Optional, Tuple
from px4_msgs.msg import VehicleStatus
import cv2

class HoopMode(Mode):
    """
    A mode for flying through a hoop.
    """

    def __init__(self, node: Node, uav: UAV, offsets: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)):
        """
        Initialize the HoopMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            offsets (Optional[Tuple[float, float, float]]):
                Should denote the position of flight path relative to the center of hoop, in meters
                In NED frame: x is forward, y is right, and z is down.
        """
        super().__init__(node, uav)

        self.response = None
        self.done = False
        self.offsets = offsets
        self.camera_offsets = self.uav.camera_offsets
        self.mode = 0 # 0 for uav centering, 1 for landing, 2 for retracting, 3 for taking off
        # New: 0 for uav centering, 1 for flying to hoop center, 2 for through hoop, 3 for done

        # VZ edits - minimal distance needed to clear hoop
        # Hard set for now, should add 0.5 meters to calcualted distance
        self.min_travel_distance = 1.5 # meters



    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for finding hoop and flying through it.
        """
        self.log(f"=== MODE {self.mode} ===")

        # If UAV is unstable, skip the update
        if self.uav.roll > 0.1 or self.uav.pitch > 0.1:
            self.log("Roll or pitch detected. Waiting for stabilization.")
            return

        # Mode transitions for post-hoop states
        if self.mode == 1:
            # After starting forward motion, advance to clearing phase
            self.mode = 2
            self.forward_start_pos = self.uav.get_local_position()
            self.min_travel_distance = self.calculate_distance()
            return

        # Older version
        # if self.mode == 2:
        #     # Check if we've traveled enough distance forward (e.g., 2 meters)
        #     current_pos = self.uav.get_local_position()
        #     distance_traveled = np.linalg.norm(
        #         np.array(current_pos[:2]) - np.array(self.forward_start_pos[:2])
        #     )
        #     self.log(f"Distance traveled: {distance_traveled:.2f}m")
        #     if distance_traveled > self.min_travel_distance:
        #         self.mode = 3
        #         self.done = True
        #         self.log("DONE! Clearing complete")
        #     else:
        #         # Keep moving forward
        #         self.log("Mode 2: Publishing (0, 1, 0)")
        #         self.uav.publish_position_setpoint((0, 0.5, 0), relative=True)
        #     return

        if self.mode == 2:
            # Pose-aware clearing: use hoop_body / distance instead of dead-reckoning odom
            if hoop_body is None:
                # No pose available → simple fallback forward
                self.log("Mode 2: No PnP pose, moving forward fallback.")
                self.uav.publish_position_setpoint((1, 0, 0), relative=True)
                return

            forward_dist = hoop_body[0]  # meters in front of UAV along X_body
            self.log(f"Mode 2: forward_dist={forward_dist:.2f} m, total_dist={distance:.2f} m")

            # While hoop center is clearly ahead, keep moving forward
            clearance_front = 0.5  # still in front if X_body > 0.5 m
            if forward_dist > clearance_front:
                self.log("Mode 2: Hoop ahead, moving forward.")
                self.uav.publish_position_setpoint((1, 0, 0), relative=True)
                return

            # Once hoop is at or behind us, we consider ourselves through it
            self.log("Mode 2: Hoop plane passed (PnP). Marking done.")
            self.mode = 3
            self.done = True
            return

        if self.mode == 3:
            return  # Done, waiting for transition
    
        request = HoopTracking.Request()
        request.altitude = -self.uav.get_local_position()[2]
        request.yaw = float(self.uav.yaw)
        request.payload_color = 'red'
        response = self.send_request(HoopTrackingNode, request)
        
        # If no hoop pose is received, exit early
        if response is None:
            return

        # edits by VZ
        # If PnP pose is available, compute hoop vectors
        hoop_cam = hoop_body = hoop_local = None
        distance = None
        if getattr(response, "has_pose", False):
            hoop_cam, hoop_body, hoop_local, distance = self._get_hoop_vectors(response)
            self.log(
                f"Hoop pose: cam={hoop_cam}, body={hoop_body}, "
                f"local={hoop_local}, dist={distance:.2f} m"
            )


        # Transform from camera frame to UAV NED frame (forward-facing camera)
        # Camera: X=right, Y=down, Z=forward -> UAV NED: X=forward, Y=right, Z=down
        # Camera Y is down (positive), UAV Z is down (positive), so direct mapping
        # Camera X is right (positive), UAV Y is right (positive), so direct mapping
        # Camera Z is forward (positive), UAV X is forward (positive), so direct mapping
        self.log(f"Pixel coords: x={response.x}, y={response.y}")
        self.log(f"Raw response.direction: {response.direction}")
        direction = [response.direction[0], response.direction[2], response.direction[1]]
        self.log(f"After frame transform: {direction}")

        offsets = tuple(x / request.altitude for x in self.offsets) if request.altitude > 1 else self.offsets
        camera_offsets = tuple(x / request.altitude for x in self.camera_offsets) if request.altitude > 1 else self.camera_offsets
        direction = [x + y + z for x, y, z in zip(direction, offsets, self.uav.uav_to_local(camera_offsets))]

        # Check if centered on hoop (left/right and up/down)
        threshold = 0.05
        if (np.abs(direction[0]) < threshold and
            np.abs(direction[2]) < threshold):
            # Centered! Fly forward through the hoop
            self.log(f"CENTERED! Publishing: (0, 1, 0)")
            self.uav.publish_position_setpoint((0, 0.5, 0), relative=True)
            self.mode = 1
            return


        #edits 
        # If we have a good PnP pose, use that for centering
        if hoop_body is not None:
            # hoop_body: [X_forward, Y_right, Z_down] in meters
            # We want Y,Z → 0 (centered left/right and up/down)
            lateral_gain = 0.5  # tune
            vy = -lateral_gain * hoop_body[1]  # move opposite the error
            vz = -lateral_gain * hoop_body[2]

            # Don't advance forward in centering mode (X=0 here)
            cmd_body = np.array([0.0, vy, vz], dtype=float)

            # Optionally saturate the command
            max_step = 0.5
            norm = np.linalg.norm(cmd_body[1:])
            if norm > max_step:
                cmd_body[1:] *= max_step / norm

            self.log(f"Centering using PnP: body cmd={cmd_body}, hoop_body={hoop_body}")

            self.uav.publish_position_setpoint(cmd_body.tolist(), relative=True)

            # Check centered condition in meters instead of arbitrary threshold
            center_thresh_m = 0.1  # within 10 cm
            if abs(hoop_body[1]) < center_thresh_m and abs(hoop_body[2]) < center_thresh_m:
                self.log("CENTERED (PnP). Switching to mode 1 (start forward).")
                self.uav.publish_position_setpoint((1, 0, 0), relative=True)  # small forward push
                self.mode = 1
            return

        # Fallback: old direction-based centering if no PnP pose
        self.log(f"Pixel coords: x={response.x}, y={response.y}")
        self.log(f"Raw response.direction: {response.direction}")
        direction = [response.direction[0], response.direction[2], response.direction[1]]


        direction[0] = -direction[0]/3  # Left / Right
        direction[1] = 0  # Forward  / Back
        direction[2] = direction[2]/3 # Up / Down 
        
        # Not centered yet - adjust position without moving forward
        # The direction vector points FROM drone TO hoop, so we use it directly to move toward the hoop
        # direction[0] = 0  # Zero out forward movement until centered
        # direction[1] = direction[1]  
        # direction[2] = direction[2]  


        self.log(f"Centering - Publishing direction: {direction}")
        self.uav.publish_position_setpoint(direction, relative=True)

    def check_status(self) -> str:
        """
        Check the status of the payload lowering.

        Returns:
            str: The status of the payload lowering.
        """
        if self.done:
            return 'complete'
        return 'continue'

#----------- VZ edits - helper functions for PnP to estimate depth ------------------
    def _hoop_cam_to_body(self, hoop_cam: np.ndarray) -> np.ndarray:
        """
        Convert hoop center from OpenCV camera frame to UAV body frame (FRD).
        Camera: X=right, Y=down, Z=forward
        Body:   X=forward, Y=right, Z=down
        """
        Xc, Yc, Zc = hoop_cam  # camera frame
        Xb = Zc  # forward
        Yb = Xc  # right
        Zb = Yc  # down
        return np.array([Xb, Yb, Zb], dtype=float)

    def _get_hoop_vectors(self, response) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        """
        From HoopTracking response, compute:
        - hoop_cam   (3,) in camera frame
        - hoop_body  (3,) in UAV body frame
        - hoop_local (3,) in local NED frame
        - distance   scalar (m) from UAV to hoop center
        """
        hoop_cam = np.array(
            [response.hoop_cam_x, response.hoop_cam_y, response.hoop_cam_z],
            dtype=float,
        )

        hoop_body = self._hoop_cam_to_body(hoop_cam)

        # Rotate body vector to local NED using UAV attitude
        hoop_local = self.uav.uav_to_local(hoop_body)

        distance = float(np.linalg.norm(hoop_body))  # camera and UAV are close enough

        return hoop_cam, hoop_body, hoop_local, distance