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

    def __init__(self, node: Node, uav: UAV,
                 offsets: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)):
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

        # 0 = centering, 1 = just started forward, 2 = traveling through, 3 = done
        self.mode = 0

        # VZ edits - minimal distance needed to clear hoop (forward-only distance)
        # Will be overwritten in mode 1 using PnP forward distance + 0.5 m clearance
        self.min_travel_distance = 1.5  # meters (fallback default)
        self.forward_start_pos = None   # local position at start of forward motion

    # --------- MAIN UPDATE LOOP ---------
    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for finding hoop and flying through it.
        """
        self.log(f"=== MODE {self.mode} ===")

        # If UAV is unstable, skip the update
        if self.uav.roll > 0.1 or self.uav.pitch > 0.1:
            self.log("Roll or pitch detected. Waiting for stabilization.")
            return

        # If we're already done, do nothing
        if self.mode == 3:
            return

        # ---------- Query vision (HoopTracking) ----------
        request = HoopTracking.Request()
        request.altitude = -self.uav.get_local_position()[2]
        request.yaw = float(self.uav.yaw)
        request.payload_color = 'red'
        response = self.send_request(HoopTrackingNode, request)

        # If no hoop pose is received, exit early
        if response is None:
            self.log("No HoopTracking response.")
            return

        # Save for use in other methods if needed
        self.response = response

        # Try to get PnP-based vectors if available
        hoop_cam = hoop_body = hoop_local = None
        distance = None
        if getattr(response, "has_pose", False):
            hoop_cam, hoop_body, hoop_local, distance = self._get_hoop_vectors(response)
            self.log(
                f"Hoop pose (PnP): cam={hoop_cam}, body={hoop_body}, "
                f"local={hoop_local}, dist={distance:.2f} m"
            )

        # ---------- MODE 1: compute forward distance & latch min_travel_distance ----------
        if self.mode == 1:
            # We assume we have just started moving forward through the hoop.
            # Here we only compute the forward distance to the hoop center and
            # set self.min_travel_distance to that + 0.5 m.
            self.forward_start_pos = self.uav.get_local_position()

            if hoop_body is not None:
                forward_dist = float(hoop_body[0])  # meters in front of UAV along X_body
                if forward_dist < 0.0:
                    # If PnP says hoop is behind us (weird), clamp to 0
                    forward_dist = 0.0
                self.min_travel_distance = forward_dist + 0.5  # add 0.5 m clearance
                self.log(
                    f"[MODE 1] Forward distance to hoop: {forward_dist:.2f} m, "
                    f"min_travel_distance set to {self.min_travel_distance:.2f} m"
                )
            else:
                # No PnP pose available: keep whatever self.min_travel_distance was
                self.log(
                    "[MODE 1] No PnP pose; keeping fallback "
                    f"min_travel_distance={self.min_travel_distance:.2f} m"
                )

            # Switch to travel-through mode
            self.mode = 2
            return

        # ---------- MODE 2: use odometry to travel min_travel_distance ----------
        if self.mode == 2:
            if self.forward_start_pos is None:
                # Safety: if somehow we never latched start position, do it now
                self.forward_start_pos = self.uav.get_local_position()

            current_pos = self.uav.get_local_position()

            # Compute distance traveled in the horizontal plane (N,E) from start
            start_xy = np.array(self.forward_start_pos[:2])
            curr_xy = np.array(current_pos[:2])
            distance_traveled = np.linalg.norm(curr_xy - start_xy)

            self.log(
                f"[MODE 2] Distance traveled: {distance_traveled:.2f} m, "
                f"target: {self.min_travel_distance:.2f} m"
            )

            if distance_traveled >= self.min_travel_distance:
                self.mode = 3
                self.done = True
                self.log("DONE! Clearing complete (min_travel_distance reached)")
            else:
                # Keep moving forward (your frame: (0, +0.5, 0) moves forward)
                self.log("Mode 2: Moving forward (0, 0.5, 0)")
                self.uav.publish_position_setpoint((0, 0.5, 0), relative=True)
            return

        # ---------- MODE 0: centering on hoop ----------
        # Transform from camera frame to UAV/local frame using your direction mapping
        self.log(f"Pixel coords: x={response.x}, y={response.y}")
        self.log(f"Raw response.direction: {response.direction}")

        # Your confirmed mapping:
        # direction[0] = left/right, direction[1] = forward/back, direction[2] = up/down
        direction = [response.direction[0],  # X: left/right
                     response.direction[2],  # Y: forward/back
                     response.direction[1]]  # Z: up/down
        self.log(f"After frame transform (LR, FB, UD): {direction}")

        offsets = tuple(x / request.altitude for x in self.offsets) if request.altitude > 1 else self.offsets
        camera_offsets = tuple(x / request.altitude for x in self.camera_offsets) if request.altitude > 1 else self.camera_offsets
        direction = [x + y + z for x, y, z in zip(direction, offsets, self.uav.uav_to_local(camera_offsets))]

        # Check if centered on hoop (left/right and up/down only)
        threshold = 0.05
        if (np.abs(direction[0]) < threshold and
                np.abs(direction[2]) < threshold):

            # Centered! Start moving forward through the hoop.
            self.log("CENTERED! Publishing forward command (0, 0.5, 0) and switching to MODE 1")
            self.uav.publish_position_setpoint((0, 0.5, 0), relative=True)

            # Next update, MODE 1 will compute forward distance from PnP and latch min_travel_distance
            self.mode = 1
            return

        # Not centered yet - adjust position without moving forward
        # direction: [LR, FB, UD]
        direction[0] = -direction[0] / 2.0  # Left/Right correction
        direction[1] = 0.0                  # No forward/back in centering
        direction[2] = direction[2] / 2.0   # Up/Down correction

        self.log(f"Centering - Publishing correction: {direction}")
        self.uav.publish_position_setpoint(direction, relative=True)

    def check_status(self) -> str:
        """
        Check the status of the hoop traversal.
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
