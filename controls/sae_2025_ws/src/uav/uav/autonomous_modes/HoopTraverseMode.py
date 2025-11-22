#!/usr/bin/env python3
# hoop_traverse_mode.py

import numpy as np
from rclpy.node import Node

from uav import UAV
from uav.autonomous_modes import Mode
from uav_interfaces.srv import HoopCenter
from uav.vision_nodes import HoopCenterNode
from px4_msgs.msg import VehicleStatus


class HoopTraverseMode(Mode):
    """
    Mode to center on a hoop using vision and then fly straight through it.

    Logic:
    - Query HoopCenter service to get direction to hoop center in camera frame.
    - Convert that to local frame (similar mapping as PayloadDropoffMode).
    - First phase: CENTERING
        Move laterally until the hoop is approximately centered in the image.
    - Second phase: TRAVERSE
        Once centered (for some time), fly forward a fixed distance through the hoop.
    """

    def __init__(self, node: Node, uav: UAV,
                 traverse_distance: float = 2.0,
                 center_time_required: float = 1.0):
        """
        Args:
            node: ROS 2 node managing the UAV
            uav:  UAV instance
            traverse_distance: how far forward (m) to fly once centered
            center_time_required: how long (s) we must remain 'centered'
        """
        super().__init__(node, uav)

        self.done = False
        self.state = "center"   # "center" -> "traverse" -> "done"

        self.center_time_required = center_time_required
        self.centered_time = 0.0

        self.traverse_distance = traverse_distance
        self.traverse_start_pos = None

        # How tight we want to be aligned, scaled w/ altitude similar to PayloadDropoff
        self.center_threshold_factor = 25.0

    def on_update(self, time_delta: float) -> None:
        # Skip if the UAV is too tilted (same idea as PayloadDropoff)
        if abs(self.uav.roll) > 0.1 or abs(self.uav.pitch) > 0.1:
            self.log("[HoopTraverse] Waiting for UAV to stabilize (roll/pitch too large).")
            return

        # If already done, do nothing
        if self.state == "done":
            self.done = True
            return

        # Request from HoopCenter service
        request = HoopCenter.Request()

        local_pos = self.uav.get_local_position()
        altitude = -local_pos[2]   # same convention as PayloadDropoff
        request.altitude = float(altitude)
        request.yaw = float(self.uav.yaw)

        response = self.send_request(HoopCenterNode, request)

        if response is None:
            self.log("[HoopTraverse] No response from HoopCenterNode.")
            return

        # Extract response
        cam_dir = np.array(response.direction, dtype=float)
        dlz_empty = bool(response.dlz_empty)

        # Map from camera frame to local frame (same mapping as PayloadDropoffMode)
        # direction_cam = [dx_cam, dy_cam, dz_cam]
        # direction_local = [-dy_cam, dx_cam, dz_cam]
        direction_local = np.array(
            [-cam_dir[1], cam_dir[0], cam_dir[2]],
            dtype=float
        )

        # For hoop traversal we typically want to adjust x/y and keep altitude same
        # so we ignore vertical movement here:
        direction_local[2] = 0.0

        # If no good hoop (dlz_empty == True), just hover (or you could do a search pattern)
        if dlz_empty:
            self.log("[HoopTraverse] Hoop not confidently detected, holding position.")
            return

        if self.state == "center":
            self._center_on_hoop(direction_local, altitude, time_delta)
        elif self.state == "traverse":
            self._traverse_through_hoop(time_delta)

    # ---------------- Phases ----------------

    def _center_on_hoop(self, direction_local: np.ndarray,
                        altitude: float,
                        dt: float) -> None:
        """
        Phase 1: laterally center the UAV with respect to the hoop.
        """
        # Scale thresholds by altitude (similar to PayloadDropoff)
        if altitude > 1.0:
            threshold = altitude / self.center_threshold_factor
        else:
            threshold = 0.05  # some small absolute threshold close to hoop

        dx = float(direction_local[0])
        dy = float(direction_local[1])

        if abs(dx) < threshold and abs(dy) < threshold:
            # We are roughly centered
            self.centered_time += dt
            self.log(
                f"[HoopTraverse] Centered for {self.centered_time:.2f}s "
                f"(threshold={threshold:.3f}, dx={dx:.3f}, dy={dy:.3f})"
            )

            if self.centered_time >= self.center_time_required:
                # Switch to traverse phase
                self.state = "traverse"
                self.traverse_start_pos = self.uav.get_local_position()
                self.log("[HoopTraverse] Hoop centered, starting traverse phase.")
        else:
            # Not centered -> reset timer and move
            self.centered_time = 0.0

            # Optionally scale by altitude so we don't overshoot when far away
            move_vec = direction_local.copy()
            if altitude > 1.0:
                move_vec[:2] /= altitude  # smaller steps when far

            self.log(
                f"[HoopTraverse] Centering on hoop: move_vec={move_vec}, "
                f"altitude={altitude:.2f}"
            )
            # Relative setpoint
            self.uav.publish_position_setpoint(
                (float(move_vec[0]), float(move_vec[1]), 0.0),
                relative=True
            )

    def _traverse_through_hoop(self, dt: float) -> None:
        """
        Phase 2: fly forward a fixed distance in the current local X direction.
        """
        if self.traverse_start_pos is None:
            # Shouldn't happen, but be defensive
            self.traverse_start_pos = self.uav.get_local_position()

        current_pos = self.uav.get_local_position()
        dx_travelled = current_pos[0] - self.traverse_start_pos[0]

        if dx_travelled >= self.traverse_distance:
            self.log(
                f"[HoopTraverse] Completed traverse: "
                f"dx_travelled={dx_travelled:.2f} m (target={self.traverse_distance:.2f} m)."
            )
            self.state = "done"
            self.done = True
            return

        # Keep pushing forward with a gentle relative setpoint
        forward_step = min(0.5, self.traverse_distance - dx_travelled)
        self.log(
            f"[HoopTraverse] Traversing hoop: forward_step={forward_step:.2f} m, "
            f"dx_travelled={dx_travelled:.2f} / {self.traverse_distance:.2f}"
        )
        self.uav.publish_position_setpoint(
            (float(forward_step), 0.0, 0.0),
            relative=True
        )

    # ---------------- Status ----------------

    def check_status(self) -> str:
        if self.done:
            return "complete"
        return "continue"
