from typing import Optional

import numpy as np
import cv2
import math
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from payload_interfaces.msg import DriveCommand
from uav.autonomous_modes import Mode
from uav import UAV
from cv_bridge import CvBridge

try:
    import apriltag
except ImportError:
    apriltag = None

DEFAULT_TAG_FAMILY = "tag36h11"

# Standard VTOL AprilTag layout in simulation (from `sim/world_gen/models/standard_vtol/model.sdf`).
# IDs (DICT_APRILTAG_36h11): front=0, back=1, left=2, right=3.
# Poses are relative_to="base_link" in SDF: (x, y, z, roll, pitch, yaw).
_VTOL_TAG_POSES = {
    0: (-0.023, 0.0, 0.0, -1.5707, 3.141592, 1.5707),  # front
    1: (0.03, 0.0, 0.0, 1.5707, 0.0, 1.5707),          # back
    2: (0.0, 0.545, -0.037, 1.5707, 0.0, 3.141592),    # left
    3: (0.0, -0.545, -0.037, 1.5707, 0.0, 0.0),        # right
}


def _wrap_angle(rad: float) -> float:
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def _rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T


def _invert_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def _yaw_from_R_v_c(R_v_c: np.ndarray) -> float:
    f = R_v_c @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return math.atan2(float(f[1]), float(f[0]))


# Object points for solvePnP, ordered to match the apriltag library's actual
# corner output in image coords: corners[0]=TL, corners[1]=TR, corners[2]=BR, corners[3]=BL.
# Tag frame: X-right, Y-up, so TL=(-half,+half), TR=(+half,+half), etc.
def _object_points_for_tag_size(tag_size_m: float) -> np.ndarray:
    half = tag_size_m / 2.0
    return np.array(
        [
            [-half, half, 0],
            [half, half, 0],
            [half, -half, 0],
            [-half, -half, 0],
        ],
        dtype=np.float32,
    )


class PayloadDriveToAprilTagMode(Mode):
    """
    Drives the payload toward a VTOL using AprilTag detection.

    When dock_rear_orthogonal is True, uses a two-layer architecture:
      Decision layer (rare): search -> orbit -> dock | recovery
      Control layer (continuous): small steering corrections only

    Orbit direction is computed once (shortest path to rear) and locked.
    Docking requires the back tag to be within 30° of the camera forward axis
    for a sustained period (hysteresis) before entering dock phase.
    Visual servo docking uses raw camera measurements, not global pose.
    """

    def __init__(
        self,
        node: Node,
        uav: UAV,
        payload_name: str = "payload_0",
        tag_id: Optional[int] = None,
        stop_distance_m: float = 0.01,
        tag_size_m: float = 0.1,
        tag_family: str = DEFAULT_TAG_FAMILY,
        linear_gain: float = 0.3,
        angular_gain: float = 0.002,
        dock_rear_orthogonal: bool = False,
        dock_target_tag_id: int = 1,
        dock_orbit_radius_m: float = 1.2,
        dock_orbit_speed_mps: float = 0.15,
        dock_cam_to_rear_m: float = 0.103,
        dock_recovery_timeout_s: float = 5.0,
        dock_heading_rate_limit: float = 0.15,
        dock_search_spin_rps: float = 0.4,
        dock_align_angle_deg: float = 30.0,
        dock_align_hold_s: float = 0.4,
        dock_recovery_flip_window_s: float = 15.0,
        dock_recovery_flip_count: int = 3,
        start_phase: Optional[str] = None,
    ):
        super().__init__(node, uav)
        self.start_phase = start_phase
        self.payload_name = str(payload_name)
        self.tag_id = tag_id
        self.stop_distance_m = float(stop_distance_m)
        self.tag_size_m = float(tag_size_m)
        self.tag_family = str(tag_family) if tag_family else DEFAULT_TAG_FAMILY
        self.linear_gain = float(linear_gain)
        self.angular_gain = float(angular_gain)
        self.dock_rear_orthogonal = bool(dock_rear_orthogonal)
        self.dock_target_tag_id = int(dock_target_tag_id)
        self.dock_orbit_radius_m = float(dock_orbit_radius_m)
        self.dock_orbit_speed_mps = float(dock_orbit_speed_mps)
        self.dock_cam_to_rear_m = float(dock_cam_to_rear_m)
        self.dock_recovery_timeout_s = float(dock_recovery_timeout_s)
        self.dock_heading_rate_limit = float(dock_heading_rate_limit)
        self.dock_search_spin_rps = float(dock_search_spin_rps)
        self.dock_align_angle_deg = float(dock_align_angle_deg)
        self.dock_align_hold_s = float(dock_align_hold_s)
        self.dock_recovery_flip_window_s = float(dock_recovery_flip_window_s)
        self.dock_recovery_flip_count = int(dock_recovery_flip_count)
        self.done = False
        self._image = None
        self._camera_info = None
        self._bridge = CvBridge()
        self._detector = None
        if apriltag is not None:
            try:
                options = apriltag.DetectorOptions(
                    families=self.tag_family,
                    refine_edges=True,
                )
                self._detector = apriltag.Detector(options)
            except (AttributeError, TypeError):
                self._detector = apriltag.Detector()

    def on_enter(self) -> None:
        self._image = None
        self._camera_info = None
        self.done = False

        if getattr(self, "start_phase", None) == "dock":
            self._phase = "dock"
            self._orbit_dir = 1
            self._orbit_locked = True
            self._pose_vtol = None
            self._vtol_center = None
            self._desired_yaw = None
            self.log("PayloadDriveToAprilTagMode: started in dock phase (handoff from color orbit)")
        else:
            self._phase = "search"       # search | orbit | dock | recovery
            self._orbit_dir = None       # +1 CCW, -1 CW; computed once, locked
            self._orbit_locked = False
            self._pose_vtol = None       # (x, y, yaw) camera in VTOL base_link
            self._vtol_center = None     # (cx, cy) slow-filtered VTOL center in payload frame
            self._desired_yaw = None     # persistent trajectory heading
        self._last_cmd = (0.0, 0.0)
        self._last_tag_time = None
        self._last_log_time = 0.0
        self._last_closest_id = None

        # 360° search scan state
        self._search_start_yaw = None
        self._search_total_rot = 0.0
        self._search_poses = []
        self._search_started = False

        # Dock alignment hysteresis: back tag must stay within angle for hold time
        self._dock_align_start = None  # timestamp when alignment first met

        # Recovery direction-flip tracking
        self._recovery_timestamps = []  # timestamps of recovery entries

        cam_topic = f"/{self.payload_name}/camera"
        info_topic = f"/{self.payload_name}/camera_info"
        self.node.get_logger().info(
            f"PayloadDriveToAprilTagMode: subscribing to {cam_topic}, {info_topic}"
        )
        self._image_sub = self.node.create_subscription(
            Image, cam_topic, self._image_cb, 10,
        )
        self._info_sub = self.node.create_subscription(
            CameraInfo, info_topic, self._info_cb, 10,
        )
        drive_topic = f"/{self.payload_name}/cmd_drive"
        self._drive_pub = self.node.create_publisher(
            DriveCommand, drive_topic, 10,
        )
        self._first_image_logged = False
        self._last_wait_log_time = 0.0
        self._last_no_tag_log_time = 0.0
        self._last_drive_log_time = 0.0
        self.node.get_logger().info(
            f"PayloadDriveToAprilTagMode: publishing drive commands to {drive_topic}"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _estimate_camera_pose_in_vtol(
        self, tag_id: int, rvec: np.ndarray, tvec: np.ndarray,
    ):
        if tag_id not in _VTOL_TAG_POSES:
            return None
        tx, ty, tz, rr, rp, ry = _VTOL_TAG_POSES[int(tag_id)]
        R_v_t = _rpy_to_rot(rr, rp, ry)
        T_v_t = _make_T(R_v_t, np.array([tx, ty, tz], dtype=np.float64))
        R_c_t, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
        T_c_t = _make_T(R_c_t, np.asarray(tvec, dtype=np.float64).reshape(3, 1))
        T_v_c = T_v_t @ _invert_T(T_c_t)
        p_v = T_v_c[:3, 3]
        yaw_v = _yaw_from_R_v_c(T_v_c[:3, :3])
        return (float(p_v[0]), float(p_v[1]), float(yaw_v))

    def _publish_drive(self, linear: float, angular: float) -> None:
        self._last_cmd = (float(linear), float(angular))
        self._drive_pub.publish(
            DriveCommand(linear=float(linear), angular=float(angular))
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _back_tag_view_angle_deg(self, tag_results) -> Optional[float]:
        """Angle between camera forward (0,0,1) and direction to back tag center."""
        back_id = self.dock_target_tag_id
        if back_id not in tag_results:
            return None
        tvec = np.asarray(tag_results[back_id][2]).ravel()
        norm = float(np.linalg.norm(tvec))
        if norm < 1e-6:
            return None
        cos_angle = float(tvec[2] / norm)
        return math.degrees(math.acos(max(-1.0, min(1.0, cos_angle))))

    # ------------------------------------------------------------------
    # Core docking logic (two-layer architecture)
    # ------------------------------------------------------------------

    def _dock_update(self, time_delta: float, detections, K, dist_coeffs) -> None:
        import time as _time
        now = _time.time()

        # ============================================================
        # 1. Process all detected tags
        # ============================================================
        obj_pts = _object_points_for_tag_size(self.tag_size_m)
        tag_results = {}
        seen_ids = []

        for d in detections or []:
            tag_id = int(getattr(d, "tag_id", -1))
            if tag_id not in _VTOL_TAG_POSES:
                continue
            seen_ids.append(tag_id)
            corners = np.array(d.corners, dtype=np.float32)
            if len(corners) != 4:
                continue
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, corners, K, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue
            if float(np.asarray(tvec).ravel()[2]) <= 0.05:
                continue
            pose = self._estimate_camera_pose_in_vtol(tag_id, rvec, tvec)
            if pose is None:
                continue
            tag_results[tag_id] = (pose, rvec, tvec, corners, d)

        back_visible = self.dock_target_tag_id in tag_results
        any_visible = len(tag_results) > 0

        all_det_ids = [int(getattr(d, "tag_id", -1)) for d in (detections or [])]
        if now - getattr(self, "_last_det_log_time", 0) >= 0.5:
            self._last_det_log_time = now
            tag_info = {
                tid: f"d={float(np.asarray(tr[2]).ravel()[2]):.3f}m"
                for tid, tr in tag_results.items()
            }
            self.log(
                f"DOCK | detections raw={all_det_ids} solved={list(tag_results.keys())} "
                f"info={tag_info} phase={self._phase}"
            )

        # ============================================================
        # 2. Update VTOL-frame pose estimate
        # ============================================================
        if any_visible:
            closest_id = min(
                tag_results,
                key=lambda tid: float(np.asarray(tag_results[tid][2]).ravel()[2]),
            )
            meas = tag_results[closest_id][0]
            if self._pose_vtol is None or closest_id != self._last_closest_id:
                self._pose_vtol = meas
                if self._last_closest_id is None:
                    self.log(
                        f"DOCK | first pose tag={closest_id} "
                        f"x={meas[0]:.3f} y={meas[1]:.3f} yaw={math.degrees(meas[2]):.1f}°"
                    )
            else:
                a = 0.35
                x0, y0, yaw0 = self._pose_vtol
                x1, y1, yaw1 = meas
                self._pose_vtol = (
                    (1 - a) * x0 + a * x1,
                    (1 - a) * y0 + a * y1,
                    _wrap_angle(yaw0 + a * _wrap_angle(yaw1 - yaw0)),
                )
            self._last_closest_id = closest_id
            self._last_tag_time = now

            # Slow-filtered VTOL center estimate (payload frame).
            # _pose_vtol = (x, y, yaw) is payload position in VTOL frame,
            # so the VTOL center in payload frame is (-x, -y).
            mx, my = -meas[0], -meas[1]
            if self._vtol_center is None:
                self._vtol_center = (mx, my)
            else:
                alpha = 0.02
                cx, cy = self._vtol_center
                self._vtol_center = (
                    cx + alpha * (mx - cx),
                    cy + alpha * (my - cy),
                )

        # ============================================================
        # 3. DECISION LAYER  (rare state transitions)
        # ============================================================

        # --- search: 360° spin then choose orbit direction ---
        if self._phase == "search":
            spin_w = self.dock_search_spin_rps * 2.0 * math.pi

            if not self._search_started:
                if self._pose_vtol is not None:
                    self._search_started = True
                    self._search_start_yaw = self._pose_vtol[2]
                    self._search_total_rot = 0.0
                    self._search_poses = []
                    self.log("DOCK | search: first tag seen, starting 360° scan")
                else:
                    self._publish_drive(0.0, spin_w)
                    return

            self._search_total_rot += abs(spin_w) * time_delta
            for tid, (pose, *_rest) in tag_results.items():
                self._search_poses.append((tid, pose))

            if self._search_total_rot < 2.0 * math.pi:
                if now - self._last_log_time >= 2.0:
                    self._last_log_time = now
                    self.log(
                        f"DOCK | search: scanning {math.degrees(self._search_total_rot):.0f}°/360° "
                        f"tags_collected={len(self._search_poses)}"
                    )
                self._publish_drive(0.0, spin_w)
                return

            x, y, yaw = self._pose_vtol
            theta = math.atan2(y, x)
            # Rear of VTOL is at angle π in the VTOL frame.
            # Choose orbit direction that is the shortest angular
            # path from the payload's current position to the rear.
            angle_to_rear = _wrap_angle(math.pi - theta)
            self._orbit_dir = 1 if angle_to_rear >= 0 else -1
            self._orbit_locked = True
            self._vtol_center = (-x, -y)
            self._desired_yaw = _wrap_angle(math.atan2(-y, -x))
            self._phase = "orbit"
            self.log(
                f"DOCK | search -> orbit (360° done, {len(self._search_poses)} sightings) "
                f"dir={'CCW' if self._orbit_dir == 1 else 'CW'} "
                f"pos=({x:.3f},{y:.3f}) θ={math.degrees(theta):.1f}° "
                f"angle_to_rear={math.degrees(angle_to_rear):.1f}° "
                f"face_vtol_yaw={math.degrees(self._desired_yaw):.1f}°"
            )

        # --- orbit -> dock: back tag visible + aligned + held ---
        if self._phase == "orbit" and back_visible:
            view_angle = self._back_tag_view_angle_deg(tag_results)
            aligned = view_angle is not None and view_angle <= self.dock_align_angle_deg
            if aligned:
                if self._dock_align_start is None:
                    self._dock_align_start = now
                elif (now - self._dock_align_start) >= self.dock_align_hold_s:
                    self._phase = "dock"
                    self._dock_align_start = None
                    self.log(
                        f"DOCK | orbit -> dock (aligned {view_angle:.1f}° held "
                        f"{self.dock_align_hold_s:.1f}s, tags={seen_ids})"
                    )
            else:
                self._dock_align_start = None
        elif self._phase != "dock":
            self._dock_align_start = None

        # --- orbit/dock -> recovery: tags lost for sustained duration ---
        # During brief tag loss, continue last command (dead-reckon)
        if self._phase in ("orbit", "dock"):
            if not any_visible and self._last_tag_time is not None:
                gap = now - self._last_tag_time
                if gap >= self.dock_recovery_timeout_s:
                    prev_phase = self._phase
                    self._phase = "recovery"
                    self._dock_align_start = None
                    self._recovery_timestamps.append(now)
                    self.log(
                        f"DOCK | {prev_phase} -> recovery (tags lost {gap:.1f}s)"
                    )
                    self._maybe_flip_direction(now)

        # --- dock: back tag lost but other tags visible -> orbit ---
        if self._phase == "dock" and not back_visible and any_visible:
            self._phase = "orbit"
            self._dock_align_start = None
            self.log(
                f"DOCK | dock -> orbit (back tag lost, others visible: {seen_ids})"
            )

        # --- recovery -> orbit: any tag reacquired ---
        if self._phase == "recovery" and any_visible:
            self._phase = "orbit"
            self._desired_yaw = None
            self.log(f"DOCK | recovery -> orbit (tags reacquired: {seen_ids})")

        # ============================================================
        # 4. CONTROL LAYER  (continuous corrections)
        # ============================================================

        if self._phase == "orbit":
            self._control_orbit(now, time_delta, seen_ids, any_visible)
        elif self._phase == "dock":
            self._control_dock(now, tag_results, seen_ids)
        elif self._phase == "recovery":
            if self._vtol_center is not None and self._pose_vtol is not None:
                cx, cy = self._vtol_center
                _x, _y, yaw = self._pose_vtol
                bearing_to_vtol = math.atan2(cy, cx)
                yaw_err = _wrap_angle(bearing_to_vtol - yaw)
                w = float(np.clip(2.0 * yaw_err, -0.6, 0.6))
            else:
                w = 0.5 * (self._orbit_dir if self._orbit_dir else 1)
            self._publish_drive(0.0, w)
            if now - self._last_log_time >= 2.0:
                self._last_log_time = now
                self.log(
                    f"DOCK | recovery: turning toward VTOL w={w:.2f}"
                )

    def _maybe_flip_direction(self, now: float) -> None:
        """Flip orbit direction if recovery keeps happening (path is blocked)."""
        window = self.dock_recovery_flip_window_s
        self._recovery_timestamps = [
            t for t in self._recovery_timestamps if (now - t) < window
        ]
        if len(self._recovery_timestamps) >= self.dock_recovery_flip_count:
            old = self._orbit_dir
            self._orbit_dir = -self._orbit_dir if self._orbit_dir else 1
            self._recovery_timestamps.clear()
            self.log(
                f"DOCK | direction flip: "
                f"{'CCW' if old == 1 else 'CW'} -> "
                f"{'CCW' if self._orbit_dir == 1 else 'CW'} "
                f"(repeated recovery)"
            )

    # ------------------------------------------------------------------
    # Control: smooth orbit with persistent heading
    # ------------------------------------------------------------------

    def _control_orbit(self, now, time_delta, seen_ids, any_visible) -> None:
        if self._vtol_center is None or self._pose_vtol is None:
            self._publish_drive(0.0, 0.0)
            return

        # Orbit geometry from stable center estimate, yaw from pose
        cx, cy = self._vtol_center
        _x, _y, yaw = self._pose_vtol
        r = math.hypot(cx, cy)
        theta = math.atan2(cy, cx)

        ideal_yaw = _wrap_angle(
            theta + self._orbit_dir * (math.pi / 2.0)
        )

        radial_err = self.dock_orbit_radius_m - r
        ideal_yaw = _wrap_angle(ideal_yaw + 0.5 * radial_err)

        if not any_visible:
            yaw_err = _wrap_angle(ideal_yaw - yaw)
            w = float(np.clip(2.0 * yaw_err, -0.6, 0.6))
            v = self.dock_orbit_speed_mps
            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self.log(
                    f"DOCK | orbit  coast r={r:.3f} θ={math.degrees(theta):.1f}° "
                    f"dir={'CCW' if self._orbit_dir == 1 else 'CW'} "
                    f"v={v:.3f} w={w:.3f} tags=[] src=coast"
                )
            self._publish_drive(v, w)
            return

        if self._desired_yaw is None:
            self._desired_yaw = ideal_yaw
        else:
            yaw_diff = _wrap_angle(ideal_yaw - self._desired_yaw)
            limit = self.dock_heading_rate_limit * time_delta * 20.0
            clamped = max(-limit, min(limit, yaw_diff))
            self._desired_yaw = _wrap_angle(self._desired_yaw + clamped)

        yaw_err = _wrap_angle(self._desired_yaw - yaw)
        w = float(np.clip(2.0 * yaw_err, -0.6, 0.6))

        radius_error_ratio = abs(radial_err) / max(self.dock_orbit_radius_m, 1e-3)
        v = self.dock_orbit_speed_mps * (1.0 - 0.5 * min(1.0, radius_error_ratio))
        v = float(np.clip(v, 0.05, 0.2))

        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self.log(
                f"DOCK | orbit  center=({cx:.3f},{cy:.3f}) "
                f"r={r:.3f} θ={math.degrees(theta):.1f}° "
                f"dir={'CCW' if self._orbit_dir == 1 else 'CW'} "
                f"v={v:.3f} w={w:.3f} tags={seen_ids}"
            )

        self._publish_drive(v, w)

    # ------------------------------------------------------------------
    # Control: visual servo docking using raw camera measurements
    # ------------------------------------------------------------------

    def _control_dock(self, now, tag_results, seen_ids) -> None:
        back_id = self.dock_target_tag_id
        if back_id not in tag_results:
            v, w = self._last_cmd
            self._publish_drive(v, w)
            return

        _pose, _rvec, tvec, _corners, detection = tag_results[back_id]
        tvec_flat = np.asarray(tvec).ravel()
        tx, tz = float(tvec_flat[0]), float(tvec_flat[2])
        distance = tz

        if distance <= self.stop_distance_m:
            self._publish_drive(0.0, 0.0)
            self.done = True
            self.log(f"DOCK | complete! dist={distance:.3f}m tags={seen_ids}")
            return

        # Pixel centering: primary steering signal
        img_w = float(self._camera_info.width) if self._camera_info else 640.0
        cx = img_w / 2.0
        tag_cx = float(detection.center[0])
        err_x = tag_cx - cx

        # Gentle lateral correction from 3D offset for perpendicular approach.
        # Bearing angle to tag: positive = tag to the right.
        bearing = math.atan2(tx, tz)

        angular = float(np.clip(
            -self.angular_gain * err_x - 0.5 * bearing,
            -0.4, 0.4
        ))

        linear = float(
            np.clip(
                self.linear_gain * (distance - self.stop_distance_m),
                0.05,
                0.2,
            )
        )

        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self.log(
                f"DOCK | dock   dist={distance:.3f}m tx={tx:.3f}m "
                f"bearing={math.degrees(bearing):.1f}° err_x={err_x:.1f}px "
                f"v={linear:.3f} w={angular:.3f} tags={seen_ids}"
            )
            # #region agent log
            import json as _json
            _log_data = {"location":"PayloadDriveToAprilTagMode.py:dock_ctrl","message":"dock_ctrl","data":{"distance":round(distance,3),"tx":round(tx,3),"bearing_deg":round(math.degrees(bearing),1),"err_x":round(err_x,1),"angular":round(angular,3),"linear":round(linear,3),"tags":seen_ids},"timestamp":int(now*1000),"hypothesisId":"D"}
            with open("/home/ubuntu/monorepo/.cursor/debug.log","a") as _f: _f.write(_json.dumps(_log_data)+"\n")
            # #endregion

        self._publish_drive(linear, angular)

    # ------------------------------------------------------------------
    # Callbacks and main loop (unchanged)
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image) -> None:
        self._image = msg

    def _info_cb(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def on_exit(self) -> None:
        if hasattr(self, "_drive_pub"):
            self._drive_pub.publish(DriveCommand(linear=0.0, angular=0.0))

    def on_update(self, time_delta: float) -> None:
        import time as _time
        now = _time.time()
        if self.done:
            return
        if self._detector is None:
            self.log("PayloadDriveToAprilTagMode: apriltag not installed; cannot detect tags.")
            return
        if self._image is None or self._camera_info is None:
            if now - getattr(self, "_last_wait_log_time", 0) >= 2.0:
                self._last_wait_log_time = now
                self.log(
                    f"PayloadDriveToAprilTagMode: waiting for payload camera "
                    f"(image={self._image is not None}, camera_info={self._camera_info is not None})"
                )
            return
        if not getattr(self, "_first_image_logged", False):
            self._first_image_logged = True
            self.log("PayloadDriveToAprilTagMode: first image received from payload camera")
        try:
            gray = self._bridge.imgmsg_to_cv2(self._image, desired_encoding="mono8")
        except Exception as e:
            self.node.get_logger().warn(f"Could not convert image: {e}")
            return
        K = np.array(self._camera_info.k, dtype=np.float64).reshape(3, 3)
        d = getattr(self._camera_info, "d", None) or []
        dist_coeffs = np.array(d, dtype=np.float64)
        if dist_coeffs.size == 0:
            dist_coeffs = np.zeros(5)
        if dist_coeffs.shape != (5,):
            dist_coeffs = np.resize(np.asarray(dist_coeffs).reshape(-1), 5)

        detections = self._detector.detect(gray)
        if self.dock_rear_orthogonal:
            self._dock_update(time_delta, detections, K, dist_coeffs)
            return

        # --- Original non-docking behavior (drive toward any tag) ---
        target = None
        if detections:
            if self.tag_id is not None:
                for det in detections:
                    if det.tag_id == self.tag_id:
                        target = det
                        break
            else:
                target = detections[0]

        if target is None:
            if now - getattr(self, "_last_no_tag_log_time", 0) >= 2.0:
                self._last_no_tag_log_time = now
                if detections:
                    ids = [det.tag_id for det in detections]
                    self.log(f"PayloadDriveToAprilTagMode: tags in view {ids} but no match, angular=0.2")
                else:
                    self.log("PayloadDriveToAprilTagMode: no tag in view, angular=0.2")
            self._drive_pub.publish(DriveCommand(linear=0.0, angular=0.2))
            return

        obj_pts = _object_points_for_tag_size(self.tag_size_m)
        corners = np.array(target.corners, dtype=np.float32)
        if len(corners) != 4:
            return
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, corners, K, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return
        tvec_flat = np.asarray(tvec).ravel()
        distance = float(tvec_flat[2]) if len(tvec_flat) >= 3 else 0.0
        if distance <= self.stop_distance_m:
            self._drive_pub.publish(DriveCommand(linear=0.0, angular=0.0))
            self.done = True
            self.log(f"PayloadDriveToAprilTagMode: done, stopped at {distance:.3f}m")
            return

        h, w = gray.shape[:2]
        cx_img = w / 2.0
        err_x = target.center[0] - cx_img
        linear = np.clip(self.linear_gain * (distance - self.stop_distance_m), 0.0, 0.5)
        angular = np.clip(-self.angular_gain * err_x, -0.5, 0.5)
        if now - getattr(self, "_last_drive_log_time", 0) >= 1.0:
            self._last_drive_log_time = now
            self.log(
                f"PayloadDriveToAprilTagMode: tag={target.tag_id} dist={distance:.3f}m "
                f"linear={linear:.2f} angular={angular:.2f}"
            )
        self._drive_pub.publish(DriveCommand(linear=linear, angular=angular))

    def check_status(self) -> str:
        if self.done:
            return "terminate"
        return "continue"
