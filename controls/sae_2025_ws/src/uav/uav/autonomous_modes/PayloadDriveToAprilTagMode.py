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

    One initial 360° scan; orbit direction is computed once (shortest path to rear)
    and locked. No full spins after scan; brief tag loss = coast; sustained loss ->
    recovery with controlled re-acquire (no new full scan). Orbit->dock uses
    distance-dependent alignment: stricter when close (e.g. <=0.8m require 10°)
    to avoid false docking. Visual servo docking uses raw camera measurements.
    """

    def __init__(
        self,
        node: Node,
        uav: UAV,
        payload_name: str = "payload_0",
        tag_id: Optional[int] = None,
        stop_distance_m: float = 0.2,
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
        dock_approach_front_dist_m: float = 2.0,
        dock_approach_front_min_m: float = 1.8,
        dock_approach_front_max_m: float = 2.2,
        dock_front_safe_standoff_m: float = 1.0,
        dock_approach_backup_margin_m: float = 0.2,
        dock_approach_bearing_gain: float = 0.3,
        dock_approach_pixel_scale: float = 0.5,
        dock_orbit_straight_duration_s: float = 3,
        dock_orbit_peek_duration_s: float = 10,
        dock_orbit_radial_k: float = 0.8,
        dock_orbit_yaw_k: float = 2.0,
        dock_orbit_peek_blend: float = 0.4,
        dock_orbit_peek_back_cooldown_s: float = 3.0,
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
        self.dock_approach_front_dist_m = float(dock_approach_front_dist_m)
        self.dock_approach_front_min_m = float(dock_approach_front_min_m)
        self.dock_approach_front_max_m = float(dock_approach_front_max_m)
        self.dock_front_safe_standoff_m = float(dock_front_safe_standoff_m)
        self.dock_approach_backup_margin_m = float(dock_approach_backup_margin_m)
        self.dock_approach_bearing_gain = float(dock_approach_bearing_gain)
        self.dock_approach_pixel_scale = float(dock_approach_pixel_scale)
        self.dock_orbit_straight_duration_s = float(dock_orbit_straight_duration_s)
        self.dock_orbit_peek_duration_s = float(dock_orbit_peek_duration_s)
        self.dock_orbit_radial_k = float(dock_orbit_radial_k)
        self.dock_orbit_yaw_k = float(dock_orbit_yaw_k)
        self.dock_orbit_peek_blend = float(dock_orbit_peek_blend)
        self.dock_orbit_peek_back_cooldown_s = float(dock_orbit_peek_back_cooldown_s)
        self.done = False
        # One-time log to verify mission YAML params are passed
        self.node.get_logger().info(
            f"PayloadDriveToAprilTagMode: mission params check "
            f"dock_orbit_straight_duration_s={self.dock_orbit_straight_duration_s} "
            f"dock_orbit_radius_m={self.dock_orbit_radius_m} dock_orbit_yaw_k={self.dock_orbit_yaw_k} "
            f"dock_approach_front_dist_m={self.dock_approach_front_dist_m}"
        )
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
            self._phase = "search"       # search | approach_front | orbit | dock | recovery
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

        # Segment-based orbit: straight -> peek_turn -> turn_back -> straight
        self._orbit_segment = None       # 'straight' | 'peek_turn' | 'turn_back'
        self._orbit_straight_start_time = None
        self._orbit_straight_heading = None   # VTOL-frame yaw to hold during straight
        self._orbit_peek_start_time = None
        self._orbit_turn_back_start_time = None

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

    def _compute_straight_heading(self, x_vtol: float, y_vtol: float) -> float:
        """Compute desired heading for straight segment (tangent + radial correction). Lock once per segment."""
        r = math.hypot(x_vtol, y_vtol)
        theta = math.atan2(y_vtol, x_vtol)
        heading_tangent = _wrap_angle(theta + self._orbit_dir * (math.pi / 2.0))
        radial_err = r - self.dock_orbit_radius_m
        radial_k_eff = self.dock_orbit_radial_k * (2.0 if radial_err < 0 else 1.0)
        corr = math.atan(radial_k_eff * radial_err)
        return _wrap_angle(heading_tangent + self._orbit_dir * corr)

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
            seen_ids.append(tag_id)

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
            # In orbit/recovery, prefer side (2,3) then back (1), use front (0) only if nothing else,
            # so front tag does not dominate and pull the arc inward (VTOL ~1.5 m long).
            if self._phase in ("orbit", "recovery"):
                side = [tid for tid in [2, 3] if tid in tag_results]
                back = [1] if 1 in tag_results else []
                front = [0] if 0 in tag_results else []
                candidates = side if side else (back if back else (front if front else list(tag_results.keys())))
                closest_id = min(
                    candidates,
                    key=lambda tid: float(np.asarray(tag_results[tid][2]).ravel()[2]),
                )
            else:
                closest_id = min(
                    tag_results,
                    key=lambda tid: float(np.asarray(tag_results[tid][2]).ravel()[2]),
                )
            meas = tag_results[closest_id][0]
            # #region agent log
            if self._phase == "orbit" and (now - getattr(self, "_pose_dbg_time", 0) >= 0.5):
                self._pose_dbg_time = now
                try:
                    import json as _json
                    _mx, _my = meas[0], meas[1]
                    _r = math.hypot(_mx, _my)
                    _pay = {"location": "PayloadDriveToAprilTagMode.py:pose_update", "message": "orbit_pose", "data": {"closest_id": closest_id, "x": _mx, "y": _my, "r_from_meas": _r, "seen_ids": list(tag_results.keys())}, "timestamp": int(now * 1000), "hypothesisId": "R2"}
                    for _path in ("/home/ubuntu/monorepo/.cursor/debug.log", "/tmp/payload_dock_debug.log"):
                        try:
                            with open(_path, "a") as _f:
                                _f.write(_json.dumps(_pay) + "\n")
                        except Exception:
                            pass
                except Exception:
                    pass
            # #endregion
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

        # --- search: always 360° scan first; during scan (or in orbit) if back tag within 30° -> dock ---
        if self._phase == "search":
            # If only front tag (0) visible and far, approach to 1.5 m first instead of 360° scan
            front_id = 0
            only_front = (set(seen_ids) == {front_id}) if seen_ids else False
            front_dist = None
            if front_id in tag_results:
                front_tvec = np.asarray(tag_results[front_id][2]).ravel()
                front_dist = float(front_tvec[2])

            if not self._search_started and self._pose_vtol is not None and only_front and front_dist is not None:
                # Enter when too far (drive forward) or too close (back up until 1.8–2.2 m band).
                too_far = front_dist > self.dock_approach_front_max_m
                too_close = front_dist < self.dock_approach_front_min_m
                if too_far:
                    self._phase = "approach_front"
                    self.log(
                        f"DOCK | search -> approach_front (only front tag, d={front_dist:.2f}m > {self.dock_approach_front_max_m}m)"
                    )
                    # #region agent log
                    try:
                        import json as _json
                        with open("/tmp/payload_dock_debug.log", "a") as _f:
                            _f.write(_json.dumps({"location": "PayloadDriveToAprilTagMode.py:search_to_approach_front", "message": "enter_approach_front", "data": {"front_dist": front_dist, "threshold": self.dock_approach_front_dist_m}, "timestamp": int(now * 1000), "hypothesisId": "AF"}) + "\n")
                    except Exception:
                        pass
                    # #endregion
                    self._control_approach_front(now, tag_results, seen_ids)
                    return
                if too_close:
                    self._phase = "approach_front"
                    self.log(
                        f"DOCK | search -> approach_front (only front tag, d={front_dist:.2f}m < {self.dock_approach_front_min_m}m, will back up to 1.8–2.2 m)"
                    )
                    # #region agent log
                    try:
                        import json as _json
                        with open("/tmp/payload_dock_debug.log", "a") as _f:
                            _f.write(_json.dumps({"location": "PayloadDriveToAprilTagMode.py:search_to_approach_front", "message": "enter_approach_front_too_close", "data": {"front_dist": front_dist, "min_m": self.dock_approach_front_min_m}, "timestamp": int(now * 1000), "hypothesisId": "AF"}) + "\n")
                    except Exception:
                        pass
                    # #endregion
                    self._control_approach_front(now, tag_results, seen_ids)
                    return
                # else: already in band [min_m, max_m], fall through to start 360° or orbit

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
                # During 360° scan: if back tag visible and within 30°, start docking (same as normal dock entry)
                view_angle = self._back_tag_view_angle_deg(tag_results)
                max_deg = self.dock_align_angle_deg
                aligned_now = (view_angle is not None) and (view_angle <= max_deg)
                if back_visible and aligned_now:
                    if self._dock_align_start is None:
                        self._dock_align_start = now
                    if (now - self._dock_align_start) >= self.dock_align_hold_s:
                        self._phase = "dock"
                        self._dock_sub = "align"
                        self._dock_align_start = None
                        self.log(
                            f"DOCK | search (360° scan) -> dock (back tag view_angle={view_angle:.1f}° <= {max_deg}° held)"
                        )
                        self._publish_drive(0.0, 0.0)
                        return
                else:
                    self._dock_align_start = None
                if now - self._last_log_time >= 2.0:
                    self._last_log_time = now
                    self.log(
                        f"DOCK | search: scanning {math.degrees(self._search_total_rot):.0f}°/360° "
                        f"tags_collected={len(self._search_poses)}"
                    )
                self._publish_drive(0.0, spin_w)
                return

            x, y, yaw = self._pose_vtol
            self._orbit_dir = self._choose_orbit_dir(x, y, "search")
            self._orbit_locked = True
            self._vtol_center = (-x, -y)
            theta_vtol = math.atan2(y, x)
            self._desired_yaw = _wrap_angle(math.atan2(-y, -x))
            self._phase = "orbit"
            self._orbit_segment = "straight"
            self._orbit_straight_start_time = now
            self._orbit_straight_heading = self._compute_straight_heading(x, y)
            self._orbit_peek_start_time = None
            self._orbit_turn_back_start_time = None
            self.log(
                f"DOCK | search -> orbit (360° done, {len(self._search_poses)} sightings) "
                f"pos=({x:.3f},{y:.3f}) θ={math.degrees(theta_vtol):.1f}°"
            )

        # --- approach_front: drive toward front tag until within 1.5 m; exit to orbit ---
        if self._phase == "approach_front":
            front_id = 0
            # #region agent log
            _af_branch = None
            _af_front_dist = None
            # #endregion
            if not any_visible or front_id not in tag_results:
                _af_branch = "tags_lost"
                self._control_approach_front(now, tag_results, seen_ids)
                return
            # If we see more than front, go to orbit using current pose (recommended in plan)
            if set(seen_ids) != {front_id}:
                _af_branch = "other_tag"
                self._transition_approach_front_to_orbit(now)
                return
            front_tvec = np.asarray(tag_results[front_id][2]).ravel()
            front_dist = float(front_tvec[2])
            _af_front_dist = front_dist
            # Hard safety: do not drive forward if already inside standoff (VTOL ~1.5 m long).
            if front_dist < self.dock_front_safe_standoff_m:
                _af_branch = "safe_standoff"
                self._transition_approach_front_to_orbit(now)
                return
            # Transition to orbit when in target band (1.8–2.2 m).
            if self.dock_approach_front_min_m <= front_dist <= self.dock_approach_front_max_m:
                _af_branch = "within_dist"
                self._transition_approach_front_to_orbit(now)
                return
            _af_branch = "control"
            self._control_approach_front(now, tag_results, seen_ids)
            # #region agent log
            if now - getattr(self, "_af_log_time", 0) >= 0.5:
                self._af_log_time = now
                try:
                    import json as _json
                    with open("/tmp/payload_dock_debug.log", "a") as _f:
                        _f.write(_json.dumps({"location": "PayloadDriveToAprilTagMode.py:approach_front", "message": "approach_front", "data": {"branch": _af_branch, "front_dist": _af_front_dist, "threshold": self.dock_approach_front_dist_m, "seen_ids": list(seen_ids) if seen_ids else []}, "timestamp": int(now * 1000), "hypothesisId": "AF"}) + "\n")
                except Exception:
                    pass
            # #endregion
            return

        # Dock entry is only from orbit segment peek_back_120 (view angle <=45° held); no instant orbit->dock here.

        # --- orbit/dock/approach_front -> recovery: tags lost for sustained duration ---
        # Skip for dock spin_180/back_up_5s: we intentionally lose the tag during spin and backup
        if self._phase in ("orbit", "dock", "approach_front"):
            dock_finishing = self._phase == "dock" and getattr(self, "_dock_sub", None) in ("spin_180", "back_up_5s")
            if not dock_finishing and not any_visible and self._last_tag_time is not None:
                gap = now - self._last_tag_time
                if gap >= self.dock_recovery_timeout_s:
                    prev_phase = self._phase
                    self._phase = "recovery"
                    self._dock_align_start = None
                    self._recovery_timestamps.append(now)
                    if prev_phase == "approach_front" and self._pose_vtol is not None:
                        x, y, _ = self._pose_vtol
                        self._vtol_center = (-x, -y)
                        self._orbit_dir = self._choose_orbit_dir(x, y, "recovery")
                    self.log(
                        f"DOCK | {prev_phase} -> recovery (tags lost {gap:.1f}s)"
                    )
                    self._maybe_flip_direction(now)

        # --- dock: back tag lost but other tags visible -> orbit (not during spin_180/back_up_5s) ---
        if self._phase == "dock" and getattr(self, "_dock_sub", None) not in ("spin_180", "back_up_5s") and not back_visible and any_visible:
            self._phase = "orbit"
            self._dock_sub = None
            self._dock_align_start = None
            if self._vtol_center is not None and self._pose_vtol is not None:
                x_vtol, y_vtol, yaw = self._pose_vtol
                self._orbit_segment = "straight"
                self._orbit_straight_start_time = now
                self._orbit_straight_heading = self._compute_straight_heading(x_vtol, y_vtol)
                self._orbit_peek_start_time = None
                self._orbit_turn_back_start_time = None
            self.log(
                f"DOCK | dock -> orbit (back tag lost, others visible: {seen_ids})"
            )

        # --- recovery -> orbit: any tag reacquired; return to orbit smoothly (no search/spin) ---
        if self._phase == "recovery" and any_visible:
            self._phase = "orbit"
            # Set desired_yaw and init segment state for segment-based orbit
            if self._vtol_center is not None and self._pose_vtol is not None:
                x_vtol, y_vtol, yaw = self._pose_vtol
                ideal_yaw = self._compute_straight_heading(x_vtol, y_vtol)
                self._desired_yaw = ideal_yaw
                self._orbit_segment = "straight"
                self._orbit_straight_start_time = now
                self._orbit_straight_heading = ideal_yaw
                self._orbit_peek_start_time = None
                self._orbit_turn_back_start_time = None
            else:
                self._desired_yaw = None
            self.log(
                f"DOCK | recovery -> orbit (tags reacquired: {seen_ids}, dir={'CCW' if self._orbit_dir == 1 else 'CW'})"
            )

        # ============================================================
        # 4. CONTROL LAYER  (continuous corrections)
        # ============================================================

        # Orbit: direction is locked (only recovery can flip). No new spins; brief tag loss = coast.
        if self._phase == "orbit":
            self._control_orbit(now, time_delta, seen_ids, any_visible, tag_results)
        elif self._phase == "dock":
            self._control_dock(now, time_delta, tag_results, seen_ids)
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

    def _choose_orbit_dir(self, x: float, y: float, context: str = "") -> int:
        """Choose orbit direction (shortest way to rear). Rear is +x in VTOL frame (back tag)."""
        theta = math.atan2(y, x)
        rear_angle = 0.0
        delta = _wrap_angle(rear_angle - theta)
        direction = 1 if delta > 0 else -1
        self.log(
            f"DOCK | orbit_dir x={x:.3f} y={y:.3f} theta_deg={math.degrees(theta):.1f} "
            f"delta_deg={math.degrees(delta):.1f} dir={'CCW' if direction == 1 else 'CW'} [{context}]"
        )
        return direction

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

    def _transition_approach_front_to_orbit(self, now: float) -> None:
        """Set orbit direction and segment state from current pose, then enter orbit."""
        if self._pose_vtol is None:
            return
        x, y, yaw = self._pose_vtol
        self._orbit_dir = self._choose_orbit_dir(x, y, "approach_front")
        self._orbit_locked = True
        self._vtol_center = (-x, -y)
        self._desired_yaw = _wrap_angle(math.atan2(-y, -x))
        self._phase = "orbit"
        self._orbit_segment = "straight"
        self._orbit_straight_start_time = now
        theta_vtol = math.atan2(y, x)
        self._orbit_straight_heading = _wrap_angle(
            theta_vtol + self._orbit_dir * (math.pi / 2.0)
        )
        radial_err = self.dock_orbit_radius_m - math.hypot(x, y)
        self._orbit_straight_heading = _wrap_angle(
            self._orbit_straight_heading + 0.5 * radial_err
        )
        self._orbit_peek_start_time = None
        self._orbit_turn_back_start_time = None
        self.log(
            f"DOCK | approach_front -> orbit (dir={'CCW' if self._orbit_dir == 1 else 'CW'})"
        )
        # #region agent log
        try:
            import json as _json
            _r = math.hypot(x, y)
            _pay = {"location": "PayloadDriveToAprilTagMode.py:_transition_approach_front_to_orbit", "message": "enter_orbit", "data": {"x": x, "y": y, "r": _r, "R": self.dock_orbit_radius_m}, "timestamp": int(now * 1000), "hypothesisId": "R0"}
            for _path in ("/home/ubuntu/monorepo/.cursor/debug.log", "/tmp/payload_dock_debug.log"):
                try:
                    with open(_path, "a") as _f:
                        _f.write(_json.dumps(_pay) + "\n")
                except Exception:
                    pass
        except Exception:
            pass
        # #endregion

    def _control_approach_front(self, now: float, tag_results: dict, seen_ids: list) -> None:
        """Drive toward front tag (ID 0). Back up if < min_m; drive forward if > max_m; target band is [min_m, max_m] (e.g. 1.8–2.2 m)."""
        front_id = 0
        if front_id not in tag_results:
            v, w = self._last_cmd
            self._publish_drive(v, w)
            return

        _pose, _rvec, tvec, _corners, detection = tag_results[front_id]
        tvec_flat = np.asarray(tvec).ravel()
        tx, tz = float(tvec_flat[0]), float(tvec_flat[2])
        distance = tz

        # Desired heading = bearing to tag (point at tag). Looser gains for less tight steering.
        bearing = math.atan2(tx, tz)
        img_w = float(self._camera_info.width) if self._camera_info else 640.0
        cx = img_w / 2.0
        tag_cx = float(detection.center[0])
        err_x = tag_cx - cx
        angular = float(np.clip(
            -self.dock_approach_bearing_gain * bearing
            - self.angular_gain * self.dock_approach_pixel_scale * err_x,
            -0.4, 0.4
        ))

        # Too close (< 1.8 m): back up until in band [1.8, 2.2] m.
        if distance < self.dock_approach_front_min_m:
            dist_to_band = self.dock_approach_front_min_m - distance
            linear = float(np.clip(-self.linear_gain * dist_to_band, -0.15, -0.05))
            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self.log(
                    f"DOCK | approach_front BACKUP d={distance:.2f}m -> [{self.dock_approach_front_min_m:.1f},{self.dock_approach_front_max_m:.1f}]m "
                    f"v={linear:.3f} w={angular:.3f} tags={seen_ids}"
                )
            self._publish_drive(linear, angular)
            return

        # Hard safety: never command forward inside standoff (VTOL ~1.5 m long).
        if distance < self.dock_front_safe_standoff_m:
            linear = 0.0
            self._publish_drive(linear, angular)
            return

        # Too far (> 2.2 m): drive forward until in band [1.8, 2.2] m.
        dist_err = distance - self.dock_approach_front_max_m
        linear = float(np.clip(
            self.linear_gain * max(0.0, dist_err),
            0.05, 0.2,
        )) if dist_err > 0.0 else 0.0

        # #region agent log
        if now - getattr(self, "_af_ctrl_log_time", 0) >= 0.5:
            self._af_ctrl_log_time = now
            try:
                import json as _json
                with open("/tmp/payload_dock_debug.log", "a") as _f:
                    _f.write(_json.dumps({"location": "PayloadDriveToAprilTagMode.py:_control_approach_front", "message": "approach_front_ctrl", "data": {"distance": distance, "dist_err": dist_err, "linear": linear, "angular": angular, "dist_err_gt_005": dist_err > 0.05}, "timestamp": int(now * 1000), "hypothesisId": "AF"}) + "\n")
            except Exception:
                pass
        # #endregion

        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self.log(
                f"DOCK | approach_front d={distance:.2f}m err={dist_err:.2f}m "
                f"v={linear:.3f} w={angular:.3f} tags={seen_ids}"
            )
        self._publish_drive(linear, angular)

    # ------------------------------------------------------------------
    # Control: smooth orbit with persistent heading
    # ------------------------------------------------------------------

    def _control_orbit(self, now, time_delta, seen_ids, any_visible, tag_results) -> None:
        if self._pose_vtol is None:
            self._publish_drive(0.0, 0.0)
            return

        x_vtol, y_vtol, yaw = self._pose_vtol
        # Radius from pose (responsive); not from laggy _vtol_center.
        r = math.hypot(x_vtol, y_vtol)
        theta_vtol = math.atan2(y_vtol, x_vtol)
        R = max(self.dock_orbit_radius_m, 1e-3)
        back_id = self.dock_target_tag_id
        back_visible = back_id in tag_results if tag_results else False

        # Orbit -> dock when back tag visible and within 30° (same as normal dock entry)
        if back_visible and any_visible:
            view_angle = self._back_tag_view_angle_deg(tag_results)
            max_deg = self.dock_align_angle_deg
            aligned_now = (view_angle is not None) and (view_angle <= max_deg)
            if aligned_now:
                if self._dock_align_start is None:
                    self._dock_align_start = now
                if (now - self._dock_align_start) >= self.dock_align_hold_s:
                    self._phase = "dock"
                    self._dock_sub = "align"
                    self._dock_align_start = None
                    self.log(
                        f"DOCK | orbit -> dock (back tag view_angle={view_angle:.1f}° <= {max_deg}° held)"
                    )
                    self._publish_drive(0.0, 0.0)
                    return
            else:
                self._dock_align_start = None

        # Hard guard: too close to VTOL — force outward turn + small forward away (no inward spiral).
        if r < (self.dock_orbit_radius_m - 0.4):
            bearing_out = math.atan2(y_vtol, x_vtol)
            yaw_err_out = _wrap_angle(bearing_out - yaw)
            w = float(np.clip(2.0 * yaw_err_out, -0.6, 0.6))
            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self.log(
                    f"DOCK | orbit GUARD r={r:.3f}m < R-0.4 — steering outward v=0.05 w={w:.3f}"
                )
            self._publish_drive(0.05, w)
            return

        # Continuous vector-field orbit heading (every tick).
        heading_tangent = _wrap_angle(
            theta_vtol + self._orbit_dir * (math.pi / 2.0)
        )
        radial_err = r - self.dock_orbit_radius_m  # positive = outside circle
        # Stronger correction when inside circle to avoid drifting into VTOL.
        radial_k_eff = self.dock_orbit_radial_k * (2.0 if radial_err < 0 else 1.0)
        corr = math.atan(radial_k_eff * radial_err)
        heading_cmd = _wrap_angle(
            heading_tangent + self._orbit_dir * corr
        )
        yaw_err = _wrap_angle(heading_cmd - yaw)

        # Lazy init segment state
        if self._orbit_segment is None:
            self._orbit_segment = "straight"
            self._orbit_straight_start_time = now
            self._orbit_straight_heading = self._compute_straight_heading(x_vtol, y_vtol)
            self._orbit_peek_start_time = None
            self._orbit_turn_back_start_time = None

        # Tags lost: rotate in place to reacquire (no forward motion — avoids inward spiral).
        if not any_visible:
            w = 0.4 * (self._orbit_dir if self._orbit_dir else 1)
            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self.log(
                    f"DOCK | orbit tags lost — rotate in place r={r:.3f}m w={w:.3f} (no inward drift)"
                )
            self._publish_drive(0.0, w)
            return

        seg = self._orbit_segment

        # When back tag appears during straight, switch to 120° peek only after cooldown (avoid tight loop).
        # Otherwise: turn_back -> straight -> next tick back_visible -> peek_back_120 -> 120° -> turn_back -> straight -> repeat.
        # (peek_back commented out)
        # if seg == "straight" and back_visible:
        #     elapsed_straight = now - (self._orbit_straight_start_time or now)
        #     if elapsed_straight >= self.dock_orbit_peek_back_cooldown_s:
        #         self._orbit_segment = "peek_back_120"
        #         self._orbit_peek_start_time = now
        #         self._orbit_peek_start_yaw = yaw
        #         self._dock_align_start = None
        #         self.log(
        #             f"DOCK | orbit straight -> peek_back_120 (back tag seen, straight {elapsed_straight:.1f}s >= cooldown {self.dock_orbit_peek_back_cooldown_s}s)"
        #         )
        #         # #region agent log
        #         try:
        #             import json as _json
        #             _n = getattr(self, "_peek_back_120_count", 0) + 1
        #             self._peek_back_120_count = _n
        #             with open("/home/ubuntu/monorepo/.cursor/debug.log", "a") as _f:
        #                 _f.write(_json.dumps({"location": "PayloadDriveToAprilTagMode.py:straight_to_peek_back_120", "message": "peek_back_120_enter", "data": {"elapsed_straight": elapsed_straight, "count": _n}, "timestamp": int(now * 1000), "hypothesisId": "H_peek_loop"}) + "\n")
        #         except Exception:
        #             pass
        #         # #endregion
        #         seg = "peek_back_120"

        # --- straight: locked heading (actually drive straight, no feed-forward turn) ---
        if seg == "straight":
            elapsed = now - (self._orbit_straight_start_time or now)
            if elapsed >= self.dock_orbit_straight_duration_s:
                self._orbit_segment = "peek_turn"
                self._orbit_peek_start_time = now
                self._orbit_peek_start_yaw = yaw
                if now - self._last_log_time >= 0.5:
                    self._last_log_time = now
                    self.log(f"DOCK | orbit straight -> peek_turn (elapsed={elapsed:.1f}s)")

            # Locked heading (straight line); no w_ff during straight
            if self._orbit_straight_heading is None:
                self._orbit_straight_heading = self._compute_straight_heading(x_vtol, y_vtol)
            yaw_err = _wrap_angle(self._orbit_straight_heading - yaw)
            v = float(np.clip(self.dock_orbit_speed_mps, 0.05, 0.2))
            w = float(np.clip(self.dock_orbit_yaw_k * yaw_err, -0.4, 0.4))

            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self.log(
                    f"DOCK | orbit straight r={r:.3f} r_err={radial_err:.3f} "
                    f"h_lock={math.degrees(self._orbit_straight_heading):.1f}° v={v:.3f} w={w:.3f} tags={seen_ids}"
                )
            self._publish_drive(v, w)
            return

        # --- peek_back_120: rotate in place toward VTOL (up to 120°); enter dock only if view_angle <= dock_align_angle_deg (e.g. 30°) held --- (commented out)
        # if seg == "peek_back_120":
        #     view_angle = self._back_tag_view_angle_deg(tag_results)
        #     max_deg = self.dock_align_angle_deg
        #     aligned_now = (view_angle is not None) and (view_angle <= max_deg)
        #
        #     if aligned_now:
        #         if self._dock_align_start is None:
        #             self._dock_align_start = now
        #         if (now - self._dock_align_start) >= self.dock_align_hold_s:
        #             self._phase = "dock"
        #             self._dock_sub = "align"
        #             self._dock_align_start = None
        #             self.log(
        #                 f"DOCK | peek_back_120 -> dock (view_angle={view_angle:.1f}° <= {max_deg}° held)"
        #             )
        #             # #region agent log
        #             try:
        #                 import json as _json
        #                 with open("/home/ubuntu/monorepo/.cursor/debug.log", "a") as _f:
        #                     _f.write(_json.dumps({"location": "PayloadDriveToAprilTagMode.py:peek_back_120", "message": "dock_entry", "data": {"view_angle_deg": view_angle, "max_deg": max_deg}, "timestamp": int(now * 1000), "hypothesisId": "H1"}) + "\n")
        #             except Exception:
        #                 pass
        #             # #endregion
        #             self._publish_drive(0.0, 0.0)
        #             return
        #     else:
        #         self._dock_align_start = None
        #
        #     # Rotate toward VTOL (same idea as normal peek: we know where the VTOL is from tags)
        #     bearing_to_vtol = math.atan2(-y_vtol, -x_vtol)
        #     yaw_err_peek = _wrap_angle(bearing_to_vtol - yaw)
        #     w = float(np.clip(2.0 * yaw_err_peek, -0.6, 0.6))
        #     if abs(w) < 0.2:
        #         w = 0.2 * (1.0 if yaw_err_peek >= 0 else -1.0)
        #
        #     peek_start_yaw = getattr(self, "_orbit_peek_start_yaw", yaw)
        #     delta = abs(_wrap_angle(yaw - peek_start_yaw))
        #
        #     if delta >= math.radians(120.0):
        #         self._orbit_segment = "turn_back"
        #         self._orbit_turn_back_start_time = now
        #         self._dock_align_start = None
        #         self.log("DOCK | peek_back_120 -> turn_back (120° done, not aligned)")
        #         self._publish_drive(0.0, 0.0)
        #         return
        #
        #     if now - getattr(self, "_last_log_time", 0) >= 1.0:
        #         self._last_log_time = now
        #         va_str = f"{view_angle:.1f}°" if view_angle is not None else "N/A"
        #         self.log(
        #             f"DOCK | peek_back_120 rotating delta={math.degrees(delta):.1f}° view_angle={va_str}"
        #         )
        #     self._publish_drive(0.0, w)
        #     return

        # --- peek_turn: pure rotation (no forward creep); use tag_results for visibility ---
        if seg == "peek_turn":
            bearing_to_center = math.atan2(-y_vtol, -x_vtol)
            blend = self.dock_orbit_peek_blend
            heading_peek = _wrap_angle(
                heading_cmd + blend * _wrap_angle(bearing_to_center - heading_cmd)
            )
            yaw_err_peek = _wrap_angle(heading_peek - yaw)
            v = 0.0
            w_ff = self._orbit_dir * v / R
            w = float(np.clip(
                w_ff + self.dock_orbit_yaw_k * yaw_err_peek,
                -0.6, 0.6
            ))
            peek_elapsed = now - (self._orbit_peek_start_time or now)
            back_visible = self.dock_target_tag_id in (tag_results or {})
            peek_start_yaw = getattr(self, "_orbit_peek_start_yaw", None)
            # When back tag visible: exit after 120° turn (finds angle where tag is within 45°)
            if back_visible and peek_start_yaw is not None:
                delta_yaw = abs(_wrap_angle(yaw - peek_start_yaw))
                if delta_yaw >= math.radians(120.0):
                    self._orbit_segment = "turn_back"
                    self._orbit_turn_back_start_time = now
                    if now - self._last_log_time >= 0.5:
                        self._last_log_time = now
                        self.log("DOCK | orbit peek_turn -> turn_back (120° with back tag)")
            elif peek_elapsed >= self.dock_orbit_peek_duration_s:
                self._orbit_segment = "turn_back"
                self._orbit_turn_back_start_time = now
                if now - self._last_log_time >= 0.5:
                    self._last_log_time = now
                    self.log("DOCK | orbit peek_turn -> turn_back")
            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self.log(
                    f"DOCK | orbit peek r={r:.3f} r_err={radial_err:.3f} "
                    f"w={w:.3f} tags={seen_ids}"
                )
            self._publish_drive(v, w)
            return

        # --- turn_back: align to orbit heading_cmd, then straight ---
        if seg == "turn_back":
            yaw_err_tb = _wrap_angle(heading_cmd - yaw)
            w = float(np.clip(self.dock_orbit_yaw_k * yaw_err_tb, -0.6, 0.6))
            v = 0.05
            if abs(yaw_err_tb) < 0.12:
                self._orbit_segment = "straight"
                self._orbit_straight_start_time = now
                self._orbit_straight_heading = self._compute_straight_heading(x_vtol, y_vtol)
                if now - self._last_log_time >= 0.5:
                    self._last_log_time = now
                    self.log("DOCK | orbit turn_back -> straight (aligned)")
            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self.log(
                    f"DOCK | orbit turn_back r={r:.3f} err={math.degrees(yaw_err_tb):.1f}° "
                    f"w={w:.3f} tags={seen_ids}"
                )
            self._publish_drive(v, w)
            return

    # ------------------------------------------------------------------
    # Control: visual servo docking using raw camera measurements
    # ------------------------------------------------------------------

    def _control_dock(self, now, time_delta, tag_results, seen_ids) -> None:
        back_id = self.dock_target_tag_id
        if getattr(self, "_dock_sub", None) is None:
            self._dock_sub = "approach"

        sub = self._dock_sub

        # --- align: straight-on with back tag (no forward motion) before approach ---
        if sub == "align":
            if back_id not in tag_results:
                v, w = self._last_cmd
                self._publish_drive(v, w)
                return
            _pose, _rvec, tvec, _corners, detection = tag_results[back_id]
            tvec_flat = np.asarray(tvec).ravel()
            tx, tz = float(tvec_flat[0]), float(tvec_flat[2])
            img_w = float(self._camera_info.width) if self._camera_info else 640.0
            cx = img_w / 2.0
            tag_cx = float(detection.center[0])
            err_x = tag_cx - cx
            bearing = math.atan2(tx, tz)

            bearing_thresh = math.radians(5.0)
            pixel_thresh = 6.0
            aligned = (abs(bearing) <= bearing_thresh) and (abs(err_x) <= pixel_thresh)

            if aligned:
                if self._dock_align_start is None:
                    self._dock_align_start = now
                if (now - self._dock_align_start) >= self.dock_align_hold_s:
                    self._dock_sub = "approach"
                    self._dock_align_start = None
                    self.log("DOCK | align -> approach (straight-on confirmed)")
            else:
                self._dock_align_start = None

            w = float(np.clip(-1.2 * bearing - self.angular_gain * err_x, -0.6, 0.6))
            self._publish_drive(0.0, w)
            return

        # --- spin_180: turn once 180° at fixed rate (time-based), then stop and reverse ---
        if sub == "spin_180":
            spin_rate_rad_s = 0.5
            # Command full pi (180°); sim may apply slower so we allow slight over-command (1.05*pi) to ensure full turn
            spin_stop_rad = 1.05 * math.pi
            total_rot = getattr(self, "_dock_spin_total_rot", 0.0) + spin_rate_rad_s * time_delta
            self._dock_spin_total_rot = total_rot
            if total_rot >= spin_stop_rad:
                self._dock_sub = "back_up_5s"
                self._dock_back_up_5s_start = now
                self._publish_drive(0.0, 0.0)
                self.log("DOCK | dock spin_180 -> back_up_5s")
                return
            self._publish_drive(0.0, spin_rate_rad_s)
            return

        # --- back_up_5s: reverse for 5 s (no VTOL/tag check) then done ---
        if sub == "back_up_5s":
            t_start = getattr(self, "_dock_back_up_5s_start", now)
            if (now - t_start) >= 5.0:
                self.done = True
                self._publish_drive(0.0, 0.0)
                self.log(f"DOCK | complete (back_up_5s done, tags={seen_ids})")
                return
            self._publish_drive(-0.1, 0.0)
            return

        # --- approach: drive straight at back tag until at stop_distance ---
        if back_id not in tag_results:
            v, w = self._last_cmd
            self._publish_drive(v, w)
            return

        _pose, _rvec, tvec, _corners, detection = tag_results[back_id]
        tvec_flat = np.asarray(tvec).ravel()
        tx, tz = float(tvec_flat[0]), float(tvec_flat[2])
        distance = tz

        # When close enough, go to spin_180 then backup regardless of angle (avoid bouncing to align and never spinning)
        if distance <= self.stop_distance_m:
            self._publish_drive(0.0, 0.0)
            self._dock_sub = "spin_180"
            self._dock_spin_total_rot = 0.0
            self.log(
                f"DOCK | dock approach -> spin_180 (at {distance:.3f}m, then back 5s)"
            )
            return

        img_w = float(self._camera_info.width) if self._camera_info else 640.0
        cx = img_w / 2.0
        tag_cx = float(detection.center[0])
        err_x = tag_cx - cx
        bearing = math.atan2(tx, tz)
        view_angle_deg = self._back_tag_view_angle_deg(tag_results)
        max_angle_deg = self.dock_align_angle_deg
        approach_align_release_deg = max_angle_deg + 2.0

        if view_angle_deg is not None and view_angle_deg > approach_align_release_deg:
            self._dock_sub = "align"
            self.log(
                f"DOCK | approach -> align (view_angle={view_angle_deg:.1f}° > {approach_align_release_deg:.1f}°)"
            )
            self._publish_drive(0.0, 0.0)
            return
        if abs(bearing) > math.radians(max_angle_deg) or abs(err_x) > 25.0:
            self._dock_sub = "align"
            self.log(
                f"DOCK | approach -> align (bearing={math.degrees(bearing):.1f}° or off-center)"
            )
            self._publish_drive(0.0, 0.0)
            return

        # #region agent log
        if now - getattr(self, "_approach_angle_log_time", 0) >= 0.5:
            self._approach_angle_log_time = now
            try:
                import json as _json
                with open("/home/ubuntu/monorepo/.cursor/debug.log", "a") as _f:
                    _f.write(_json.dumps({"location": "PayloadDriveToAprilTagMode.py:approach", "message": "approach_angle", "data": {"view_angle_deg": view_angle_deg, "bearing_deg": math.degrees(bearing), "max_deg": max_angle_deg, "distance": distance}, "timestamp": int(now * 1000), "hypothesisId": "H2"}) + "\n")
            except Exception:
                pass
        # #endregion

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
                f"DOCK | dock approach dist={distance:.3f}m tx={tx:.3f}m "
                f"bearing={math.degrees(bearing):.1f}° v={linear:.3f} w={angular:.3f} tags={seen_ids}"
            )
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
