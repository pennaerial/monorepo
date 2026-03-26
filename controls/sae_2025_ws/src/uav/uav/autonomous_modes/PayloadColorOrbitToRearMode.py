"""
PayloadColorOrbitToRearMode: navigate to rear when starting on opposite side of VTOL.

Uses AprilTag for localization/decision and DLZ color boundary (pink/green) for motion.
When rear tag becomes reachable, hands off to PayloadDriveToAprilTagMode for docking.
Reuses same publishers, subscriptions, AprilTag detection, and pose helpers as
PayloadDriveToAprilTagMode. Does not duplicate docking logic.
"""

from typing import Optional, Tuple

import numpy as np
import cv2
import math
import time as _time
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from payload_interfaces.msg import DriveCommand
from uav.autonomous_modes import Mode
from uav import UAV
from cv_bridge import CvBridge
from uav.utils import pink, green

try:
    import apriltag
except ImportError:
    apriltag = None

# Reuse tag layout and helpers from PayloadDriveToAprilTagMode
from uav.autonomous_modes.PayloadDriveToAprilTagMode import (
    DEFAULT_TAG_FAMILY,
    _VTOL_TAG_POSES,
    _wrap_angle,
    _rpy_to_rot,
    _make_T,
    _invert_T,
    _yaw_from_R_v_c,
    _object_points_for_tag_size,
)


def _estimate_camera_pose_in_vtol(tag_id: int, rvec: np.ndarray, tvec: np.ndarray) -> Optional[Tuple[float, float, float]]:
    """Same pose estimation as PayloadDriveToAprilTagMode."""
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


def _back_tag_view_angle_deg(tag_results: dict, back_id: int) -> Optional[float]:
    """Angle between camera forward and direction to back tag center."""
    if back_id not in tag_results:
        return None
    tvec = np.asarray(tag_results[back_id][2]).ravel()
    norm = float(np.linalg.norm(tvec))
    if norm < 1e-6:
        return None
    cos_angle = float(tvec[2] / norm)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_angle))))


class PayloadColorOrbitToRearMode(Mode):
    """
    Navigate payload to rear of VTOL when starting on opposite side.

    Startup: full 360° rotation in place (no translation), collect AprilTags and estimate
    pose relative to VTOL. After scan: if rear tag visible -> dock immediately; else lock
    orbit direction and outward heading (never recompute), drive to DLZ edge, turn 90°,
    follow boundary by color; when rear tag aligned -> hand off to docking.

    Tags used only for decisions; color used for motion control. No orbit direction flipping.
    Search occurs only once at activation.
    """

    def __init__(
        self,
        node: Node,
        uav: UAV,
        payload_name: str = "payload_0",
        tag_size_m: float = 0.1,
        tag_family: str = DEFAULT_TAG_FAMILY,
        dock_target_tag_id: int = 1,
        dock_align_angle_deg: float = 30.0,
        dock_align_hold_s: float = 0.4,
        search_spin_rps: float = 0.3,
        go_to_edge_speed_mps: float = 0.12,
        edge_threshold: float = 0.35,
        edge_stable_frames: int = 5,
        edge_min_pixels: int = 500,
        edge_invalid_max_frames: int = 30,
        edge_orbit_speed_mps: float = 0.12,
        edge_k_lat: float = 0.008,
        edge_k_ang: float = 0.6,
        handoff_dock_mode_name: str = "dock",
    ):
        super().__init__(node, uav)
        self.payload_name = str(payload_name)
        self.tag_size_m = float(tag_size_m)
        self.tag_family = str(tag_family) or DEFAULT_TAG_FAMILY
        self.dock_target_tag_id = int(dock_target_tag_id)
        self.dock_align_angle_deg = float(dock_align_angle_deg)
        self.dock_align_hold_s = float(dock_align_hold_s)
        self.search_spin_rps = float(search_spin_rps)
        self.go_to_edge_speed_mps = float(go_to_edge_speed_mps)
        self.edge_threshold = float(edge_threshold)
        self.edge_stable_frames = int(edge_stable_frames)
        self.edge_min_pixels = int(edge_min_pixels)
        self.edge_invalid_max_frames = int(edge_invalid_max_frames)
        self.edge_orbit_speed_mps = float(edge_orbit_speed_mps)
        self.edge_k_lat = float(edge_k_lat)
        self.edge_k_ang = float(edge_k_ang)
        self.handoff_dock_mode_name = str(handoff_dock_mode_name)

        self._image = None
        self._camera_info = None
        self._bridge = CvBridge()
        self._detector = None
        if apriltag is not None:
            try:
                options = apriltag.DetectorOptions(families=self.tag_family, refine_edges=True)
                self._detector = apriltag.Detector(options)
            except (AttributeError, TypeError):
                self._detector = apriltag.Detector()

        # HSV bounds from uav.utils (H, S, V)
        self._lower_pink = np.array(pink[0], dtype=np.uint8)
        self._upper_pink = np.array(pink[1], dtype=np.uint8)
        self._lower_green = np.array(green[0], dtype=np.uint8)
        self._upper_green = np.array(green[1], dtype=np.uint8)

    def on_enter(self) -> None:
        self._image = None
        self._camera_info = None
        self._phase = "search"
        self._orbit_dir = None  # +1 CCW, -1 CW; locked after scan, never recomputed
        self._orbit_locked = False
        self._pose_vtol = None  # (x, y, yaw) from tag pose estimation
        self._search_total_rot = 0.0  # rad turned during 360° scan
        self._search_poses = []  # (tid, pose) collected during scan
        self._search_rear_seen = False  # rear tag seen during scan -> dock immediately
        self._outward_heading = None  # rad, away from VTOL; locked after scan
        self._edge_align_start = None
        self._edge_align_yaw_turned = 0.0  # rad turned so far in EDGE_ALIGN
        self._dock_align_start = None
        self._edge_stable_count = 0
        self._edge_invalid_count = 0
        self._last_log_time = 0.0
        self._handoff_requested = False

        cam_topic = f"/{self.payload_name}/camera"
        info_topic = f"/{self.payload_name}/camera_info"
        self.node.get_logger().info(
            f"PayloadColorOrbitToRearMode: subscribing to {cam_topic}, {info_topic}"
        )
        self._image_sub = self.node.create_subscription(Image, cam_topic, self._image_cb, 10)
        self._info_sub = self.node.create_subscription(CameraInfo, info_topic, self._info_cb, 10)
        drive_topic = f"/{self.payload_name}/cmd_drive"
        self._drive_pub = self.node.create_publisher(DriveCommand, drive_topic, 10)
        self.node.get_logger().info(
            f"PayloadColorOrbitToRearMode: publishing to {drive_topic}"
        )

    def _image_cb(self, msg: Image) -> None:
        self._image = msg

    def _info_cb(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _publish_drive(self, linear: float, angular: float) -> None:
        self._drive_pub.publish(DriveCommand(linear=float(linear), angular=float(angular)))

    def _process_tags(self, detections, K, dist_coeffs) -> Tuple[dict, list]:
        """Same tag processing as PayloadDriveToAprilTagMode: tag_results, seen_ids."""
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
                obj_pts, corners, K, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ok or float(np.asarray(tvec).ravel()[2]) <= 0.05:
                continue
            pose = _estimate_camera_pose_in_vtol(tag_id, rvec, tvec)
            if pose is None:
                continue
            tag_results[tag_id] = (pose, rvec, tvec, corners, d)
        return tag_results, seen_ids

    def _compute_edge_metrics(self, bgr: np.ndarray) -> Tuple[float, int, int]:
        """Lower half of image: pink_ratio, pink_count, green_count. Returns (pink_ratio, pink_count, green_count)."""
        h, w = bgr.shape[:2]
        lower = bgr[h // 2 :, :]
        hsv = cv2.cvtColor(lower, cv2.COLOR_BGR2HSV)
        pink_mask = cv2.inRange(hsv, self._lower_pink, self._upper_pink)
        green_mask = cv2.inRange(hsv, self._lower_green, self._upper_green)
        pink_count = int(np.count_nonzero(pink_mask))
        green_count = int(np.count_nonzero(green_mask))
        total = pink_count + green_count
        pink_ratio = pink_count / total if total > 0 else 0.5
        return pink_ratio, pink_count, green_count

    def _edge_detected(self, pink_ratio: float, pink_count: int, green_count: int) -> bool:
        total = pink_count + green_count
        if total < self.edge_min_pixels:
            return False
        return pink_ratio < self.edge_threshold

    def _compute_edge_follow_control(self, bgr: np.ndarray) -> Tuple[float, float]:
        """From lower half: lateral_error (px from center), boundary_angle (rad). CCW follow left edge, CW follow right."""
        h, w = bgr.shape[:2]
        lower = bgr[h // 2 :, :]
        hsv = cv2.cvtColor(lower, cv2.COLOR_BGR2HSV)
        pink_mask = cv2.inRange(hsv, self._lower_pink, self._upper_pink)
        green_mask = cv2.inRange(hsv, self._lower_green, self._upper_green)
        lh, lw = lower.shape[:2]
        center_x = lw / 2.0
        step = max(1, lw // 25)

        def boundary_x_at_row(row: int) -> Optional[float]:
            xs, ratios = [], []
            for col in range(0, lw, step):
                col_pink = np.count_nonzero(pink_mask[row, col : col + step])
                col_green = np.count_nonzero(green_mask[row, col : col + step])
                total = col_pink + col_green
                if total < 10:
                    continue
                xs.append(col + step / 2.0)
                ratios.append(col_pink / total)
            if len(xs) < 2:
                return None
            for i in range(len(ratios) - 1):
                if ratios[i] >= 0.5 >= ratios[i + 1] or ratios[i] <= 0.5 <= ratios[i + 1]:
                    t = (0.5 - ratios[i]) / (ratios[i + 1] - ratios[i] + 1e-9)
                    return xs[i] + t * (xs[i + 1] - xs[i])
            return sum(xs) / len(xs) if xs else None

        x_top = boundary_x_at_row(0)
        x_bot = boundary_x_at_row(lh - 1)
        boundary_center = None
        if x_top is not None:
            boundary_center = x_top
        if x_bot is not None:
            boundary_center = (boundary_center + x_bot) / 2.0 if boundary_center is not None else x_bot

        if boundary_center is None:
            return 0.0, 0.0

        lateral_error_px = boundary_center - center_x
        if self._orbit_dir == -1:
            lateral_error_px = -lateral_error_px

        if x_top is not None and x_bot is not None and lh > 1:
            dx = x_bot - x_top
            dy = float(lh)
            boundary_angle = math.atan2(dx, dy)
        else:
            boundary_angle = 0.0

        return lateral_error_px, boundary_angle

    def on_update(self, time_delta: float) -> None:
        now = _time.time()
        if self._handoff_requested:
            return
        if self._detector is None:
            self.log("PayloadColorOrbitToRearMode: apriltag not installed.")
            return
        if self._image is None or self._camera_info is None:
            return
        try:
            gray = self._bridge.imgmsg_to_cv2(self._image, desired_encoding="mono8")
            bgr = self._bridge.imgmsg_to_cv2(self._image, desired_encoding="bgr8")
        except Exception as e:
            self.node.get_logger().warn(f"PayloadColorOrbitToRearMode: image convert failed: {e}")
            return
        K = np.array(self._camera_info.k, dtype=np.float64).reshape(3, 3)
        d = getattr(self._camera_info, "d", None) or []
        dist_coeffs = np.array(d, dtype=np.float64)
        if dist_coeffs.size == 0:
            dist_coeffs = np.zeros(5)
        if dist_coeffs.shape != (5,):
            dist_coeffs = np.resize(np.asarray(dist_coeffs).reshape(-1), 5)

        detections = self._detector.detect(gray)
        tag_results, seen_ids = self._process_tags(detections, K, dist_coeffs)
        any_visible = len(tag_results) > 0
        back_visible = self.dock_target_tag_id in tag_results

        if any_visible:
            closest_id = min(
                tag_results,
                key=lambda tid: float(np.asarray(tag_results[tid][2]).ravel()[2]),
            )
            self._pose_vtol = tag_results[closest_id][0]
        if back_visible and self._phase == "search":
            self._search_rear_seen = True

        # ----- SEARCH: full 360° rotation in place, collect all tags and poses -----
        if self._phase == "search":
            spin_w = self.search_spin_rps * 2.0 * math.pi
            self._search_total_rot += abs(spin_w) * time_delta
            if any_visible:
                for tid in tag_results:
                    self._search_poses.append((tid, tag_results[tid][0]))
            if self._search_total_rot < 2.0 * math.pi:
                if now - self._last_log_time >= 2.0:
                    self._last_log_time = now
                    self.log(
                        f"PayloadColorOrbitToRearMode: SEARCH scanning "
                        f"{math.degrees(self._search_total_rot):.0f}/360° "
                        f"tags={len(set(p[0] for p in self._search_poses))} rear_seen={self._search_rear_seen}"
                    )
                self._publish_drive(0.0, spin_w)
                return
            self.log(
                f"PayloadColorOrbitToRearMode: SEARCH complete 360° "
                f"sightings={len(self._search_poses)} rear_seen={self._search_rear_seen}"
            )
            if self._search_rear_seen:
                self._phase = "handoff_to_dock"
                self._handoff_requested = True
                self._publish_drive(0.0, 0.0)
                self.log("PayloadColorOrbitToRearMode: rear tag seen during scan -> HANDOFF_TO_DOCK (dock immediately)")
                return
            if self._pose_vtol is None:
                self.log("PayloadColorOrbitToRearMode: no pose after scan, cannot proceed")
                self._publish_drive(0.0, 0.0)
                return
            self._phase = "decide_direction"

        # ----- DECIDE_DIRECTION: lock orbit direction and outward heading (never recomputed) -----
        if self._phase == "decide_direction":
            x, y, yaw = self._pose_vtol
            theta = math.atan2(y, x)
            angle_to_rear = _wrap_angle(math.pi - theta)
            self._orbit_dir = 1 if angle_to_rear >= 0 else -1
            self._outward_heading = theta
            self._orbit_locked = True
            self._phase = "go_to_edge"
            self.log(
                f"PayloadColorOrbitToRearMode: DECIDE_DIRECTION -> GO_TO_EDGE "
                f"dir={'CCW' if self._orbit_dir == 1 else 'CW'} outward={math.degrees(self._outward_heading):.1f}° (locked)"
            )

        # ----- GO_TO_EDGE -----
        if self._phase == "go_to_edge":
            pink_ratio, pink_count, green_count = self._compute_edge_metrics(bgr)
            at_edge = self._edge_detected(pink_ratio, pink_count, green_count)
            total = pink_count + green_count
            valid = total >= self.edge_min_pixels

            if not valid:
                self._edge_invalid_count += 1
                if self._edge_invalid_count >= self.edge_invalid_max_frames:
                    self.log("PayloadColorOrbitToRearMode: GO_TO_EDGE too many invalid frames, stopping")
                    self._publish_drive(0.0, 0.0)
                    return
                self._publish_drive(self.go_to_edge_speed_mps, 0.0)
                return
            self._edge_invalid_count = 0

            if at_edge:
                self._edge_stable_count += 1
                if self._edge_stable_count >= self.edge_stable_frames:
                    self._publish_drive(0.0, 0.0)
                    self._phase = "edge_align"
                    self._edge_stable_count = 0
                    self._edge_align_yaw_turned = 0.0
                    self._edge_align_start = now
                    self.log("PayloadColorOrbitToRearMode: GO_TO_EDGE -> EDGE_ALIGN (edge detected)")
                    return
            else:
                self._edge_stable_count = 0

            yaw_err = _wrap_angle(self._outward_heading - self._pose_vtol[2]) if self._pose_vtol else 0.0
            # Deadband + lower gain to avoid angular flip-flop (circles) from pose noise/overshoot
            yaw_deadband = 0.12  # rad (~7°)
            if abs(yaw_err) <= yaw_deadband:
                w = 0.0
            else:
                w = float(np.clip(1.0 * yaw_err, -0.5, 0.5))
            self._publish_drive(self.go_to_edge_speed_mps, w)
            return

        # ----- EDGE_ALIGN -----
        if self._phase == "edge_align":
            turn_target = math.pi / 2.0
            if self._edge_align_yaw_turned >= turn_target - 0.1:
                self._phase = "edge_orbit"
                self.log("PayloadColorOrbitToRearMode: EDGE_ALIGN -> EDGE_ORBIT")
                return
            w = 0.5 if self._orbit_dir == 1 else -0.5
            self._edge_align_yaw_turned += abs(w) * time_delta
            self._publish_drive(0.0, w)
            return

        # ----- EDGE_ORBIT: follow boundary by color only; tags only for rear-alignment check -----
        if self._phase == "edge_orbit":
            lateral_px, boundary_angle = self._compute_edge_follow_control(bgr)
            angular = self.edge_k_lat * lateral_px + self.edge_k_ang * boundary_angle
            angular = float(np.clip(angular, -0.5, 0.5))
            linear = self.edge_orbit_speed_mps

            view_angle = _back_tag_view_angle_deg(tag_results, self.dock_target_tag_id)
            aligned = view_angle is not None and view_angle <= self.dock_align_angle_deg
            if aligned and back_visible:
                if self._dock_align_start is None:
                    self._dock_align_start = now
                elif (now - self._dock_align_start) >= self.dock_align_hold_s:
                    self._phase = "handoff_to_dock"
                    self._handoff_requested = True
                    self._publish_drive(0.0, 0.0)
                    self.log(
                        f"PayloadColorOrbitToRearMode: EDGE_ORBIT -> HANDOFF_TO_DOCK "
                        f"(back tag aligned {view_angle:.1f}° for {self.dock_align_hold_s}s)"
                    )
                    return
            else:
                self._dock_align_start = None

            self._publish_drive(linear, angular)
            return

        # ----- HANDOFF_TO_DOCK -----
        if self._phase == "handoff_to_dock":
            self._publish_drive(0.0, 0.0)
            return

    def check_status(self) -> str:
        if self._handoff_requested and self._phase == "handoff_to_dock":
            return "handoff_dock"
        return "continue"

    def on_exit(self) -> None:
        if hasattr(self, "_drive_pub"):
            self._drive_pub.publish(DriveCommand(linear=0.0, angular=0.0))
