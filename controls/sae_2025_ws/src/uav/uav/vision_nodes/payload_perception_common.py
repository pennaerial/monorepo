from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo

try:
    import apriltag
except ImportError:
    apriltag = None

DEFAULT_TAG_FAMILY = "tag36h11"

# Standard VTOL AprilTag layout in simulation (from `sim/world_gen/models/standard_vtol/model.sdf`).
# IDs (`tag36h11`): front=0, back=1, left=2, right=3.
# Poses are relative_to="base_link" in SDF: (x, y, z, roll, pitch, yaw).
VTOL_TAG_POSES = {
    0: (-0.023, 0.0, 0.0, -1.5707, 3.141592, 1.5707),  # front
    1: (0.03, 0.0, 0.0, 1.5707, 0.0, 1.5707),  # back
    2: (0.0, 0.545, -0.037, 1.5707, 0.0, 3.141592),  # left
    3: (0.0, -0.545, -0.037, 1.5707, 0.0, 0.0),  # right
}


@dataclass(frozen=True)
class AprilTagObservation:
    tag_id: int
    center_x: float
    center_y: float
    tvec_x: float
    tvec_y: float
    tvec_z: float
    yaw_error: float
    area: float
    pose_x: Optional[float] = None
    pose_y: Optional[float] = None
    pose_yaw: Optional[float] = None


class AprilTagDetectorCache:
    def __init__(self) -> None:
        self._family: str | None = None
        self._detector = None
        self._backend: str | None = None

    def get(self, family: str | None):
        normalized_family = family or DEFAULT_TAG_FAMILY
        if self._detector is not None and self._family == normalized_family:
            return self._detector

        detector = None
        backend = None
        if apriltag is not None:
            try:
                options = apriltag.DetectorOptions(
                    families=normalized_family,
                    refine_edges=True,
                )
                detector = apriltag.Detector(options)
            except (AttributeError, TypeError):
                detector = apriltag.Detector()
            backend = "apriltag"

        self._family = normalized_family
        self._detector = detector
        self._backend = backend
        return detector

    @property
    def backend(self) -> Optional[str]:
        return self._backend


def _iter_detections(gray: np.ndarray, detector, backend: Optional[str]):
    if detector is None or backend != "apriltag":
        return []
    return detector.detect(gray) or []


def camera_model_from_info(camera_info: CameraInfo) -> tuple[np.ndarray, np.ndarray]:
    k_matrix = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.array(getattr(camera_info, "d", None) or [], dtype=np.float64)
    if dist_coeffs.size == 0:
        dist_coeffs = np.zeros(5, dtype=np.float64)
    if dist_coeffs.shape != (5,):
        dist_coeffs = np.resize(np.asarray(dist_coeffs).reshape(-1), 5)
    return k_matrix, dist_coeffs


def back_view_angle_deg(tvec_x: float, tvec_y: float, tvec_z: float) -> Optional[float]:
    norm = math.sqrt((tvec_x * tvec_x) + (tvec_y * tvec_y) + (tvec_z * tvec_z))
    if norm < 1e-6:
        return None
    cos_angle = max(-1.0, min(1.0, tvec_z / norm))
    return math.degrees(math.acos(cos_angle))


def compute_edge_metrics(
    bgr: np.ndarray,
    lower_pink: np.ndarray,
    upper_pink: np.ndarray,
    lower_green: np.ndarray,
    upper_green: np.ndarray,
) -> tuple[float, int, int]:
    height = bgr.shape[0]
    lower_half = bgr[height // 2 :, :]
    hsv = cv2.cvtColor(lower_half, cv2.COLOR_BGR2HSV)
    pink_mask = cv2.inRange(hsv, lower_pink, upper_pink)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    pink_count = int(np.count_nonzero(pink_mask))
    green_count = int(np.count_nonzero(green_mask))
    total = pink_count + green_count
    pink_ratio = pink_count / total if total > 0 else 0.5
    return pink_ratio, pink_count, green_count


def compute_edge_follow_control(
    bgr: np.ndarray,
    lower_pink: np.ndarray,
    upper_pink: np.ndarray,
    lower_green: np.ndarray,
    upper_green: np.ndarray,
    orbit_dir: int = 1,
) -> tuple[bool, float, float]:
    height, width = bgr.shape[:2]
    lower_half = bgr[height // 2 :, :]
    hsv = cv2.cvtColor(lower_half, cv2.COLOR_BGR2HSV)
    pink_mask = cv2.inRange(hsv, lower_pink, upper_pink)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    lower_height, lower_width = lower_half.shape[:2]
    center_x = lower_width / 2.0
    step = max(1, lower_width // 25)

    def boundary_x_at_row(row: int) -> Optional[float]:
        xs: list[float] = []
        ratios: list[float] = []
        for col in range(0, lower_width, step):
            col_pink = np.count_nonzero(pink_mask[row, col : col + step])
            col_green = np.count_nonzero(green_mask[row, col : col + step])
            total = col_pink + col_green
            if total < 10:
                continue
            xs.append(col + (step / 2.0))
            ratios.append(col_pink / total)

        if len(xs) < 2:
            return None

        for index in range(len(ratios) - 1):
            ratio_a = ratios[index]
            ratio_b = ratios[index + 1]
            if ratio_a >= 0.5 >= ratio_b or ratio_a <= 0.5 <= ratio_b:
                blend = (0.5 - ratio_a) / ((ratio_b - ratio_a) + 1e-9)
                return xs[index] + blend * (xs[index + 1] - xs[index])

        return sum(xs) / len(xs)

    x_top = boundary_x_at_row(0)
    x_bottom = boundary_x_at_row(lower_height - 1)
    boundary_center = None
    if x_top is not None:
        boundary_center = x_top
    if x_bottom is not None:
        if boundary_center is None:
            boundary_center = x_bottom
        else:
            boundary_center = (boundary_center + x_bottom) / 2.0

    if boundary_center is None:
        return False, 0.0, 0.0

    lateral_error_px = boundary_center - center_x
    if orbit_dir == -1:
        lateral_error_px = -lateral_error_px

    if x_top is not None and x_bottom is not None and lower_height > 1:
        dx = x_bottom - x_top
        dy = float(lower_height)
        boundary_angle = math.atan2(dx, dy)
    else:
        boundary_angle = 0.0

    return True, float(lateral_error_px), float(boundary_angle)


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


def _make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation.reshape(3)
    return transform


def _invert_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def _yaw_from_rotation(rotation_v_c: np.ndarray) -> float:
    forward = rotation_v_c @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return math.atan2(float(forward[1]), float(forward[0]))


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


def _estimate_camera_pose_in_vtol(
    tag_id: int, rvec: np.ndarray, tvec: np.ndarray
) -> Optional[tuple[float, float, float]]:
    if tag_id not in VTOL_TAG_POSES:
        return None

    tx, ty, tz, rr, rp, ry = VTOL_TAG_POSES[int(tag_id)]
    rotation_v_t = _rpy_to_rot(rr, rp, ry)
    transform_v_t = _make_transform(
        rotation_v_t, np.array([tx, ty, tz], dtype=np.float64)
    )
    rotation_c_t, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    transform_c_t = _make_transform(
        rotation_c_t, np.asarray(tvec, dtype=np.float64).reshape(3, 1)
    )
    transform_v_c = transform_v_t @ _invert_transform(transform_c_t)
    position_v = transform_v_c[:3, 3]
    yaw_v = _yaw_from_rotation(transform_v_c[:3, :3])
    return (float(position_v[0]), float(position_v[1]), float(yaw_v))


def _yaw_error_from_rvec(rvec: np.ndarray) -> float:
    rotation_c_t, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return math.atan2(float(rotation_c_t[0, 2]), float(-rotation_c_t[2, 2]))


def _solve_detection_pose(
    detection,
    object_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    *,
    flags: int | None = None,
    min_forward_distance_m: float | None = None,
) -> Optional[
    tuple[int, np.ndarray, np.ndarray, np.ndarray, float, float, float, float]
]:
    tag_id = int(getattr(detection, "tag_id", -1))
    corners = np.asarray(detection.corners, dtype=np.float32)
    if len(corners) != 4:
        return None

    solve_kwargs = {}
    if flags is not None:
        solve_kwargs["flags"] = flags
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        corners,
        camera_matrix,
        dist_coeffs,
        **solve_kwargs,
    )
    if not ok:
        return None

    tvec_flat = np.asarray(tvec, dtype=np.float64).reshape(-1)
    if len(tvec_flat) < 3:
        return None
    if min_forward_distance_m is not None and float(tvec_flat[2]) <= float(
        min_forward_distance_m
    ):
        return None

    return (
        tag_id,
        np.asarray(rvec, dtype=np.float64),
        tvec_flat,
        float(detection.center[0]),
        float(detection.center[1]),
        float(_yaw_error_from_rvec(rvec)),
        float(abs(cv2.contourArea(corners.reshape(-1, 1, 2).astype(np.float32)))),
        float(tvec_flat[2]),
    )


def detect_payload_apriltags(
    gray: np.ndarray,
    camera_info: CameraInfo,
    detector,
    detector_backend: Optional[str],
    tag_size_m: float,
) -> list[AprilTagObservation]:
    if detector is None:
        return []

    camera_matrix = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)
    donor_dist_coeffs = np.zeros((4, 1), dtype=np.float64)
    object_points = _object_points_for_tag_size(float(tag_size_m))
    observations: list[AprilTagObservation] = []

    for detection in _iter_detections(gray, detector, detector_backend):
        solved = _solve_detection_pose(
            detection,
            object_points,
            camera_matrix,
            donor_dist_coeffs,
        )
        if solved is None:
            continue

        tag_id, rvec, tvec_flat, center_x, center_y, yaw_error, area, _ = solved
        pose = _estimate_camera_pose_in_vtol(tag_id, rvec, tvec_flat)

        observations.append(
            AprilTagObservation(
                tag_id=tag_id,
                center_x=float(center_x),
                center_y=float(center_y),
                tvec_x=float(tvec_flat[0]),
                tvec_y=float(tvec_flat[1]),
                tvec_z=float(tvec_flat[2]),
                yaw_error=float(yaw_error),
                area=float(area),
                pose_x=float(pose[0]) if pose is not None else None,
                pose_y=float(pose[1]) if pose is not None else None,
                pose_yaw=float(pose[2]) if pose is not None else None,
            )
        )

    return observations


def solve_payload_apriltags(
    gray: np.ndarray,
    camera_info: CameraInfo,
    detector,
    detector_backend: Optional[str],
    tag_size_m: float,
) -> list[AprilTagObservation]:
    if detector is None:
        return []

    camera_matrix, dist_coeffs = camera_model_from_info(camera_info)
    object_points = _object_points_for_tag_size(float(tag_size_m))
    observations: list[AprilTagObservation] = []

    for detection in _iter_detections(gray, detector, detector_backend):
        solved = _solve_detection_pose(
            detection,
            object_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
            min_forward_distance_m=0.05,
        )
        if solved is None:
            continue

        tag_id, rvec, tvec_flat, center_x, center_y, yaw_error, area, _ = solved
        pose = _estimate_camera_pose_in_vtol(tag_id, rvec, tvec_flat)
        if pose is None:
            continue

        observations.append(
            AprilTagObservation(
                tag_id=tag_id,
                center_x=float(center_x),
                center_y=float(center_y),
                tvec_x=float(tvec_flat[0]),
                tvec_y=float(tvec_flat[1]),
                tvec_z=float(tvec_flat[2]),
                yaw_error=float(yaw_error),
                area=float(area),
                pose_x=float(pose[0]),
                pose_y=float(pose[1]),
                pose_yaw=float(pose[2]),
            )
        )

    return observations


def detect_payload_unreeled(
    image,
    lower_hsv: tuple[int, int, int] = (0, 0, 180),
    upper_hsv: tuple[int, int, int] = (180, 20, 255),
    debug: bool = False,
):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h = image.shape[0]
    floor_strip = hsv[int(h * 0.75) : h, :]  # bottom 25%

    white_mask = cv2.inRange(floor_strip, np.array(lower_hsv), np.array(upper_hsv))

    # Erode to remove small noise specks, then dilate to restore remaining blobs
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    white_mask = cv2.erode(white_mask, kernel, iterations=2)
    white_mask = cv2.dilate(white_mask, kernel, iterations=2)

    white_count = np.sum(white_mask > 0)
    white_ratio = white_count / white_mask.size

    debug_frame = None
    if debug:
        vis = image.copy()
        h_full = vis.shape[0]
        cv2.rectangle(
            vis, (0, int(h_full * 0.75)), (vis.shape[1], h_full), (0, 255, 255), 2
        )
        vis[int(h_full * 0.75) : h_full, :][white_mask > 0] = (255, 100, 0)
        cv2.putText(
            vis,
            f"white_ratio={white_ratio:.3f}  count={white_count}",
            (8, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        debug_frame = vis

    return white_mask, white_ratio, white_count, debug_frame


def _make_camera_info(image: np.ndarray) -> CameraInfo:
    """Synthesise a plausible pinhole CameraInfo from image dimensions."""
    h, w = image.shape[:2]
    fx = fy = float(max(w, h))
    cx, cy = w / 2.0, h / 2.0
    info = CameraInfo()
    info.width = w
    info.height = h
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    return info


if __name__ == "__main__":
    import sys

    USAGE = """
Usage:
  python payload_perception_common.py unreeled <image>
  python payload_perception_common.py apriltag  <image> [tag_size_m] [tag_family]

Examples:
  python payload_perception_common.py unreeled frame.jpg
  python payload_perception_common.py apriltag frame.jpg 0.1 tag36h11
"""

    if len(sys.argv) < 3:
        print(USAGE)
        sys.exit(1)

    command = sys.argv[1].lower()
    image_path = sys.argv[2]

    bgr = cv2.imread(image_path)
    if bgr is None:
        print(f"ERROR: could not load image '{image_path}'")
        sys.exit(1)

    if command == "unreeled":
        mask, ratio, count, debug_frame = detect_payload_unreeled(bgr, debug=True)
        print(f"white_ratio={ratio:.4f}  white_count={count}")
        cv2.imshow("detect_payload_unreeled", debug_frame)
        cv2.imshow("white_mask", mask)
        print("Press any key to close.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    elif command == "apriltag":
        tag_size_m = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
        tag_family = sys.argv[4] if len(sys.argv) > 4 else DEFAULT_TAG_FAMILY

        cache = AprilTagDetectorCache()
        detector = cache.get(tag_family)
        if detector is None:
            print("ERROR: apriltag library not installed.")
            sys.exit(1)

        camera_info = _make_camera_info(bgr)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        all_obs = detect_payload_apriltags(
            gray, camera_info, detector, cache.backend, tag_size_m
        )
        solved_obs = solve_payload_apriltags(
            gray, camera_info, detector, cache.backend, tag_size_m
        )

        print(f"detected={len(all_obs)}  solved={len(solved_obs)}")
        for o in all_obs:
            pose_str = (
                f"  pose=({o.pose_x:.3f}, {o.pose_y:.3f}, yaw={o.pose_yaw:.3f})"
                if o.pose_x is not None
                else ""
            )
            print(
                f"  id={o.tag_id}  tvec=({o.tvec_x:.3f}, {o.tvec_y:.3f}, {o.tvec_z:.3f})"
                f"  yaw_err={o.yaw_error:.3f}  area={o.area:.0f}{pose_str}"
            )

    else:
        print(f"ERROR: unknown command '{command}'")
        print(USAGE)
        sys.exit(1)
