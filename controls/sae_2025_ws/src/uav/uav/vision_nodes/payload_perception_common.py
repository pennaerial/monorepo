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
_CV2_APRILTAG_DICTIONARIES = {
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

# Standard VTOL AprilTag layout in simulation (from `sim/world_gen/models/standard_vtol/model.sdf`).
# IDs (DICT_APRILTAG_36h11): front=0, back=1, left=2, right=3.
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
    pose_x: float
    pose_y: float
    pose_yaw: float
    tvec_x: float
    tvec_y: float
    tvec_z: float
    center_x: float
    center_y: float


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
        else:
            dictionary_id = _CV2_APRILTAG_DICTIONARIES.get(normalized_family)
            if dictionary_id is not None and hasattr(cv2, "aruco"):
                detector = cv2.aruco.ArucoDetector(
                    cv2.aruco.getPredefinedDictionary(dictionary_id)
                )
                backend = "opencv"

        self._family = normalized_family
        self._detector = detector
        self._backend = backend
        return detector

    @property
    def backend(self) -> Optional[str]:
        return self._backend


@dataclass(frozen=True)
class _DetectionAdapter:
    tag_id: int
    corners: np.ndarray
    center: tuple[float, float]


def _iter_detections(gray: np.ndarray, detector, backend: Optional[str]):
    if detector is None:
        return []
    if backend == "opencv":
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is None:
            return []
        detections: list[_DetectionAdapter] = []
        for marker_corners, marker_id in zip(corners, ids.flatten(), strict=False):
            corners_array = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
            center = tuple(np.mean(corners_array, axis=0).tolist())
            detections.append(
                _DetectionAdapter(
                    tag_id=int(marker_id),
                    corners=corners_array,
                    center=(float(center[0]), float(center[1])),
                )
            )
        return detections
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
        tag_id = int(getattr(detection, "tag_id", -1))
        if tag_id not in VTOL_TAG_POSES:
            continue

        corners = np.array(detection.corners, dtype=np.float32)
        if len(corners) != 4:
            continue

        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            corners,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            continue

        tvec_flat = np.asarray(tvec, dtype=np.float64).reshape(-1)
        if len(tvec_flat) < 3 or float(tvec_flat[2]) <= 0.05:
            continue

        pose = _estimate_camera_pose_in_vtol(tag_id, rvec, tvec)
        if pose is None:
            continue

        observations.append(
            AprilTagObservation(
                tag_id=tag_id,
                pose_x=pose[0],
                pose_y=pose[1],
                pose_yaw=pose[2],
                tvec_x=float(tvec_flat[0]),
                tvec_y=float(tvec_flat[1]),
                tvec_z=float(tvec_flat[2]),
                center_x=float(detection.center[0]),
                center_y=float(detection.center[1]),
            )
        )

    return observations
