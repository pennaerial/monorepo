from __future__ import annotations

import importlib
import sys
from pathlib import Path
from types import SimpleNamespace
import types

import numpy as np
import cv2
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from uav.cv.dlz_convex_hull import build_dlz_hull_mask  # noqa: E402


def _bgr_from_hsv(h: int, s: int, v: int) -> tuple[int, int, int]:
    pixel = cv2.cvtColor(
        np.uint8([[[h, s, v]]]),
        cv2.COLOR_HSV2BGR,
    )[0, 0]
    return int(pixel[0]), int(pixel[1]), int(pixel[2])


ORANGE_BGR = _bgr_from_hsv(15, 150, 220)
BLUE_BGR = _bgr_from_hsv(110, 220, 220)


def _roi_ratio(mask: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> float:
    region = mask[y0:y1, x0:x1]
    if region.size == 0:
        return 0.0
    return float(np.count_nonzero(region)) / float(region.size)


def _install_import_stubs() -> None:
    if "rclpy" not in sys.modules:
        rclpy = types.ModuleType("rclpy")
        node_module = types.ModuleType("rclpy.node")

        class Node:
            pass

        node_module.Node = Node
        rclpy.node = node_module
        sys.modules.update({"rclpy": rclpy, "rclpy.node": node_module})

    if "cv_bridge" not in sys.modules:
        cv_bridge = types.ModuleType("cv_bridge")

        class CvBridge:
            def imgmsg_to_cv2(self, *_args, **_kwargs):
                raise NotImplementedError

            def cv2_to_compressed_imgmsg(self, *_args, **_kwargs):
                return SimpleNamespace(header=SimpleNamespace(stamp=None))

        cv_bridge.CvBridge = CvBridge
        sys.modules["cv_bridge"] = cv_bridge

    if "sensor_msgs" not in sys.modules:
        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.CompressedImage = type("CompressedImage", (), {})
        sensor_msgs_msg.Image = type("Image", (), {})
        sensor_msgs.msg = sensor_msgs_msg
        sys.modules.update(
            {"sensor_msgs": sensor_msgs, "sensor_msgs.msg": sensor_msgs_msg}
        )

    if "uav.vehicles.Payload" not in sys.modules:
        payload_module = types.ModuleType("uav.vehicles.Payload")
        payload_module.Payload = type("Payload", (), {})
        sys.modules["uav.vehicles.Payload"] = payload_module

    if "uav.vision_nodes" not in sys.modules:
        vision_nodes = types.ModuleType("uav.vision_nodes")
        vision_nodes.PayloadAprilTagNode = type("PayloadAprilTagNode", (), {})
        sys.modules["uav.vision_nodes"] = vision_nodes

    if "uav.vision_nodes.payload_perception_common" not in sys.modules:
        common = types.ModuleType("uav.vision_nodes.payload_perception_common")
        common.DEFAULT_TAG_FAMILY = "tag36h11"
        sys.modules["uav.vision_nodes.payload_perception_common"] = common

    if "uav_interfaces" not in sys.modules:
        sys.modules["uav_interfaces"] = types.ModuleType("uav_interfaces")

    if "uav_interfaces.srv" not in sys.modules:
        srv_module = types.ModuleType("uav_interfaces.srv")

        class PayloadAprilTagState:
            class Request:
                pass

            class Response:
                pass

        srv_module.PayloadAprilTagState = PayloadAprilTagState
        sys.modules["uav_interfaces.srv"] = srv_module


_install_import_stubs()

PayloadCornerNavigateMode = importlib.import_module(
    "uav.modes.payload.PayloadCornerNavigateMode"
).PayloadCornerNavigateMode
PayloadDLZNavigateMode = importlib.import_module(
    "uav.modes.payload.PayloadDLZNavigateMode"
).PayloadDLZNavigateMode


class _FakeLogger:
    def info(self, _message: str) -> None:
        pass

    def warn(self, _message: str) -> None:
        pass

    def warning(self, _message: str) -> None:
        pass

    def error(self, _message: str) -> None:
        pass


class _FakeNode:
    def __init__(self) -> None:
        self._logger = _FakeLogger()

    def get_logger(self) -> _FakeLogger:
        return self._logger


class _RecordingVehicle:
    def __init__(self) -> None:
        self.commands: list[tuple] = []

    def drive(self, linear: float, angular: float) -> None:
        self.commands.append(("drive", float(linear), float(angular)))

    def stop(self) -> None:
        self.commands.append(("stop",))


def _orange_rect_frame() -> np.ndarray:
    frame = np.zeros((220, 260, 3), dtype=np.uint8)
    frame[40:180, 30:210] = ORANGE_BGR
    return frame


def _empty_mask() -> np.ndarray:
    return np.zeros((24, 24), dtype=np.uint8)


def _make_turn_to_center_mode(**kwargs):
    vehicle = _RecordingVehicle()
    mode = PayloadCornerNavigateMode(
        node=_FakeNode(),
        vehicle=vehicle,
        align_angular_speed=0.6,
        center_tol_px=20.0,
        center_min_pixels=50,
        center_stable_frames=2,
        **kwargs,
    )
    mode._phase = "turn_to_center"
    mode._turn_to_center_rad = 0.0
    mode._center_stable = 0
    mode._latest_dominant = None
    mode._first_color = "A"
    mode._prev_color = None
    mode._annotated_pub = None
    mode._done = False
    mode._terminate = False
    mode._decode_image = lambda: np.zeros((32, 32, 3), dtype=np.uint8)
    return mode, vehicle


def _contour_bbox(mask: np.ndarray) -> tuple[int, int, int, int]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    assert len(contours) == 1
    return cv2.boundingRect(contours[0])


def test_build_dlz_hull_mask_keeps_orange_rectangle_and_excludes_outside():
    frame = _orange_rect_frame()

    mask = build_dlz_hull_mask(frame)

    assert mask is not None
    assert _roi_ratio(mask, 50, 50, 190, 170) > 0.95
    assert _roi_ratio(mask, 0, 0, 25, 220) == 0.0
    assert _roi_ratio(mask, 220, 0, 260, 220) == 0.0


def test_corner_mode_single_color_metrics_ignore_outside_dlz_distractors():
    frame = _orange_rect_frame()
    frame[150:215, 0:25] = BLUE_BGR

    mode = PayloadCornerNavigateMode(
        node=SimpleNamespace(),
        vehicle=SimpleNamespace(),
        cw_lower_hsv=(100, 100, 100),
        cw_upper_hsv=(130, 255, 255),
    )

    count, lateral_error_px = mode._single_color_strip_metrics(frame, "B")

    assert count == 0
    assert lateral_error_px == 0.0


def test_dlz_mode_detect_color_ignores_outside_dlz_distractors():
    frame = _orange_rect_frame()
    frame[150:215, 0:25] = BLUE_BGR

    mode = PayloadDLZNavigateMode(
        node=SimpleNamespace(),
        vehicle=SimpleNamespace(),
        cw_lower_hsv=(100, 100, 100),
        cw_upper_hsv=(130, 255, 255),
        ccw_lower_hsv=(0, 80, 80),
        ccw_upper_hsv=(10, 255, 255),
    )

    (
        current_color,
        boundary_detected,
        lateral_error_px,
        _boundary_angle,
        mask_a,
        mask_b,
        _strip,
        _strip_start,
    ) = mode._detect_color(frame)

    assert current_color == "none"
    assert boundary_detected is False
    assert lateral_error_px == 0.0
    assert np.count_nonzero(mask_a) == 0
    assert np.count_nonzero(mask_b) == 0


def test_primary_contour_mask_chooses_smallest_width_when_multiple_contours_pass():
    mode = PayloadCornerNavigateMode(node=_FakeNode(), vehicle=_RecordingVehicle())
    mask = np.zeros((100, 120), dtype=np.uint8)
    cv2.rectangle(mask, (8, 20), (40, 70), 255, cv2.FILLED)
    cv2.rectangle(mask, (72, 12), (82, 82), 255, cv2.FILLED)

    selected = mode._primary_contour_mask(mask, min_area_px=200)
    x, _y, w, _h = _contour_bbox(selected)

    assert x >= 72
    assert w <= 11


def test_primary_contour_mask_ignores_narrow_noise_that_fails_threshold():
    mode = PayloadCornerNavigateMode(node=_FakeNode(), vehicle=_RecordingVehicle())
    mask = np.zeros((100, 120), dtype=np.uint8)
    cv2.rectangle(mask, (10, 25), (28, 80), 255, cv2.FILLED)
    cv2.rectangle(mask, (75, 10), (79, 22), 255, cv2.FILLED)

    selected = mode._primary_contour_mask(mask, min_area_px=150)
    x, _y, w, _h = _contour_bbox(selected)

    assert x <= 10
    assert w >= 19


@pytest.mark.parametrize(
    ("direction", "expected_angular"),
    [("ccw", 0.6), ("cw", -0.6)],
)
def test_turn_to_center_searches_in_nominal_direction_without_visible_tape(
    direction: str,
    expected_angular: float,
):
    mode, vehicle = _make_turn_to_center_mode(direction=direction)
    mode._turn_both_color_metrics = lambda _bgr: (
        0,
        0.0,
        None,
        _empty_mask(),
        _empty_mask(),
        0,
    )

    mode._update_turn_to_center(0.5)

    assert vehicle.commands[-1] == ("drive", 0.0, expected_angular)
    assert mode._center_stable == 0
    assert mode._turn_to_center_rad == pytest.approx(0.3)


@pytest.mark.parametrize(
    ("lateral_px", "expected_angular"),
    [(35.0, -0.6), (-35.0, 0.6)],
)
def test_turn_to_center_turns_toward_center_from_centroid_sign(
    lateral_px: float,
    expected_angular: float,
):
    mode, vehicle = _make_turn_to_center_mode()
    mode._turn_both_color_metrics = lambda _bgr: (
        80,
        lateral_px,
        "A",
        _empty_mask(),
        _empty_mask(),
        0,
    )

    mode._update_turn_to_center(0.25)

    assert vehicle.commands[-1] == ("drive", 0.0, expected_angular)
    assert mode._center_stable == 0
    assert mode._turn_to_center_rad == pytest.approx(0.15)


def test_turn_to_center_locks_after_stable_frames_and_seeds_prev_color():
    mode, vehicle = _make_turn_to_center_mode()
    mode._turn_both_color_metrics = lambda _bgr: (
        80,
        5.0,
        "B",
        _empty_mask(),
        _empty_mask(),
        0,
    )

    mode._update_turn_to_center(0.25)

    assert vehicle.commands[-1] == ("stop",)
    assert mode._center_stable == 1
    assert mode._phase == "turn_to_center"
    assert mode._latest_dominant == "B"

    mode._update_turn_to_center(0.25)

    assert vehicle.commands[-1] == ("stop",)
    assert mode._center_stable == 2
    assert mode._phase == "line_follow"
    assert mode._prev_color == "B"
    assert mode._turn_to_center_rad == pytest.approx(0.0)


def test_turn_to_center_resets_stable_when_target_leaves_tolerance():
    mode, vehicle = _make_turn_to_center_mode()
    mode._center_stable = 1
    mode._turn_both_color_metrics = lambda _bgr: (
        80,
        32.0,
        "A",
        _empty_mask(),
        _empty_mask(),
        0,
    )

    mode._update_turn_to_center(0.25)

    assert mode._center_stable == 0
    assert vehicle.commands[-1] == ("drive", 0.0, -0.6)
    assert mode._turn_to_center_rad == pytest.approx(0.15)
