from __future__ import annotations

import importlib
import sys
from pathlib import Path
from types import SimpleNamespace
import types

import cv2
import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


def _bgr_from_hsv(h: int, s: int, v: int) -> tuple[int, int, int]:
    pixel = cv2.cvtColor(
        np.uint8([[[h, s, v]]]),
        cv2.COLOR_HSV2BGR,
    )[0, 0]
    return int(pixel[0]), int(pixel[1]), int(pixel[2])


ORANGE_BGR = _bgr_from_hsv(15, 150, 220)
BLUE_BGR = _bgr_from_hsv(105, 110, 170)
PURPLE_BGR = _bgr_from_hsv(145, 100, 170)
GREEN_BGR = _bgr_from_hsv(60, 100, 170)


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
        sys.modules.update({"sensor_msgs": sensor_msgs, "sensor_msgs.msg": sensor_msgs_msg})

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


def _orange_frame(height: int = 180, width: int = 240) -> np.ndarray:
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:] = ORANGE_BGR
    return frame


def _roi_ratio(mask: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> float:
    region = mask[y0:y1, x0:x1]
    if region.size == 0:
        return 0.0
    return float(np.count_nonzero(region)) / float(region.size)


@pytest.fixture()
def payload_corner_mode():
    return PayloadCornerNavigateMode(
        node=SimpleNamespace(),
        vehicle=SimpleNamespace(),
    )


@pytest.fixture()
def payload_dlz_mode():
    return PayloadDLZNavigateMode(
        node=SimpleNamespace(),
        vehicle=SimpleNamespace(),
    )


def test_corner_mode_turn_metrics_use_blue_and_purple_and_ignore_green(
    payload_corner_mode,
):
    frame = _orange_frame()
    frame[124:158, 20:76] = BLUE_BGR
    frame[126:160, 144:224] = PURPLE_BGR
    frame[124:160, 84:106] = GREEN_BGR

    total, lateral_error_px, dominant, mask_a, mask_b, row_start = (
        payload_corner_mode._turn_both_color_metrics(frame)
    )

    assert row_start == 120
    assert total == int(np.count_nonzero(mask_a)) + int(np.count_nonzero(mask_b))
    assert dominant == "B"
    assert lateral_error_px > 0.0

    assert _roi_ratio(mask_a, 20, 4, 76, 38) > 0.95
    assert _roi_ratio(mask_b, 144, 6, 224, 40) > 0.95
    assert _roi_ratio(mask_a, 84, 4, 106, 40) < 0.05
    assert _roi_ratio(mask_b, 84, 4, 106, 40) < 0.05


def test_corner_mode_guide_metrics_follow_green_mask(payload_corner_mode):
    frame = _orange_frame()
    frame[110:180, 100:145] = GREEN_BGR

    count, lateral_error_px = payload_corner_mode._guide_centroid_metrics(frame)
    line_count, line_lateral_error_px = payload_corner_mode._single_color_strip_metrics(
        frame,
        "A",
    )

    assert count > 2500
    assert abs(lateral_error_px) < 5.0
    assert line_count == 0
    assert line_lateral_error_px == 0.0


def test_corner_mode_single_color_strip_metrics_follow_blue_mask(payload_corner_mode):
    frame = _orange_frame()
    frame[124:178, 40:120] = BLUE_BGR

    count, lateral_error_px = payload_corner_mode._single_color_strip_metrics(frame, "A")

    assert count > 3000
    assert lateral_error_px < 0.0


def test_dlz_mode_detect_color_maps_purple_to_a_and_blue_to_b(payload_dlz_mode):
    frame = _orange_frame()
    frame[126:178, 24:124] = BLUE_BGR
    frame[126:178, 160:220] = PURPLE_BGR
    frame[126:178, 126:158] = GREEN_BGR

    (
        current_color,
        boundary_detected,
        lateral_error_px,
        boundary_angle,
        mask_a,
        mask_b,
        strip,
        strip_start,
    ) = payload_dlz_mode._detect_color(frame)

    assert strip_start == 120
    assert strip.shape == frame[strip_start:, :].shape
    assert boundary_detected is True
    assert current_color == "B"
    assert lateral_error_px < 0.0
    assert abs(boundary_angle) < 0.2

    assert _roi_ratio(mask_a, 160, 6, 220, 58) > 0.95
    assert _roi_ratio(mask_b, 24, 6, 124, 58) > 0.95
    assert _roi_ratio(mask_a, 126, 6, 158, 58) < 0.05
    assert _roi_ratio(mask_b, 126, 6, 158, 58) < 0.05
