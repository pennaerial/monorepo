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

BLUE_BGR = (177, 75, 54)
PURPLE_BGR = (156, 85, 170)
PINK_BGR = (127, 62, 147)
YELLOW_BGR = (69, 201, 227)


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


def _blank_frame(height: int = 180, width: int = 240) -> np.ndarray:
    return np.zeros((height, width, 3), dtype=np.uint8)


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
        ccw_lower_hsv=(100, 100, 100),
        ccw_upper_hsv=(130, 255, 255),
        cw_lower_hsv=(140, 0, 85),
        cw_upper_hsv=(170, 255, 255),
        region_color_match_enabled=True,
        ccw_color_bgr=BLUE_BGR,
        cw_color_bgr=PURPLE_BGR,
        reject_colors_bgr=(PINK_BGR, YELLOW_BGR),
    )


@pytest.fixture()
def payload_dlz_mode():
    return PayloadDLZNavigateMode(
        node=SimpleNamespace(),
        vehicle=SimpleNamespace(),
        ccw_lower_hsv=(100, 100, 100),
        ccw_upper_hsv=(130, 255, 255),
        cw_lower_hsv=(140, 0, 85),
        cw_upper_hsv=(170, 255, 255),
        region_color_match_enabled=True,
        ccw_color_bgr=BLUE_BGR,
        cw_color_bgr=PURPLE_BGR,
        reject_colors_bgr=(PINK_BGR, YELLOW_BGR),
    )


def test_corner_mode_turn_metrics_only_count_configured_blue_and_purple(
    payload_corner_mode,
):
    frame = _blank_frame()
    frame[124:158, 20:76] = BLUE_BGR
    frame[126:160, 144:224] = PURPLE_BGR
    frame[124:160, 84:106] = PINK_BGR
    frame[124:160, 112:136] = YELLOW_BGR

    total, lateral_error_px, dominant, mask_a, mask_b, row_start = (
        payload_corner_mode._turn_both_color_metrics(frame)
    )

    assert row_start == 120
    assert total == int(np.count_nonzero(mask_a)) + int(np.count_nonzero(mask_b))
    assert dominant == "B"
    assert lateral_error_px > 0.0

    assert _roi_ratio(mask_a, 20, 4, 76, 38) > 0.90
    assert _roi_ratio(mask_b, 144, 6, 224, 40) > 0.90
    assert _roi_ratio(mask_a, 84, 4, 106, 40) < 0.05
    assert _roi_ratio(mask_b, 84, 4, 106, 40) < 0.05
    assert _roi_ratio(mask_a, 112, 4, 136, 40) < 0.05
    assert _roi_ratio(mask_b, 112, 4, 136, 40) < 0.05


def test_dlz_mode_detect_color_ignores_pink_and_yellow_distractors(payload_dlz_mode):
    frame = _blank_frame()
    frame[126:178, 24:124] = BLUE_BGR
    frame[126:178, 160:220] = PURPLE_BGR
    frame[126:178, 126:144] = PINK_BGR
    frame[126:178, 144:158] = YELLOW_BGR

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

    assert _roi_ratio(mask_b, 24, 6, 124, 58) > 0.90
    assert _roi_ratio(mask_a, 160, 6, 220, 58) > 0.90
    assert _roi_ratio(mask_a, 126, 6, 144, 58) < 0.05
    assert _roi_ratio(mask_b, 126, 6, 144, 58) < 0.05
    assert _roi_ratio(mask_a, 144, 6, 158, 58) < 0.05
    assert _roi_ratio(mask_b, 144, 6, 158, 58) < 0.05
