from __future__ import annotations

import importlib
import sys
from pathlib import Path
from types import SimpleNamespace
import types

import numpy as np
import cv2

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


def _orange_rect_frame() -> np.ndarray:
    frame = np.zeros((220, 260, 3), dtype=np.uint8)
    frame[40:180, 30:210] = ORANGE_BGR
    return frame


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
