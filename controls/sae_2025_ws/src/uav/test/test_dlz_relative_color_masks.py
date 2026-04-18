from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from uav.cv.dlz_relative_color_masks import detect_dlz_relative_color_masks  # noqa: E402


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


def _orange_frame(height: int = 180, width: int = 240) -> np.ndarray:
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:] = ORANGE_BGR
    return frame


def _roi_ratio(mask: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> float:
    region = mask[y0:y1, x0:x1]
    if region.size == 0:
        return 0.0
    return float(np.count_nonzero(region)) / float(region.size)


def test_relative_dlz_masks_detect_blue_purple_and_green_from_orange_reference():
    frame = _orange_frame()
    frame[20:80, 20:80] = BLUE_BGR
    frame[20:80, 150:210] = PURPLE_BGR
    frame[100:160, 85:155] = GREEN_BGR

    result = detect_dlz_relative_color_masks(
        frame,
        dlz_mask=np.full(frame.shape[:2], 255, dtype=np.uint8),
    )

    assert _roi_ratio(result.mask_blue, 20, 20, 80, 80) > 0.95
    assert _roi_ratio(result.mask_purple, 150, 20, 210, 80) > 0.95
    assert _roi_ratio(result.mask_green, 85, 100, 155, 160) > 0.95

    assert _roi_ratio(result.mask_blue, 150, 20, 210, 80) < 0.05
    assert _roi_ratio(result.mask_blue, 85, 100, 155, 160) < 0.05
    assert _roi_ratio(result.mask_purple, 20, 20, 80, 80) < 0.05
    assert _roi_ratio(result.mask_purple, 85, 100, 155, 160) < 0.05
    assert _roi_ratio(result.mask_green, 20, 20, 80, 80) < 0.05
    assert _roi_ratio(result.mask_green, 150, 20, 210, 80) < 0.05


def test_relative_dlz_masks_respect_the_supplied_dlz_roi_mask():
    frame = _orange_frame()
    frame[20:80, 20:80] = BLUE_BGR
    frame[20:80, 150:210] = PURPLE_BGR
    frame[100:160, 85:155] = GREEN_BGR

    dlz_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    dlz_mask[:, :120] = 255

    result = detect_dlz_relative_color_masks(frame, dlz_mask=dlz_mask)

    assert _roi_ratio(result.mask_blue, 20, 20, 80, 80) > 0.95
    assert _roi_ratio(result.mask_green, 85, 100, 120, 160) > 0.90
    assert _roi_ratio(result.mask_purple, 150, 20, 210, 80) == 0.0
    assert np.count_nonzero(result.mask_purple) == 0
