from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from uav.cv.dlz_color_regions import (  # noqa: E402
    DLZLineColorMaskConfig,
    detect_dlz_line_color_regions,
)

BLUE_BGR = (177, 75, 54)
PURPLE_BGR = (156, 85, 170)
PINK_BGR = (127, 62, 147)
YELLOW_BGR = (69, 201, 227)


def _blank_frame(height: int = 180, width: int = 240) -> np.ndarray:
    return np.zeros((height, width, 3), dtype=np.uint8)


def _roi_ratio(mask: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> float:
    region = mask[y0:y1, x0:x1]
    if region.size == 0:
        return 0.0
    return float(np.count_nonzero(region)) / float(region.size)


def _component_count(mask: np.ndarray) -> int:
    binary = (mask > 0).astype(np.uint8)
    if not np.any(binary):
        return 0
    component_count, _labels = cv2.connectedComponents(binary, 8)
    return int(component_count - 1)


def _config() -> DLZLineColorMaskConfig:
    return DLZLineColorMaskConfig(
        color_a_bgr=BLUE_BGR,
        color_b_bgr=PURPLE_BGR,
        reject_colors_bgr=(PINK_BGR, YELLOW_BGR),
        nonblack_min_value=45,
        nonblack_min_saturation=35,
        min_region_area_px=20,
        match_max_distance_lab=38.0,
        match_min_margin_lab=8.0,
    )


def test_detect_dlz_line_color_regions_accepts_blue_and_purple_and_rejects_pad_colors():
    frame = _blank_frame()
    frame[18:88, 18:88] = BLUE_BGR
    frame[18:88, 150:220] = PURPLE_BGR
    frame[104:174, 18:88] = PINK_BGR
    frame[104:174, 150:220] = YELLOW_BGR

    result = detect_dlz_line_color_regions(frame, _config())

    assert _roi_ratio(result.mask_a, 18, 18, 88, 88) > 0.90
    assert _roi_ratio(result.mask_b, 150, 18, 220, 88) > 0.90
    assert _roi_ratio(result.mask_unknown, 18, 18, 88, 88) < 0.05
    assert _roi_ratio(result.mask_unknown, 150, 18, 220, 88) < 0.05

    assert _roi_ratio(result.mask_a, 18, 104, 88, 174) < 0.05
    assert _roi_ratio(result.mask_b, 18, 104, 88, 174) < 0.05
    assert _roi_ratio(result.mask_a, 150, 104, 220, 174) < 0.05
    assert _roi_ratio(result.mask_b, 150, 104, 220, 174) < 0.05
    assert _roi_ratio(result.mask_unknown, 18, 104, 88, 174) > 0.90
    assert _roi_ratio(result.mask_unknown, 150, 104, 220, 174) > 0.90


def test_detect_dlz_line_color_regions_leaves_unmatched_region_unknown():
    frame = _blank_frame()
    frame[48:132, 52:116] = (40, 190, 60)

    result = detect_dlz_line_color_regions(frame, _config())

    assert np.count_nonzero(result.mask_a) == 0
    assert np.count_nonzero(result.mask_b) == 0
    assert _roi_ratio(result.mask_unknown, 52, 48, 116, 132) > 0.95


def test_detect_dlz_line_color_regions_separates_touching_regions():
    frame = _blank_frame()
    frame[48:132, 52:116] = BLUE_BGR
    frame[48:132, 116:180] = PURPLE_BGR

    result = detect_dlz_line_color_regions(frame, _config())

    assert _roi_ratio(result.mask_a, 52, 48, 116, 132) > 0.90
    assert _roi_ratio(result.mask_b, 116, 48, 180, 132) > 0.90
    assert _roi_ratio(result.mask_a, 116, 48, 180, 132) < 0.05
    assert _roi_ratio(result.mask_b, 52, 48, 116, 132) < 0.05
    assert _component_count(result.mask_a) == 1
    assert _component_count(result.mask_b) == 1
