from __future__ import annotations

import cv2
import numpy as np

from uav.cv.dlz_masking import (
    DLZOrangeBarrierMaskConfig,
    detect_dlz_orange_barrier_mask,
)


def _bgr_from_hsv(h: int, s: int, v: int) -> tuple[int, int, int]:
    hsv = np.uint8([[[h, s, v]]])
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return tuple(int(channel) for channel in bgr)


def _blank_frame(height: int = 120, width: int = 160) -> np.ndarray:
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:] = (35, 40, 30)
    return frame


def _rect_mask(
    height: int,
    width: int,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
) -> np.ndarray:
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[y0:y1, x0:x1] = 255
    return mask


def _default_config() -> DLZOrangeBarrierMaskConfig:
    return DLZOrangeBarrierMaskConfig(
        orange_barrier_hsv=(((10, 180, 180), (20, 255, 255)),),
        seed_reject_hsv=(
            ((100, 180, 180), (120, 255, 255)),
            ((135, 100, 100), (170, 255, 255)),
            ((21, 160, 180), (32, 255, 255)),
        ),
        seed_patch_size_px=24,
        seed_barrier_ratio_threshold=0.10,
        seed_reject_ratio_threshold=0.10,
        min_orange_pixels=1500,
        min_component_pixels=300,
        min_mask_area_frac=0.15,
        max_mask_area_frac=0.85,
        min_bbox_span_ratio=0.45,
        morphology_kernel_size=3,
        dilate_iterations=0,
    )


def test_detect_dlz_orange_barrier_masks_inside_region_and_preserves_shape():
    frame = _blank_frame()
    orange_bgr = _bgr_from_hsv(15, 220, 220)
    blue_bgr = _bgr_from_hsv(110, 220, 220)
    purple_bgr = _bgr_from_hsv(150, 180, 220)
    yellow_bgr = _bgr_from_hsv(25, 230, 230)

    frame[18:102, 20:140] = orange_bgr
    frame[34:90, 34:44] = blue_bgr
    frame[52:62, 44:116] = purple_bgr
    frame[26:94, 116:128] = yellow_bgr

    result = detect_dlz_orange_barrier_mask(frame, _default_config())

    expected_mask = _rect_mask(frame.shape[0], frame.shape[1], 20, 18, 140, 102)
    assert result.plausible is True
    assert result.passthrough is False
    assert result.reason == "masked"
    assert result.frame.shape == frame.shape
    assert result.frame.dtype == frame.dtype
    assert result.component_bbox == (20, 18, 120, 84)
    assert np.array_equal(result.applied_mask, expected_mask)
    assert np.count_nonzero(result.outside_fill_mask) > 0
    assert len(result.seed_points) == 4
    assert result.barrier_component_count == 1
    assert result.candidate_region_count == 1
    assert result.used_hull_merge is False
    assert np.array_equal(result.frame[18:102, 20:140], frame[18:102, 20:140])
    assert np.all(result.frame[expected_mask == 0] == 0)


def test_detect_dlz_orange_barrier_ignores_small_barrier_components():
    frame = _blank_frame()
    orange_bgr = _bgr_from_hsv(15, 220, 220)

    frame[18:102, 20:140] = orange_bgr
    frame[6:14, 6:14] = orange_bgr
    frame[6:14, 146:154] = orange_bgr

    result = detect_dlz_orange_barrier_mask(frame, _default_config())

    assert result.plausible is True
    assert result.barrier_component_count == 1
    assert result.orange_pixels == 120 * 84
    assert result.component_bbox == (20, 18, 120, 84)
    assert result.used_hull_merge is False


def test_detect_dlz_orange_barrier_rejects_contaminated_corner_but_uses_other_seeds():
    frame = _blank_frame()
    orange_bgr = _bgr_from_hsv(15, 220, 220)
    purple_bgr = _bgr_from_hsv(150, 180, 220)

    frame[20:104, 24:142] = orange_bgr
    frame[:24, :24] = purple_bgr

    result = detect_dlz_orange_barrier_mask(frame, _default_config())

    assert result.plausible is True
    assert result.passthrough is False
    assert (0, 0) not in result.seed_points
    assert len(result.seed_points) == 3
    assert result.component_bbox == (24, 20, 118, 84)


def test_detect_dlz_orange_barrier_returns_passthrough_when_no_seed_corner_is_valid():
    frame = _blank_frame()
    orange_bgr = _bgr_from_hsv(15, 220, 220)
    purple_bgr = _bgr_from_hsv(150, 180, 220)

    frame[20:104, 24:142] = orange_bgr
    frame[:24, :24] = purple_bgr
    frame[:24, -24:] = purple_bgr
    frame[-24:, :24] = purple_bgr
    frame[-24:, -24:] = purple_bgr

    result = detect_dlz_orange_barrier_mask(frame, _default_config())

    assert result.plausible is False
    assert result.passthrough is True
    assert result.reason == "no_seed_corners"
    assert result.seed_points == ()
    assert np.array_equal(result.frame, frame)


def test_detect_dlz_orange_barrier_hulls_multiple_candidate_regions():
    frame = _blank_frame()
    orange_bgr = _bgr_from_hsv(15, 220, 220)

    frame[18:102, 18:58] = orange_bgr
    frame[18:102, 62:102] = orange_bgr

    result = detect_dlz_orange_barrier_mask(
        frame,
        DLZOrangeBarrierMaskConfig(
            **{
                **_default_config().__dict__,
                "min_orange_pixels": 5000,
                "max_mask_area_frac": 0.95,
                "min_bbox_span_ratio": 0.50,
            }
        ),
    )

    expected_mask = _rect_mask(frame.shape[0], frame.shape[1], 18, 18, 102, 102)
    assert result.plausible is True
    assert result.used_hull_merge is True
    assert result.candidate_region_count == 2
    assert result.component_bbox == (18, 18, 84, 84)
    assert np.array_equal(result.applied_mask, expected_mask)


def test_detect_dlz_orange_barrier_rejects_far_apart_regions_via_hull_guard():
    frame = _blank_frame()
    orange_bgr = _bgr_from_hsv(15, 220, 220)

    frame[20:96, 8:28] = orange_bgr
    frame[20:96, 132:152] = orange_bgr

    result = detect_dlz_orange_barrier_mask(
        frame,
        DLZOrangeBarrierMaskConfig(
            **{
                **_default_config().__dict__,
                "min_orange_pixels": 2500,
                "max_mask_area_frac": 0.95,
            }
        ),
    )

    assert result.plausible is False
    assert result.passthrough is True
    assert result.reason == "hull_too_large"
    assert result.candidate_region_count == 2
    assert result.used_hull_merge is True
    assert np.array_equal(result.frame, frame)
