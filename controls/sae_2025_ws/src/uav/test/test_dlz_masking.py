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


def _blank_frame(height: int = 220, width: int = 220) -> np.ndarray:
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


def test_detect_dlz_orange_barrier_masks_large_rectangle_and_preserves_shape():
    frame = _blank_frame()
    orange_bgr = _bgr_from_hsv(15, 220, 220)

    frame[30:190, 40:180] = orange_bgr

    result = detect_dlz_orange_barrier_mask(frame, _default_config())

    expected_mask = _rect_mask(frame.shape[0], frame.shape[1], 40, 30, 180, 190)
    assert result.plausible is True
    assert result.passthrough is False
    assert result.reason == "masked"
    assert result.frame.shape == frame.shape
    assert result.frame.dtype == frame.dtype
    assert result.applied_mask.dtype == np.uint8
    assert np.array_equal(result.applied_mask, expected_mask)
    assert np.array_equal(result.barrier_mask, expected_mask)
    assert result.component_bbox == (40, 30, 140, 160)
    assert result.mask_area_px == 140 * 160
    assert result.orange_pixels == 140 * 160
    assert result.seed_points == ((0, 0), (frame.shape[1] - 1, 0))
    assert result.barrier_component_count == 1
    assert result.candidate_region_count == 1
    assert result.used_hull_merge is False
    assert np.count_nonzero(result.outside_fill_mask) > 0
    assert np.array_equal(result.frame[expected_mask == 255], frame[expected_mask == 255])
    assert np.all(result.frame[expected_mask == 0] == 0)


def test_detect_dlz_orange_barrier_ignores_seed_reject_hsv_and_still_uses_top_seeds():
    frame = _blank_frame()
    orange_bgr = _bgr_from_hsv(15, 220, 220)
    purple_bgr = _bgr_from_hsv(150, 180, 220)

    frame[30:190, 40:180] = orange_bgr
    frame[:24, :24] = purple_bgr
    frame[:24, -24:] = purple_bgr

    result = detect_dlz_orange_barrier_mask(frame, _default_config())

    assert result.plausible is True
    assert result.passthrough is False
    assert result.reason == "masked"
    assert result.seed_points == ((0, 0), (frame.shape[1] - 1, 0))
    assert result.candidate_region_count == 1


def test_detect_dlz_orange_barrier_returns_passthrough_when_no_contour_survives_area_cutoff():
    frame = _blank_frame()
    orange_bgr = _bgr_from_hsv(15, 220, 220)

    frame[40:90, 50:100] = orange_bgr

    result = detect_dlz_orange_barrier_mask(frame, _default_config())

    assert result.plausible is False
    assert result.passthrough is True
    assert result.reason == "no_dlz_region"
    assert result.mask_area_px == 0
    assert result.component_bbox is None
    assert result.orange_pixels == 50 * 50
    assert result.seed_points == ((0, 0), (frame.shape[1] - 1, 0))
    assert result.barrier_component_count == 1
    assert result.candidate_region_count == 0
    assert result.used_hull_merge is False
    assert np.array_equal(result.frame, frame)


def test_detect_dlz_orange_barrier_ignores_subthreshold_contours_when_large_one_survives():
    frame = _blank_frame(height=220, width=260)
    orange_bgr = _bgr_from_hsv(15, 220, 220)

    frame[40:180, 30:130] = orange_bgr
    frame[50:105, 180:220] = orange_bgr

    result = detect_dlz_orange_barrier_mask(frame, _default_config())

    expected_mask = _rect_mask(frame.shape[0], frame.shape[1], 30, 40, 130, 180)
    assert result.plausible is True
    assert result.passthrough is False
    assert result.reason == "masked"
    assert result.seed_points == ((0, 0), (frame.shape[1] - 1, 0))
    assert result.barrier_component_count == 2
    assert result.candidate_region_count == 1
    assert result.used_hull_merge is False
    assert result.component_bbox == (30, 40, 100, 140)
    assert np.array_equal(result.applied_mask, expected_mask)


def test_detect_dlz_orange_barrier_hulls_multiple_surviving_contours():
    frame = _blank_frame(height=220, width=240)
    orange_bgr = _bgr_from_hsv(15, 220, 220)

    frame[40:160, 20:80] = orange_bgr
    frame[40:160, 140:200] = orange_bgr

    result = detect_dlz_orange_barrier_mask(frame, _default_config())

    expected_mask = _rect_mask(frame.shape[0], frame.shape[1], 20, 40, 200, 160)
    assert result.plausible is True
    assert result.passthrough is False
    assert result.reason == "masked"
    assert result.used_hull_merge is True
    assert result.barrier_component_count == 2
    assert result.candidate_region_count == 2
    assert result.component_bbox == (20, 40, 180, 120)
    assert np.array_equal(result.applied_mask, expected_mask)


def test_detect_dlz_orange_barrier_ignores_legacy_seed_and_plausibility_guards():
    frame = _blank_frame(height=220, width=260)
    orange_bgr = _bgr_from_hsv(15, 220, 220)
    purple_bgr = _bgr_from_hsv(150, 180, 220)

    frame[40:180, 30:210] = orange_bgr
    frame[:48, :48] = purple_bgr
    frame[:48, -48:] = purple_bgr

    result = detect_dlz_orange_barrier_mask(
        frame,
        DLZOrangeBarrierMaskConfig(
            **{
                **_default_config().__dict__,
                "seed_reject_hsv": (((0, 0, 0), (180, 255, 255)),),
                "seed_patch_size_px": 64,
                "seed_barrier_ratio_threshold": 0.0,
                "seed_reject_ratio_threshold": 0.0,
                "min_orange_pixels": 999999,
                "min_component_pixels": 999999,
                "min_mask_area_frac": 0.90,
                "max_mask_area_frac": 0.05,
                "min_bbox_span_ratio": 0.99,
                "morphology_kernel_size": 3,
                "dilate_iterations": 7,
            }
        ),
    )

    expected_mask = _rect_mask(frame.shape[0], frame.shape[1], 30, 40, 210, 180)
    assert result.plausible is True
    assert result.passthrough is False
    assert result.reason == "masked"
    assert result.seed_points == ((0, 0), (frame.shape[1] - 1, 0))
    assert result.barrier_component_count == 1
    assert result.candidate_region_count == 1
    assert result.used_hull_merge is False
    assert result.component_bbox == (30, 40, 180, 140)
    assert np.array_equal(result.applied_mask, expected_mask)


def test_detect_dlz_orange_barrier_keeps_far_apart_regions_when_they_survive():
    frame = _blank_frame(height=240, width=320)
    orange_bgr = _bgr_from_hsv(15, 220, 220)

    frame[60:180, 20:80] = orange_bgr
    frame[60:180, 220:280] = orange_bgr

    result = detect_dlz_orange_barrier_mask(frame, _default_config())

    expected_mask = _rect_mask(frame.shape[0], frame.shape[1], 20, 60, 280, 180)
    assert result.plausible is True
    assert result.passthrough is False
    assert result.reason == "masked"
    assert result.used_hull_merge is True
    assert result.barrier_component_count == 2
    assert result.candidate_region_count == 2
    assert result.component_bbox == (20, 60, 260, 120)
    assert np.array_equal(result.applied_mask, expected_mask)
