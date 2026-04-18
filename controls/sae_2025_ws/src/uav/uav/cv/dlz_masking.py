from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np
from typing_extensions import TypedDict

HSVTriplet = tuple[int, int, int]
HSVRange = tuple[HSVTriplet, HSVTriplet]
_INTERIOR_KERNEL_SIZE = 15
_MIN_SURVIVING_CONTOUR_AREA = 5000


class HSVRangeConfig(TypedDict):
    lower: list[int]
    upper: list[int]


DEFAULT_ORANGE_BARRIER_HSV: tuple[HSVRange, ...] = (((5, 78, 158), (25, 189, 255)),)


@dataclass(frozen=True)
class DLZOrangeBarrierMaskConfig:
    orange_barrier_hsv: tuple[HSVRange, ...] = DEFAULT_ORANGE_BARRIER_HSV
    seed_reject_hsv: tuple[HSVRange, ...] = ()
    seed_patch_size_px: int = 48
    seed_barrier_ratio_threshold: float = 0.08
    seed_reject_ratio_threshold: float = 0.08
    min_orange_pixels: int = 400
    min_component_pixels: int = 200
    min_mask_area_frac: float = 0.05
    max_mask_area_frac: float = 0.98
    min_bbox_span_ratio: float = 0.45
    morphology_kernel_size: int = 5
    dilate_iterations: int = 1
    masked_pixel_bgr: tuple[int, int, int] = (0, 0, 0)


@dataclass(frozen=True)
class DLZOrangeBarrierMaskResult:
    frame: np.ndarray
    applied_mask: np.ndarray
    barrier_mask: np.ndarray
    outside_fill_mask: np.ndarray
    plausible: bool
    passthrough: bool
    reason: str
    component_bbox: tuple[int, int, int, int] | None
    mask_area_px: int
    orange_pixels: int
    seed_points: tuple[tuple[int, int], ...]
    barrier_component_count: int
    candidate_region_count: int
    used_hull_merge: bool


def _as_hsv_triplet(values: Sequence[int], *, label: str) -> np.ndarray:
    if len(values) != 3:
        raise ValueError(f"{label} must contain exactly 3 HSV values.")
    return np.array([int(v) for v in values], dtype=np.uint8)


def coerce_hsv_range_configs(configs: Sequence[HSVRangeConfig]) -> tuple[HSVRange, ...]:
    ranges: list[HSVRange] = []
    for index, config in enumerate(configs):
        ranges.append(
            (
                tuple(int(value) for value in config["lower"]),
                tuple(int(value) for value in config["upper"]),
            )
        )
        if len(ranges[-1][0]) != 3 or len(ranges[-1][1]) != 3:
            raise ValueError(
                f"HSV range entry {index} must provide exactly 3 lower and 3 upper values."
            )
    return tuple(ranges)


def _mask_for_ranges(hsv: np.ndarray, ranges: Sequence[HSVRange]) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        mask = cv2.bitwise_or(
            mask,
            cv2.inRange(
                hsv,
                _as_hsv_triplet(lower, label="lower_hsv"),
                _as_hsv_triplet(upper, label="upper_hsv"),
            ),
        )
    return mask


def _corner_seed_points(width: int) -> tuple[tuple[int, int], ...]:
    return ((0, 0), (width - 1, 0))


def _barrier_component_count(mask: np.ndarray) -> int:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return len(contours)


def _passthrough_result(
    *,
    bgr: np.ndarray,
    barrier_mask: np.ndarray,
    outside_fill_mask: np.ndarray,
    reason: str,
    component_bbox: tuple[int, int, int, int] | None,
    mask_area_px: int,
    orange_pixels: int,
    seed_points: tuple[tuple[int, int], ...],
    barrier_component_count: int,
    candidate_region_count: int,
    used_hull_merge: bool = False,
) -> DLZOrangeBarrierMaskResult:
    return DLZOrangeBarrierMaskResult(
        frame=bgr,
        applied_mask=np.zeros(bgr.shape[:2], dtype=np.uint8),
        barrier_mask=barrier_mask,
        outside_fill_mask=outside_fill_mask,
        plausible=False,
        passthrough=True,
        reason=reason,
        component_bbox=component_bbox,
        mask_area_px=mask_area_px,
        orange_pixels=orange_pixels,
        seed_points=seed_points,
        barrier_component_count=barrier_component_count,
        candidate_region_count=candidate_region_count,
        used_hull_merge=used_hull_merge,
    )


def detect_dlz_orange_barrier_mask(
    bgr: np.ndarray,
    config: DLZOrangeBarrierMaskConfig = DLZOrangeBarrierMaskConfig(),
) -> DLZOrangeBarrierMaskResult:
    """Mask everything outside the orange DLZ while preserving frame geometry.

    Internally this follows the literal flood-fill hull algorithm:
    threshold orange, flood fill from the two top corners, invert to get the
    interior, erode+dilate with a fixed 15x15 kernel, then convex-hull every
    surviving contour whose area is at least 5000 pixels.

    For compatibility, the public config and result shapes are preserved. Under
    this literal implementation only ``orange_barrier_hsv`` and
    ``masked_pixel_bgr`` materially affect the output.
    """

    if not isinstance(bgr, np.ndarray):
        raise TypeError("bgr must be a numpy ndarray.")
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("bgr must be an HxWx3 BGR image.")

    height, width = bgr.shape[:2]
    if height <= 0 or width <= 0:
        raise ValueError("bgr must be non-empty.")

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    barrier_mask = _mask_for_ranges(hsv, config.orange_barrier_hsv)
    orange_pixels = int(np.count_nonzero(barrier_mask))
    barrier_component_count = _barrier_component_count(barrier_mask)
    seed_points = _corner_seed_points(width)

    flood_fill_mask = np.zeros((height + 2, width + 2), dtype=np.uint8)
    flood_fill_mask[1:-1, 1:-1] = barrier_mask
    outside_fill_mask = np.zeros((height, width), dtype=np.uint8)
    for seed_point in seed_points:
        cv2.floodFill(
            outside_fill_mask,
            flood_fill_mask.copy(),
            seedPoint=seed_point,
            newVal=255,
        )

    interior_mask = cv2.bitwise_not(outside_fill_mask)
    kernel = np.ones((_INTERIOR_KERNEL_SIZE, _INTERIOR_KERNEL_SIZE), dtype=np.uint8)
    interior_mask = cv2.erode(interior_mask, kernel)
    interior_mask = cv2.dilate(interior_mask, kernel)

    contours, _ = cv2.findContours(
        interior_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    surviving_contours = [
        contour
        for contour in contours
        if cv2.contourArea(contour) >= _MIN_SURVIVING_CONTOUR_AREA
    ]

    if not surviving_contours:
        return _passthrough_result(
            bgr=bgr,
            barrier_mask=barrier_mask,
            outside_fill_mask=outside_fill_mask,
            reason="no_dlz_region",
            component_bbox=None,
            mask_area_px=0,
            orange_pixels=orange_pixels,
            seed_points=seed_points,
            barrier_component_count=barrier_component_count,
            candidate_region_count=0,
        )

    hull = cv2.convexHull(np.concatenate(surviving_contours, axis=0))
    applied_mask = np.zeros((height, width), dtype=np.uint8)
    cv2.drawContours(applied_mask, [hull], -1, 255, cv2.FILLED)

    component_bbox = tuple(int(value) for value in cv2.boundingRect(hull))
    mask_area_px = int(np.count_nonzero(applied_mask))
    used_hull_merge = len(surviving_contours) > 1
    masked_frame = np.full_like(bgr, config.masked_pixel_bgr, dtype=bgr.dtype)
    cv2.copyTo(bgr, applied_mask, masked_frame)

    return DLZOrangeBarrierMaskResult(
        frame=masked_frame,
        applied_mask=applied_mask,
        barrier_mask=barrier_mask,
        outside_fill_mask=outside_fill_mask,
        plausible=True,
        passthrough=False,
        reason="masked",
        component_bbox=component_bbox,
        mask_area_px=mask_area_px,
        orange_pixels=orange_pixels,
        seed_points=seed_points,
        barrier_component_count=barrier_component_count,
        candidate_region_count=len(surviving_contours),
        used_hull_merge=used_hull_merge,
    )


def mask_dlz_orange_barrier(
    bgr: np.ndarray,
    config: DLZOrangeBarrierMaskConfig = DLZOrangeBarrierMaskConfig(),
) -> np.ndarray:
    return detect_dlz_orange_barrier_mask(bgr, config=config).frame


def preprocess_dlz_orange_barrier(frame: np.ndarray) -> np.ndarray:
    return mask_dlz_orange_barrier(frame)
