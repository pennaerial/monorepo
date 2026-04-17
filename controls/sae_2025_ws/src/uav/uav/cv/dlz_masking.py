from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np
from typing_extensions import TypedDict

HSVTriplet = tuple[int, int, int]
HSVRange = tuple[HSVTriplet, HSVTriplet]
_MAX_HULL_EXPANSION_RATIO = 1.35


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


def _normalized_kernel_size(kernel_size: int) -> int:
    size = max(1, int(kernel_size))
    if size % 2 == 0:
        size += 1
    return size


def _clean_barrier_mask(mask: np.ndarray, kernel_size: int) -> np.ndarray:
    size = _normalized_kernel_size(kernel_size)
    if size <= 1:
        return mask
    kernel = np.ones((size, size), dtype=np.uint8)
    cleaned = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)


def _corner_patch(mask: np.ndarray, corner: str, patch_size: int) -> np.ndarray:
    height, width = mask.shape[:2]
    patch = max(1, min(int(patch_size), height, width))
    if corner == "top_left":
        return mask[:patch, :patch]
    if corner == "top_right":
        return mask[:patch, width - patch :]
    if corner == "bottom_left":
        return mask[height - patch :, :patch]
    if corner == "bottom_right":
        return mask[height - patch :, width - patch :]
    raise ValueError(f"Unsupported corner name {corner!r}.")


def _corner_seed_points(width: int, height: int) -> tuple[tuple[str, tuple[int, int]], ...]:
    return (
        ("top_left", (0, 0)),
        ("top_right", (width - 1, 0)),
        ("bottom_left", (0, height - 1)),
        ("bottom_right", (width - 1, height - 1)),
    )


def _valid_seed_points(
    barrier_mask: np.ndarray,
    reject_mask: np.ndarray | None,
    config: DLZOrangeBarrierMaskConfig,
) -> tuple[tuple[int, int], ...]:
    height, width = barrier_mask.shape[:2]
    seed_points: list[tuple[int, int]] = []
    patch_area = float(
        max(
            1,
            min(int(config.seed_patch_size_px), height, width)
            * min(int(config.seed_patch_size_px), height, width),
        )
    )
    for corner_name, seed_point in _corner_seed_points(width, height):
        barrier_patch = _corner_patch(
            barrier_mask, corner_name, config.seed_patch_size_px
        )
        barrier_ratio = float(np.count_nonzero(barrier_patch)) / patch_area
        if barrier_ratio >= config.seed_barrier_ratio_threshold:
            continue
        if reject_mask is not None:
            reject_patch = _corner_patch(
                reject_mask, corner_name, config.seed_patch_size_px
            )
            reject_ratio = float(np.count_nonzero(reject_patch)) / patch_area
            if reject_ratio >= config.seed_reject_ratio_threshold:
                continue
        seed_points.append(seed_point)
    return tuple(seed_points)


def _filter_components_by_area(
    mask: np.ndarray, min_pixels: int
) -> tuple[np.ndarray, int, int]:
    binary_mask = (mask > 0).astype(np.uint8)
    if not np.any(binary_mask):
        return np.zeros_like(mask), 0, 0

    label_count, labels, stats, _ = cv2.connectedComponentsWithStats(binary_mask, 8)
    filtered = np.zeros_like(mask)
    kept_components = 0
    kept_pixels = 0

    for label in range(1, label_count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < min_pixels:
            continue
        filtered[labels == label] = 255
        kept_components += 1
        kept_pixels += area

    return filtered, kept_components, kept_pixels


def _mask_bbox(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    points = cv2.findNonZero(mask)
    if points is None:
        return None
    return tuple(int(value) for value in cv2.boundingRect(points))


def _merge_candidate_regions(
    mask: np.ndarray,
) -> tuple[np.ndarray, tuple[int, int, int, int] | None, int, bool, float]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return np.zeros_like(mask), None, 0, False, 0.0

    if len(contours) == 1:
        bbox = tuple(int(value) for value in cv2.boundingRect(contours[0]))
        area_px = int(np.count_nonzero(mask))
        return mask, bbox, 1, False, 1.0

    union_area_px = float(max(1, np.count_nonzero(mask)))
    hull_points = cv2.convexHull(np.vstack(contours))
    hull_mask = np.zeros_like(mask)
    cv2.drawContours(hull_mask, [hull_points], -1, 255, cv2.FILLED)
    hull_area_px = float(np.count_nonzero(hull_mask))
    expansion_ratio = hull_area_px / union_area_px
    bbox = tuple(int(value) for value in cv2.boundingRect(hull_points))
    return hull_mask, bbox, len(contours), True, expansion_ratio


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

    Orange is treated as the flood-fill barrier. Corner patches can be rejected
    as seed candidates if they already contain orange or configured interior
    colours, which avoids starting the outside flood fill from inside the DLZ.
    """

    if not isinstance(bgr, np.ndarray):
        raise TypeError("bgr must be a numpy ndarray.")
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("bgr must be an HxWx3 BGR image.")

    height, width = bgr.shape[:2]
    frame_area = float(max(1, height * width))
    empty_mask = np.zeros((height, width), dtype=np.uint8)

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    raw_barrier_mask = _clean_barrier_mask(
        _mask_for_ranges(hsv, config.orange_barrier_hsv),
        config.morphology_kernel_size,
    )
    barrier_mask, barrier_component_count, orange_pixels = _filter_components_by_area(
        raw_barrier_mask,
        max(1, int(config.min_component_pixels)),
    )

    if orange_pixels < max(0, int(config.min_orange_pixels)):
        return _passthrough_result(
            bgr=bgr,
            barrier_mask=barrier_mask,
            outside_fill_mask=empty_mask,
            reason="orange_too_small",
            component_bbox=None,
            mask_area_px=0,
            orange_pixels=orange_pixels,
            seed_points=(),
            barrier_component_count=barrier_component_count,
            candidate_region_count=0,
        )

    reject_mask = (
        _mask_for_ranges(hsv, config.seed_reject_hsv)
        if config.seed_reject_hsv
        else None
    )
    seed_points = _valid_seed_points(barrier_mask, reject_mask, config)
    if not seed_points:
        return _passthrough_result(
            bgr=bgr,
            barrier_mask=barrier_mask,
            outside_fill_mask=empty_mask,
            reason="no_seed_corners",
            component_bbox=None,
            mask_area_px=0,
            orange_pixels=orange_pixels,
            seed_points=(),
            barrier_component_count=barrier_component_count,
            candidate_region_count=0,
        )

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

    raw_candidate_mask = cv2.bitwise_not(outside_fill_mask)
    candidate_mask, candidate_region_count, _ = _filter_components_by_area(
        raw_candidate_mask,
        max(1, int(config.min_component_pixels)),
    )
    if not np.any(candidate_mask):
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

    applied_mask, component_bbox, candidate_region_count, used_hull_merge, expansion_ratio = (
        _merge_candidate_regions(candidate_mask)
    )
    if component_bbox is None:
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
            candidate_region_count=candidate_region_count,
            used_hull_merge=used_hull_merge,
        )

    if used_hull_merge and expansion_ratio > _MAX_HULL_EXPANSION_RATIO:
        return _passthrough_result(
            bgr=bgr,
            barrier_mask=barrier_mask,
            outside_fill_mask=outside_fill_mask,
            reason="hull_too_large",
            component_bbox=component_bbox,
            mask_area_px=int(np.count_nonzero(candidate_mask)),
            orange_pixels=orange_pixels,
            seed_points=seed_points,
            barrier_component_count=barrier_component_count,
            candidate_region_count=candidate_region_count,
            used_hull_merge=True,
        )

    mask_area_px = int(np.count_nonzero(applied_mask))
    mask_area_frac = mask_area_px / frame_area
    if mask_area_frac < config.min_mask_area_frac:
        return _passthrough_result(
            bgr=bgr,
            barrier_mask=barrier_mask,
            outside_fill_mask=outside_fill_mask,
            reason="dlz_too_small",
            component_bbox=component_bbox,
            mask_area_px=mask_area_px,
            orange_pixels=orange_pixels,
            seed_points=seed_points,
            barrier_component_count=barrier_component_count,
            candidate_region_count=candidate_region_count,
            used_hull_merge=used_hull_merge,
        )
    if mask_area_frac > config.max_mask_area_frac:
        return _passthrough_result(
            bgr=bgr,
            barrier_mask=barrier_mask,
            outside_fill_mask=outside_fill_mask,
            reason="dlz_too_large",
            component_bbox=component_bbox,
            mask_area_px=mask_area_px,
            orange_pixels=orange_pixels,
            seed_points=seed_points,
            barrier_component_count=barrier_component_count,
            candidate_region_count=candidate_region_count,
            used_hull_merge=used_hull_merge,
        )

    bbox_width = component_bbox[2]
    bbox_height = component_bbox[3]
    bbox_span_ratio = max(bbox_width / max(width, 1), bbox_height / max(height, 1))
    if bbox_span_ratio < config.min_bbox_span_ratio:
        return _passthrough_result(
            bgr=bgr,
            barrier_mask=barrier_mask,
            outside_fill_mask=outside_fill_mask,
            reason="dlz_span_too_small",
            component_bbox=component_bbox,
            mask_area_px=mask_area_px,
            orange_pixels=orange_pixels,
            seed_points=seed_points,
            barrier_component_count=barrier_component_count,
            candidate_region_count=candidate_region_count,
            used_hull_merge=used_hull_merge,
        )

    if config.dilate_iterations > 0:
        kernel_size = _normalized_kernel_size(config.morphology_kernel_size)
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        applied_mask = cv2.dilate(
            applied_mask,
            kernel,
            iterations=max(0, int(config.dilate_iterations)),
        )
        component_bbox = _mask_bbox(applied_mask)
        mask_area_px = int(np.count_nonzero(applied_mask))

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
        candidate_region_count=candidate_region_count,
        used_hull_merge=used_hull_merge,
    )


def mask_dlz_orange_barrier(
    bgr: np.ndarray,
    config: DLZOrangeBarrierMaskConfig = DLZOrangeBarrierMaskConfig(),
) -> np.ndarray:
    return detect_dlz_orange_barrier_mask(bgr, config=config).frame


def preprocess_dlz_orange_barrier(frame: np.ndarray) -> np.ndarray:
    return mask_dlz_orange_barrier(frame)
