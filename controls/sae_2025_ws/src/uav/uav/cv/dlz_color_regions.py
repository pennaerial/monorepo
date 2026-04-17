from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np

BGRTriplet = tuple[int, int, int]
_EDGE_BLUR_KERNEL = (5, 5)
_EDGE_CANNY_LOW = 40
_EDGE_CANNY_HIGH = 120
_EDGE_KERNEL = np.ones((3, 3), dtype=np.uint8)


@dataclass(frozen=True)
class DLZLineColorMaskConfig:
    color_a_bgr: BGRTriplet
    color_b_bgr: BGRTriplet
    reject_colors_bgr: tuple[BGRTriplet, ...] = ()
    nonblack_min_value: int = 45
    nonblack_min_saturation: int = 35
    min_region_area_px: int = 20
    match_max_distance_lab: float = 38.0
    match_min_margin_lab: float = 8.0


@dataclass(frozen=True)
class DLZLineColorRegion:
    label: int
    area_px: int
    bbox: tuple[int, int, int, int]
    median_bgr: BGRTriplet
    median_lab: tuple[float, float, float]
    assigned_color: str
    best_distance_lab: float
    second_distance_lab: float
    reject_distance_lab: float | None


@dataclass(frozen=True)
class DLZLineColorMaskResult:
    mask_a: np.ndarray
    mask_b: np.ndarray
    mask_unknown: np.ndarray
    foreground_mask: np.ndarray
    edge_mask: np.ndarray
    split_region_mask: np.ndarray
    regions: tuple[DLZLineColorRegion, ...]


def coerce_bgr_triplet(values: Sequence[int], *, label: str) -> BGRTriplet:
    if len(values) != 3:
        raise ValueError(f"{label} must contain exactly 3 BGR values.")
    return tuple(int(value) for value in values)


def _lab_triplet_from_bgr(bgr: Sequence[int]) -> np.ndarray:
    triplet = np.array([[list(coerce_bgr_triplet(bgr, label="bgr"))]], dtype=np.uint8)
    return cv2.cvtColor(triplet, cv2.COLOR_BGR2LAB)[0, 0].astype(np.float32)


def _region_assignment(
    region_lab: np.ndarray,
    *,
    color_a_lab: np.ndarray,
    color_b_lab: np.ndarray,
    reject_labs: tuple[np.ndarray, ...],
    config: DLZLineColorMaskConfig,
) -> tuple[str, float, float, float | None]:
    distances = {
        "A": float(np.linalg.norm(region_lab - color_a_lab)),
        "B": float(np.linalg.norm(region_lab - color_b_lab)),
    }
    ordered = sorted(distances.items(), key=lambda item: item[1])
    assigned_color, best_distance = ordered[0]
    second_distance = ordered[1][1]
    reject_distance = (
        min(float(np.linalg.norm(region_lab - reject_lab)) for reject_lab in reject_labs)
        if reject_labs
        else None
    )

    if best_distance > config.match_max_distance_lab:
        return "unknown", best_distance, second_distance, reject_distance
    if (second_distance - best_distance) < config.match_min_margin_lab:
        return "unknown", best_distance, second_distance, reject_distance
    if reject_distance is not None and reject_distance < best_distance:
        return "unknown", best_distance, second_distance, reject_distance
    return assigned_color, best_distance, second_distance, reject_distance


def _filter_mask_components(mask: np.ndarray, min_area_px: int) -> np.ndarray:
    if min_area_px <= 1 or not np.any(mask):
        return mask

    filtered = np.zeros_like(mask)
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (mask > 0).astype(np.uint8), 8
    )
    for label in range(1, component_count):
        if int(stats[label, cv2.CC_STAT_AREA]) < min_area_px:
            continue
        filtered[labels == label] = 255
    return filtered


def detect_dlz_line_color_regions(
    bgr: np.ndarray,
    config: DLZLineColorMaskConfig,
) -> DLZLineColorMaskResult:
    if not isinstance(bgr, np.ndarray):
        raise TypeError("bgr must be a numpy.ndarray.")
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("bgr must have shape (height, width, 3).")

    empty_mask = np.zeros(bgr.shape[:2], dtype=np.uint8)
    if bgr.size == 0:
        return DLZLineColorMaskResult(
            mask_a=empty_mask.copy(),
            mask_b=empty_mask.copy(),
            mask_unknown=empty_mask.copy(),
            foreground_mask=empty_mask.copy(),
            edge_mask=empty_mask.copy(),
            split_region_mask=empty_mask.copy(),
            regions=(),
        )

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    foreground_mask = np.where(
        (hsv[:, :, 1] >= int(config.nonblack_min_saturation))
        & (hsv[:, :, 2] >= int(config.nonblack_min_value)),
        255,
        0,
    ).astype(np.uint8)
    if not np.any(foreground_mask):
        return DLZLineColorMaskResult(
            mask_a=empty_mask.copy(),
            mask_b=empty_mask.copy(),
            mask_unknown=empty_mask.copy(),
            foreground_mask=foreground_mask,
            edge_mask=empty_mask.copy(),
            split_region_mask=empty_mask.copy(),
            regions=(),
        )

    blurred_bgr = cv2.GaussianBlur(bgr, _EDGE_BLUR_KERNEL, 0)
    blurred_lab = cv2.cvtColor(blurred_bgr, cv2.COLOR_BGR2LAB)
    edge_mask = cv2.bitwise_or(
        cv2.Canny(blurred_lab[:, :, 1], _EDGE_CANNY_LOW, _EDGE_CANNY_HIGH),
        cv2.Canny(blurred_lab[:, :, 2], _EDGE_CANNY_LOW, _EDGE_CANNY_HIGH),
    )
    edge_mask = cv2.bitwise_and(edge_mask, foreground_mask)
    edge_mask = cv2.morphologyEx(edge_mask, cv2.MORPH_CLOSE, _EDGE_KERNEL)
    edge_mask = cv2.dilate(edge_mask, _EDGE_KERNEL, iterations=1)

    split_region_mask = cv2.bitwise_and(foreground_mask, cv2.bitwise_not(edge_mask))
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (split_region_mask > 0).astype(np.uint8), 8
    )

    mask_a = np.zeros_like(foreground_mask)
    mask_b = np.zeros_like(foreground_mask)
    original_lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    color_a_lab = _lab_triplet_from_bgr(config.color_a_bgr)
    color_b_lab = _lab_triplet_from_bgr(config.color_b_bgr)
    reject_labs = tuple(
        _lab_triplet_from_bgr(reject_color) for reject_color in config.reject_colors_bgr
    )
    regions: list[DLZLineColorRegion] = []

    for label in range(1, component_count):
        area_px = int(stats[label, cv2.CC_STAT_AREA])
        if area_px < int(config.min_region_area_px):
            continue

        region_mask = labels == label
        median_bgr = tuple(
            int(round(value)) for value in np.median(bgr[region_mask], axis=0)
        )
        median_lab_np = np.median(original_lab[region_mask], axis=0).astype(np.float32)
        assigned_color, best_distance, second_distance, reject_distance = (
            _region_assignment(
                median_lab_np,
                color_a_lab=color_a_lab,
                color_b_lab=color_b_lab,
                reject_labs=reject_labs,
                config=config,
            )
        )
        bbox = (
            int(stats[label, cv2.CC_STAT_LEFT]),
            int(stats[label, cv2.CC_STAT_TOP]),
            int(stats[label, cv2.CC_STAT_WIDTH]),
            int(stats[label, cv2.CC_STAT_HEIGHT]),
        )
        regions.append(
            DLZLineColorRegion(
                label=label,
                area_px=area_px,
                bbox=bbox,
                median_bgr=median_bgr,
                median_lab=tuple(float(value) for value in median_lab_np),
                assigned_color=assigned_color,
                best_distance_lab=best_distance,
                second_distance_lab=second_distance,
                reject_distance_lab=reject_distance,
            )
        )
        if assigned_color == "A":
            mask_a[region_mask] = 255
        elif assigned_color == "B":
            mask_b[region_mask] = 255

    distance_a = np.linalg.norm(original_lab - color_a_lab, axis=2)
    distance_b = np.linalg.norm(original_lab - color_b_lab, axis=2)
    if reject_labs:
        reject_distance = np.min(
            np.stack(
                [
                    np.linalg.norm(original_lab - reject_lab, axis=2)
                    for reject_lab in reject_labs
                ],
                axis=0,
            ),
            axis=0,
        )
    else:
        reject_distance = np.full(distance_a.shape, np.inf, dtype=np.float32)

    pixel_mask_a = np.where(
        (foreground_mask > 0)
        & (distance_a <= float(config.match_max_distance_lab))
        & ((distance_b - distance_a) >= float(config.match_min_margin_lab))
        & (reject_distance >= distance_a),
        255,
        0,
    ).astype(np.uint8)
    pixel_mask_b = np.where(
        (foreground_mask > 0)
        & (distance_b <= float(config.match_max_distance_lab))
        & ((distance_a - distance_b) >= float(config.match_min_margin_lab))
        & (reject_distance >= distance_b),
        255,
        0,
    ).astype(np.uint8)
    mask_a = cv2.bitwise_or(
        mask_a,
        _filter_mask_components(pixel_mask_a, int(config.min_region_area_px)),
    )
    mask_b = cv2.bitwise_or(
        mask_b,
        _filter_mask_components(pixel_mask_b, int(config.min_region_area_px)),
    )
    mask_unknown = cv2.bitwise_and(
        foreground_mask,
        cv2.bitwise_not(cv2.bitwise_or(mask_a, mask_b)),
    )
    return DLZLineColorMaskResult(
        mask_a=mask_a,
        mask_b=mask_b,
        mask_unknown=mask_unknown,
        foreground_mask=foreground_mask,
        edge_mask=edge_mask,
        split_region_mask=split_region_mask,
        regions=tuple(regions),
    )
