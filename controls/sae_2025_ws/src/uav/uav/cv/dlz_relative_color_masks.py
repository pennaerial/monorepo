from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import cv2
import numpy as np

ORANGE_LOWER = np.array([5, 78, 158], dtype=np.uint8)
ORANGE_UPPER = np.array([25, 189, 255], dtype=np.uint8)

# All colour thresholds are relative to the frame's orange DLZ colour in Lab.
CANDIDATE_DAB_MIN = 8
CANDIDATE_S_MIN = 25
CANDIDATE_S_MAX = 130
CANDIDATE_V_MIN = 70

GREEN_DA_MAX = -34

PURPLE_DB_MAX = -20
PURPLE_KEEP_MEAN_H_MIN = 140
PURPLE_KEEP_MEAN_DB_MAX = -44
PURPLE_KEEP_DMAX_MIN = 10

BLUE_STRONG_H_MIN = 95
BLUE_STRONG_H_MAX = 130
BLUE_STRONG_DB_MAX = -25
BLUE_STRONG_MIN_AREA = 12
BLUE_STRONG_DMAX_MIN = 2

WEAK_BLUE_MIN_AREA = 80
WEAK_BLUE_MAX_COMPONENT_H = 40
WEAK_BLUE_MEAN_DA_MAX = -20
WEAK_BLUE_MEAN_DB_MAX = -8

WEAK_PURPLE_MIN_AREA = 40
WEAK_PURPLE_MAX_COMPONENT_H = 20
WEAK_PURPLE_MIN_X_FRAC = 0.45
WEAK_PURPLE_MEAN_DB_MAX = -12
WEAK_PURPLE_MEAN_DA_MIN = -20


@dataclass(frozen=True)
class DLZRelativeColorMaskConfig:
    candidate_dab_min: int = CANDIDATE_DAB_MIN
    candidate_s_min: int = CANDIDATE_S_MIN
    candidate_s_max: int = CANDIDATE_S_MAX
    candidate_v_min: int = CANDIDATE_V_MIN
    green_da_max: int = GREEN_DA_MAX
    purple_db_max: int = PURPLE_DB_MAX
    purple_keep_mean_h_min: int = PURPLE_KEEP_MEAN_H_MIN
    purple_keep_mean_db_max: int = PURPLE_KEEP_MEAN_DB_MAX
    purple_keep_dmax_min: int = PURPLE_KEEP_DMAX_MIN
    blue_strong_h_min: int = BLUE_STRONG_H_MIN
    blue_strong_h_max: int = BLUE_STRONG_H_MAX
    blue_strong_db_max: int = BLUE_STRONG_DB_MAX
    blue_strong_min_area: int = BLUE_STRONG_MIN_AREA
    blue_strong_dmax_min: int = BLUE_STRONG_DMAX_MIN
    weak_blue_min_area: int = WEAK_BLUE_MIN_AREA
    weak_blue_max_component_h: int = WEAK_BLUE_MAX_COMPONENT_H
    weak_blue_mean_da_max: int = WEAK_BLUE_MEAN_DA_MAX
    weak_blue_mean_db_max: int = WEAK_BLUE_MEAN_DB_MAX
    weak_purple_min_area: int = WEAK_PURPLE_MIN_AREA
    weak_purple_max_component_h: int = WEAK_PURPLE_MAX_COMPONENT_H
    weak_purple_min_x_frac: float = WEAK_PURPLE_MIN_X_FRAC
    weak_purple_mean_db_max: int = WEAK_PURPLE_MEAN_DB_MAX
    weak_purple_mean_da_min: int = WEAK_PURPLE_MEAN_DA_MIN
    orange_lower_hsv: tuple[int, int, int] = (5, 78, 158)
    orange_upper_hsv: tuple[int, int, int] = (25, 189, 255)


@dataclass(frozen=True)
class DLZRelativeColorComponent:
    area: int
    x: int
    y: int
    w: int
    h: int
    mean_h: float
    mean_s: float
    mean_da: float
    mean_db: float
    dmax: float


@dataclass(frozen=True)
class DLZRelativeColorMaskResult:
    dlz_mask: np.ndarray
    orange_mask: np.ndarray
    orange_roi_mask: np.ndarray
    candidate_mask: np.ndarray
    mask_green: np.ndarray
    mask_blue: np.ndarray
    mask_purple: np.ndarray
    green_raw_mask: np.ndarray
    purple_raw_mask: np.ndarray
    blue_strong_raw_mask: np.ndarray
    blue_strong_mask: np.ndarray
    residual_mask: np.ndarray
    residual2_mask: np.ndarray
    weak_blue_mask: np.ndarray
    weak_purple_mask: np.ndarray
    green_components: tuple[DLZRelativeColorComponent, ...]
    purple_components: tuple[DLZRelativeColorComponent, ...]
    blue_strong_components: tuple[DLZRelativeColorComponent, ...]
    weak_blue_components: tuple[DLZRelativeColorComponent, ...]
    weak_purple_components: tuple[DLZRelativeColorComponent, ...]
    orange_a0: int
    orange_b0: int


def _empty_result(shape: tuple[int, int]) -> DLZRelativeColorMaskResult:
    zeros = np.zeros(shape, dtype=np.uint8)
    return DLZRelativeColorMaskResult(
        dlz_mask=zeros.copy(),
        orange_mask=zeros.copy(),
        orange_roi_mask=zeros.copy(),
        candidate_mask=zeros.copy(),
        mask_green=zeros.copy(),
        mask_blue=zeros.copy(),
        mask_purple=zeros.copy(),
        green_raw_mask=zeros.copy(),
        purple_raw_mask=zeros.copy(),
        blue_strong_raw_mask=zeros.copy(),
        blue_strong_mask=zeros.copy(),
        residual_mask=zeros.copy(),
        residual2_mask=zeros.copy(),
        weak_blue_mask=zeros.copy(),
        weak_purple_mask=zeros.copy(),
        green_components=(),
        purple_components=(),
        blue_strong_components=(),
        weak_blue_components=(),
        weak_purple_components=(),
        orange_a0=0,
        orange_b0=0,
    )


def _component_features(
    *,
    area: int,
    x: int,
    y: int,
    width: int,
    height: int,
    hue: np.ndarray,
    saturation: np.ndarray,
    delta_a: np.ndarray,
    delta_b: np.ndarray,
    dist_to_boundary: np.ndarray,
    component: np.ndarray,
) -> DLZRelativeColorComponent:
    return DLZRelativeColorComponent(
        area=int(area),
        x=int(x),
        y=int(y),
        w=int(width),
        h=int(height),
        mean_h=float(hue[component].mean()),
        mean_s=float(saturation[component].mean()),
        mean_da=float(delta_a[component].mean()),
        mean_db=float(delta_b[component].mean()),
        dmax=float(dist_to_boundary[component].max()),
    )


def _filter_components(
    mask: np.ndarray,
    hue: np.ndarray,
    saturation: np.ndarray,
    delta_a: np.ndarray,
    delta_b: np.ndarray,
    dist_to_boundary: np.ndarray,
    keep_fn: Callable[[DLZRelativeColorComponent], bool],
) -> tuple[np.ndarray, tuple[DLZRelativeColorComponent, ...]]:
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    kept_mask = np.zeros_like(mask)
    kept_features: list[DLZRelativeColorComponent] = []

    for label in range(1, num_labels):
        component = labels == label
        x, y, width, height, area = stats[label]
        features = _component_features(
            area=int(area),
            x=int(x),
            y=int(y),
            width=int(width),
            height=int(height),
            hue=hue,
            saturation=saturation,
            delta_a=delta_a,
            delta_b=delta_b,
            dist_to_boundary=dist_to_boundary,
            component=component,
        )
        if keep_fn(features):
            kept_mask[component] = 255
            kept_features.append(features)

    return kept_mask, tuple(kept_features)


def detect_dlz_relative_color_masks(
    frame: np.ndarray,
    *,
    dlz_mask: np.ndarray | None = None,
    config: DLZRelativeColorMaskConfig = DLZRelativeColorMaskConfig(),
) -> DLZRelativeColorMaskResult:
    if not isinstance(frame, np.ndarray):
        raise TypeError("frame must be a numpy.ndarray.")
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must have shape (height, width, 3).")
    if frame.size == 0:
        return _empty_result(frame.shape[:2])

    height, width = frame.shape[:2]
    if dlz_mask is None:
        dlz_mask_u8 = np.full((height, width), 255, dtype=np.uint8)
    else:
        if not isinstance(dlz_mask, np.ndarray):
            raise TypeError("dlz_mask must be a numpy.ndarray.")
        if dlz_mask.shape != frame.shape[:2]:
            raise ValueError("dlz_mask must match the frame height/width.")
        dlz_mask_u8 = np.where(dlz_mask > 0, 255, 0).astype(np.uint8)

    if not np.any(dlz_mask_u8):
        return _empty_result(frame.shape[:2])

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    hue = hsv[:, :, 0]
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    lab_a = lab[:, :, 1].astype(np.int16)
    lab_b = lab[:, :, 2].astype(np.int16)

    orange_mask = cv2.inRange(
        hsv,
        np.array(config.orange_lower_hsv, dtype=np.uint8),
        np.array(config.orange_upper_hsv, dtype=np.uint8),
    )
    orange_roi = (orange_mask > 0) & (dlz_mask_u8 > 0)
    if np.any(orange_roi):
        orange_a = int(np.median(lab_a[orange_roi]))
        orange_b = int(np.median(lab_b[orange_roi]))
    else:
        orange_a = int(np.median(lab_a[dlz_mask_u8 > 0]))
        orange_b = int(np.median(lab_b[dlz_mask_u8 > 0]))

    delta_a = lab_a - orange_a
    delta_b = lab_b - orange_b
    delta_ab = np.sqrt(delta_a.astype(np.float32) ** 2 + delta_b.astype(np.float32) ** 2)

    candidate = (
        (dlz_mask_u8 > 0)
        & (delta_ab > float(config.candidate_dab_min))
        & (saturation >= int(config.candidate_s_min))
        & (saturation <= int(config.candidate_s_max))
        & (value > int(config.candidate_v_min))
    ).astype(np.uint8) * 255

    dist_to_boundary = cv2.distanceTransform(dlz_mask_u8, cv2.DIST_L2, 5)

    green_raw = ((candidate > 0) & (delta_a <= int(config.green_da_max))).astype(np.uint8) * 255
    green_mask, green_components = _filter_components(
        green_raw,
        hue,
        saturation,
        delta_a,
        delta_b,
        dist_to_boundary,
        lambda features: features.area >= 150 and features.dmax >= 5,
    )

    red_channel = frame[:, :, 2]
    green_channel = frame[:, :, 1]
    blue_channel = frame[:, :, 0]

    purple_raw = (
        (candidate > 0)
        & (delta_b <= int(config.purple_db_max))
        & (
            (hue >= 145)
            | ((hue <= 8) & (red_channel > green_channel) & (red_channel >= (blue_channel - 10)))
        )
    ).astype(np.uint8) * 255
    purple_mask, purple_components = _filter_components(
        purple_raw,
        hue,
        saturation,
        delta_a,
        delta_b,
        dist_to_boundary,
        lambda features: features.dmax >= float(config.purple_keep_dmax_min)
        and (
            features.mean_h >= float(config.purple_keep_mean_h_min)
            or features.mean_db <= float(config.purple_keep_mean_db_max)
        ),
    )

    blue_strong_raw = (
        (candidate > 0)
        & (hue >= int(config.blue_strong_h_min))
        & (hue <= int(config.blue_strong_h_max))
        & (delta_b <= int(config.blue_strong_db_max))
    ).astype(np.uint8) * 255
    blue_strong_mask, blue_strong_components = _filter_components(
        blue_strong_raw,
        hue,
        saturation,
        delta_a,
        delta_b,
        dist_to_boundary,
        lambda features: features.area >= int(config.blue_strong_min_area)
        and features.dmax >= float(config.blue_strong_dmax_min),
    )

    frame_has_green = np.count_nonzero(green_mask) > 0
    frame_has_purple = np.count_nonzero(purple_mask) > 0
    frame_has_blue_strong = np.count_nonzero(blue_strong_mask) > 0

    residual = (
        (candidate > 0)
        & (green_mask == 0)
        & (purple_mask == 0)
        & (blue_strong_mask == 0)
    ).astype(np.uint8) * 255

    weak_blue_mask, weak_blue_components = _filter_components(
        residual,
        hue,
        saturation,
        delta_a,
        delta_b,
        dist_to_boundary,
        lambda features: (frame_has_green or frame_has_blue_strong)
        and features.area >= int(config.weak_blue_min_area)
        and features.h <= int(config.weak_blue_max_component_h)
        and features.mean_da <= float(config.weak_blue_mean_da_max)
        and features.mean_db <= float(config.weak_blue_mean_db_max),
    )

    residual2 = ((residual > 0) & (weak_blue_mask == 0)).astype(np.uint8) * 255
    weak_purple_mask, weak_purple_components = _filter_components(
        residual2,
        hue,
        saturation,
        delta_a,
        delta_b,
        dist_to_boundary,
        lambda features: frame_has_green
        and (not frame_has_purple)
        and features.area >= int(config.weak_purple_min_area)
        and features.h <= int(config.weak_purple_max_component_h)
        and features.x >= int(float(config.weak_purple_min_x_frac) * width)
        and features.mean_db <= float(config.weak_purple_mean_db_max)
        and features.mean_da > float(config.weak_purple_mean_da_min),
    )

    blue_mask = cv2.bitwise_or(blue_strong_mask, weak_blue_mask)
    purple_mask = cv2.bitwise_or(purple_mask, weak_purple_mask)

    thin_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    blue_mask = cv2.bitwise_and(cv2.dilate(blue_mask, thin_kernel, iterations=1), dlz_mask_u8)
    purple_mask = cv2.bitwise_and(
        cv2.dilate(purple_mask, thin_kernel, iterations=1),
        dlz_mask_u8,
    )
    green_mask = cv2.bitwise_and(green_mask, dlz_mask_u8)

    return DLZRelativeColorMaskResult(
        dlz_mask=dlz_mask_u8,
        orange_mask=orange_mask,
        orange_roi_mask=orange_roi.astype(np.uint8) * 255,
        candidate_mask=candidate,
        mask_green=green_mask,
        mask_blue=blue_mask,
        mask_purple=purple_mask,
        green_raw_mask=green_raw,
        purple_raw_mask=purple_raw,
        blue_strong_raw_mask=blue_strong_raw,
        blue_strong_mask=blue_strong_mask,
        residual_mask=residual,
        residual2_mask=residual2,
        weak_blue_mask=weak_blue_mask,
        weak_purple_mask=weak_purple_mask,
        green_components=green_components,
        purple_components=purple_components,
        blue_strong_components=blue_strong_components,
        weak_blue_components=weak_blue_components,
        weak_purple_components=weak_purple_components,
        orange_a0=orange_a,
        orange_b0=orange_b,
    )
