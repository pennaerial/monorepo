from __future__ import annotations

import cv2
import numpy as np


def detect_focused_dlz_paper_masks(
    focused_bgr: np.ndarray,
    *,
    background_max_value: int = 0,
) -> dict[str, np.ndarray]:
    """
    Split a background-masked DLZ image into candidate paper regions.

    ``focused_bgr`` must already have the DLZ isolated, with non-DLZ pixels
    blacked out.
    """
    if not isinstance(focused_bgr, np.ndarray):
        raise TypeError("focused_bgr must be a numpy.ndarray")
    if focused_bgr.ndim != 3 or focused_bgr.shape[2] != 3:
        raise ValueError("focused_bgr must have shape (H, W, 3)")

    h, w = focused_bgr.shape[:2]
    zeros = np.zeros((h, w), dtype=np.uint8)

    ORANGE_LOWER = np.array([5, 78, 158], dtype=np.uint8)
    ORANGE_UPPER = np.array([25, 189, 255], dtype=np.uint8)

    MIN_SAT = 20
    MIN_VAL = 45

    MIN_PRODUCT_GAP = 2500.0
    MIN_REGION_PRODUCT_GAP = 3500.0
    MAX_BLUE_PRODUCT = 16500.0

    MIN_DELTA_AXIS_STRENGTH = 10.0
    DELTA_AXIS_MARGIN = 4.0
    BLUE_DELTA_A_MAX = -20.0
    STRONG_PURPLE_DELTA_B_MAX = -40.0

    MIN_REGION_AREA = 80
    FINAL_MIN_AREA = 40

    dlz_mask = np.where(
        np.any(focused_bgr > int(background_max_value), axis=2),
        255,
        0,
    ).astype(np.uint8)

    if not np.any(dlz_mask):
        return {
            "dlz_mask": zeros.copy(),
            "candidate_mask": zeros.copy(),
            "proposal_mask": zeros.copy(),
            "green_mask": zeros.copy(),
            "blue_mask": zeros.copy(),
            "purple_mask": zeros.copy(),
            "unknown_mask": zeros.copy(),
        }

    hsv = cv2.cvtColor(focused_bgr, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(focused_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)

    lab_a = lab[:, :, 1].astype(np.int32)
    lab_b = lab[:, :, 2].astype(np.int32)

    orange_mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
    orange_roi = (orange_mask > 0) & (dlz_mask > 0)

    if np.any(orange_roi):
        orange_a = int(np.median(lab_a[orange_roi]))
        orange_b = int(np.median(lab_b[orange_roi]))
    else:
        orange_a = int(np.median(lab_a[dlz_mask > 0]))
        orange_b = int(np.median(lab_b[dlz_mask > 0]))

    delta_a = lab_a - orange_a
    delta_b = lab_b - orange_b

    lab_ab_product = lab_a * lab_b
    orange_product = orange_a * orange_b
    product_gap = orange_product - lab_ab_product

    axis_strength = np.maximum(-delta_a, -delta_b)

    candidate_mask = np.where(
        (dlz_mask > 0)
        & (hsv[:, :, 1] >= MIN_SAT)
        & (hsv[:, :, 2] >= MIN_VAL)
        & (product_gap >= MIN_PRODUCT_GAP)
        & (axis_strength >= MIN_DELTA_AXIS_STRENGTH),
        255,
        0,
    ).astype(np.uint8)

    blur_lab = cv2.GaussianBlur(lab.astype(np.uint8), (5, 5), 0)
    edge_mask = cv2.bitwise_or(
        cv2.Canny(blur_lab[:, :, 1], 6, 18),
        cv2.Canny(blur_lab[:, :, 2], 6, 18),
    )
    edge_mask = cv2.morphologyEx(edge_mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    edge_mask = cv2.dilate(edge_mask, np.ones((3, 3), np.uint8), iterations=1)
    edge_mask = cv2.bitwise_and(edge_mask, dlz_mask)

    split_mask = cv2.bitwise_and(candidate_mask, cv2.bitwise_not(edge_mask))

    proposal_mask = np.zeros_like(candidate_mask)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        (split_mask > 0).astype(np.uint8), connectivity=8
    )

    for label in range(1, num_labels):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < MIN_REGION_AREA:
            continue

        region = labels == label
        region_gap = float(np.median(product_gap[region]))
        region_axis = float(np.median(axis_strength[region]))

        if region_gap < MIN_REGION_PRODUCT_GAP:
            continue
        if region_axis < MIN_DELTA_AXIS_STRENGTH:
            continue

        proposal_mask[region] = 255

    proposal_candidate_mask = cv2.bitwise_and(candidate_mask, proposal_mask)

    da_strength = -delta_a
    db_strength = -delta_b

    provisional_green = np.where(
        (proposal_candidate_mask > 0)
        & (da_strength >= db_strength + DELTA_AXIS_MARGIN)
        & (delta_a <= -MIN_DELTA_AXIS_STRENGTH),
        255,
        0,
    ).astype(np.uint8)

    b_family = np.where(
        (proposal_candidate_mask > 0)
        & (db_strength >= da_strength + DELTA_AXIS_MARGIN)
        & (delta_b <= -MIN_DELTA_AXIS_STRENGTH),
        255,
        0,
    ).astype(np.uint8)

    provisional_blue = np.where(
        (b_family > 0)
        & (
            (lab_ab_product <= MAX_BLUE_PRODUCT)
            | (delta_a <= BLUE_DELTA_A_MAX)
            | (delta_b > STRONG_PURPLE_DELTA_B_MAX)
        ),
        255,
        0,
    ).astype(np.uint8)

    provisional_purple = cv2.bitwise_and(b_family, cv2.bitwise_not(provisional_blue))

    def finalize(mask: np.ndarray) -> np.ndarray:
        if not np.any(mask):
            return mask.copy()

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)

        out = np.zeros_like(mask)
        n, cc_labels, cc_stats, _ = cv2.connectedComponentsWithStats(
            (cleaned > 0).astype(np.uint8), connectivity=8
        )

        for cc in range(1, n):
            area = int(cc_stats[cc, cv2.CC_STAT_AREA])
            if area < FINAL_MIN_AREA:
                continue

            component = np.where(cc_labels == cc, 255, 0).astype(np.uint8)
            contours, _ = cv2.findContours(
                component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            contour = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect).astype(np.int32)

            rect_mask = np.zeros_like(mask)
            cv2.fillConvexPoly(rect_mask, box, 255)
            out = cv2.bitwise_or(out, cv2.bitwise_and(rect_mask, dlz_mask))

        return out

    green_mask = finalize(provisional_green)
    blue_mask = finalize(provisional_blue)
    purple_mask = finalize(provisional_purple)

    classified = cv2.bitwise_or(green_mask, cv2.bitwise_or(blue_mask, purple_mask))
    unknown_mask = cv2.bitwise_and(proposal_candidate_mask, cv2.bitwise_not(classified))

    return {
        "dlz_mask": dlz_mask,
        "candidate_mask": candidate_mask,
        "proposal_mask": proposal_mask,
        "green_mask": green_mask,
        "blue_mask": blue_mask,
        "purple_mask": purple_mask,
        "unknown_mask": unknown_mask,
    }
