import cv2
import numpy as np

# Hard‑coded HSV color ranges
COLOR_RANGES = {
    "red": [
        ((0, 80, 80), (10, 255, 255)),
        ((170, 80, 80), (180, 255, 255))
    ],
    "green": [
        ((40, 60, 60), (80, 255, 255))
    ],
    "blue": [
        ((100, 80, 80), (130, 255, 255))
    ],
}


def recognise_rectangle(frame, target_color):
    """
    Detect the largest rectangle of the target_color.
    Returns contour or None.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = []

    for (lo, hi) in COLOR_RANGES[target_color]:
        mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
        masks.append(mask)

    full_mask = masks[0]
    for m in masks[1:]:
        full_mask = cv2.bitwise_or(full_mask, m)

    contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    for c in contours:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        if len(approx) == 4:
            contour = approx    # (4,1,2)
            return contour

    return None
