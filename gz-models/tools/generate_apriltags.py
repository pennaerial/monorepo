#!/usr/bin/env python3
"""Render the tag36h11 PNGs stamped on the In House 2026 shapes.

Run once from anywhere after installing python3-opencv:
    python3 gz-models/tools/generate_apriltags.py
"""

import os

import cv2
import numpy as np

FAMILY = "tag36h11"
IDS = range(10, 35)  # 25 tags; ids 0-7 belong to the VTOL / payload tags; can update second param later
CELLS = 8  # 6 data bits + 1 black border cell per side
QUIET_CELLS = 1  # white margin; without it the tag does not decode
PX_PER_CELL = 40
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = f"{REPO_ROOT}/models/AprilTag36h11"


def render(tag_id: int) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    # drawMarker on OpenCV 4.6, generateImageMarker on 4.7+
    draw = getattr(cv2.aruco, "drawMarker", None) or cv2.aruco.generateImageMarker
    marker = draw(dictionary, tag_id, CELLS * PX_PER_CELL)

    pad = QUIET_CELLS * PX_PER_CELL
    side = CELLS * PX_PER_CELL + 2 * pad
    img = np.full((side, side), 255, np.uint8)
    img[pad : side - pad, pad : side - pad] = marker
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    for tag_id in IDS:
        path = f"{OUT_DIR}/{FAMILY}_{tag_id:05d}.png"
        cv2.imwrite(path, render(tag_id))
        print(f"wrote {path}")


if __name__ == "__main__":
    main()