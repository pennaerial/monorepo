from __future__ import annotations

import os
import sys

import cv2
import numpy as np

ORANGE_LOWER = np.array([5, 78, 158], dtype=np.uint8)
ORANGE_UPPER = np.array([25, 189, 255], dtype=np.uint8)
MIN_CONTOUR_AREA = 5000
BLUR_KERNEL = (5, 5)
MORPH_KERNEL = (15, 15)


def preprocess_frame(frame: np.ndarray) -> np.ndarray:
    """Denoise the frame before HSV thresholding."""
    return cv2.GaussianBlur(frame, BLUR_KERNEL, 0)


def build_debug_masks(
    frame: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    preprocessed = preprocess_frame(frame)
    hsv = cv2.cvtColor(preprocessed, cv2.COLOR_BGR2HSV)
    orange_mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)

    h, w = frame.shape[:2]
    ff_mask = np.zeros((h + 2, w + 2), dtype=np.uint8)
    ff_mask[1:-1, 1:-1] = orange_mask

    fill_img = np.zeros((h, w), dtype=np.uint8)
    cv2.floodFill(fill_img, ff_mask.copy(), (0, 0), 255)
    cv2.floodFill(fill_img, ff_mask.copy(), (w - 1, 0), 255)

    interior = cv2.bitwise_not(fill_img)

    # Remove edge speckle before contour extraction, then restore the main body.
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, MORPH_KERNEL)
    interior = cv2.erode(interior, kernel)
    interior = cv2.dilate(interior, kernel)

    return preprocessed, orange_mask, fill_img, interior


def get_dlz_hull(
    frame: np.ndarray, interior: np.ndarray | None = None
) -> np.ndarray | None:
    if interior is None:
        _, _, _, interior = build_debug_masks(frame)
    contours, _ = cv2.findContours(interior, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    print(f"  Found {len(contours)} contours:")
    surviving: list[np.ndarray] = []
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        status = "PASS" if area >= MIN_CONTOUR_AREA else "FAIL"
        print(f"    contour {i}: area={area:.0f} [{status}]")
        if area >= MIN_CONTOUR_AREA:
            surviving.append(contour)

    if not surviving:
        return None

    all_points = np.concatenate(surviving, axis=0)
    return cv2.convexHull(all_points)


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: python dlz_convex_hull.py <folder_path>")
        return 1

    folder = sys.argv[1]
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif"}
    paths = sorted(
        os.path.join(folder, name)
        for name in os.listdir(folder)
        if os.path.splitext(name)[1].lower() in exts
    )

    if not paths:
        print(f"No images found in '{folder}'")
        return 1

    print(f"Found {len(paths)} images. Press Enter to advance, 'q' to quit.")

    for i, path in enumerate(paths):
        frame = cv2.imread(path)
        if frame is None:
            print(f"[{i + 1}/{len(paths)}] Could not load {path}, skipping.")
            continue

        print(f"[{i + 1}/{len(paths)}] {os.path.basename(path)}")

        preprocessed, orange_mask, fill_img, interior = build_debug_masks(frame)
        hull = get_dlz_hull(frame, interior=interior)

        debug = frame.copy()
        if hull is not None:
            cv2.polylines(debug, [hull], True, (0, 255, 0), 2)
        else:
            print("  No DLZ detected.")

        cv2.imshow("Preprocessed", preprocessed)
        cv2.imshow("Orange mask", orange_mask)
        cv2.imshow("Flood fill (white=outside)", fill_img)
        cv2.imshow("Interior mask", interior)
        cv2.imshow("DLZ Hull", debug)

        while True:
            key = cv2.waitKey(0) & 0xFF
            if key == 13:
                break
            if key == ord("q"):
                cv2.destroyAllWindows()
                return 0

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
