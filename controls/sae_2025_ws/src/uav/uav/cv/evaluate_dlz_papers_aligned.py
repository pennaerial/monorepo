from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}

ORANGE_LOWER = np.array([5, 78, 158], dtype=np.uint8)
ORANGE_UPPER = np.array([25, 189, 255], dtype=np.uint8)
MIN_CONTOUR_AREA = 5000
DLZ_KERNEL = np.ones((15, 15), np.uint8)

# All color thresholds are relative to the frame's orange DLZ color in Lab.
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


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactive viewer for the relative-Lab DLZ paper heuristic.",
    )
    parser.add_argument("folder", type=Path, help="Folder containing input images.")
    parser.add_argument(
        "--save-dir",
        type=Path,
        default=None,
        help="Optional folder to save the overview and substep grids.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print kept component features for each color.",
    )
    return parser.parse_args()


def _iter_image_paths(folder: Path) -> list[Path]:
    return sorted(
        path for path in folder.iterdir() if path.suffix.lower() in IMAGE_EXTENSIONS
    )


def get_dlz_hull(frame: np.ndarray) -> np.ndarray | None:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    orange_mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)

    height, width = frame.shape[:2]
    flood_fill_mask = np.zeros((height + 2, width + 2), dtype=np.uint8)
    flood_fill_mask[1:-1, 1:-1] = orange_mask

    fill_img = np.zeros((height, width), dtype=np.uint8)
    cv2.floodFill(fill_img, flood_fill_mask.copy(), (0, 0), 255)
    cv2.floodFill(fill_img, flood_fill_mask.copy(), (width - 1, 0), 255)

    interior = cv2.bitwise_not(fill_img)
    interior = cv2.erode(interior, DLZ_KERNEL)
    interior = cv2.dilate(interior, DLZ_KERNEL)

    contours, _ = cv2.findContours(interior, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    surviving = [contour for contour in contours if cv2.contourArea(contour) >= MIN_CONTOUR_AREA]
    if not surviving:
        return None

    return cv2.convexHull(np.concatenate(surviving, axis=0))


def _filter_components(
    mask: np.ndarray,
    hue: np.ndarray,
    saturation: np.ndarray,
    delta_a: np.ndarray,
    delta_b: np.ndarray,
    dist_to_boundary: np.ndarray,
    keep_fn,
) -> tuple[np.ndarray, list[dict[str, float | int]]]:
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    kept_mask = np.zeros_like(mask)
    kept_features: list[dict[str, float | int]] = []

    for label in range(1, num_labels):
        component = labels == label
        x, y, width, height, area = stats[label]
        features: dict[str, float | int] = {
            "area": int(area),
            "x": int(x),
            "y": int(y),
            "w": int(width),
            "h": int(height),
            "meanH": float(hue[component].mean()),
            "meanS": float(saturation[component].mean()),
            "meanDA": float(delta_a[component].mean()),
            "meanDB": float(delta_b[component].mean()),
            "dmax": float(dist_to_boundary[component].max()),
        }
        if keep_fn(features):
            kept_mask[component] = 255
            kept_features.append(features)

    return kept_mask, kept_features


def get_color_masks(
    frame: np.ndarray,
) -> tuple[np.ndarray | None, np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, object]]:
    hull = get_dlz_hull(frame)
    height, width = frame.shape[:2]
    zeros = np.zeros((height, width), dtype=np.uint8)
    if hull is None:
        info: dict[str, object] = {
            "orange_a0": 0,
            "orange_b0": 0,
            "orange_mask": zeros.copy(),
            "orange_roi_mask": zeros.copy(),
            "candidate_mask": zeros.copy(),
            "green_raw_mask": zeros.copy(),
            "purple_raw_mask": zeros.copy(),
            "blue_strong_raw_mask": zeros.copy(),
            "blue_strong_mask": zeros.copy(),
            "residual_mask": zeros.copy(),
            "residual2_mask": zeros.copy(),
            "weak_blue_mask": zeros.copy(),
            "weak_purple_mask": zeros.copy(),
            "green_components": [],
            "purple_components": [],
            "blue_strong_components": [],
            "weak_blue_components": [],
            "weak_purple_components": [],
        }
        return None, zeros, zeros, zeros, zeros, info

    dlz_mask = np.zeros((height, width), dtype=np.uint8)
    cv2.drawContours(dlz_mask, [hull], -1, 255, thickness=cv2.FILLED)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    hue = hsv[:, :, 0]
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    lab_a = lab[:, :, 1].astype(np.int16)
    lab_b = lab[:, :, 2].astype(np.int16)

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
    delta_ab = np.sqrt(delta_a.astype(np.float32) ** 2 + delta_b.astype(np.float32) ** 2)

    candidate = (
        (dlz_mask > 0)
        & (delta_ab > CANDIDATE_DAB_MIN)
        & (saturation >= CANDIDATE_S_MIN)
        & (saturation <= CANDIDATE_S_MAX)
        & (value > CANDIDATE_V_MIN)
    ).astype(np.uint8) * 255

    dist_to_boundary = cv2.distanceTransform(dlz_mask, cv2.DIST_L2, 5)

    green_raw = ((candidate > 0) & (delta_a <= GREEN_DA_MAX)).astype(np.uint8) * 255
    green_mask, green_components = _filter_components(
        green_raw,
        hue,
        saturation,
        delta_a,
        delta_b,
        dist_to_boundary,
        lambda features: features["area"] >= 150 and features["dmax"] >= 5,
    )

    red_channel = frame[:, :, 2]
    green_channel = frame[:, :, 1]
    blue_channel = frame[:, :, 0]

    purple_raw = (
        (candidate > 0)
        & (delta_b <= PURPLE_DB_MAX)
        & (
            (hue >= 145)
            | ((hue <= 8) & (red_channel > green_channel) & (red_channel >= blue_channel - 10))
        )
    ).astype(np.uint8) * 255
    purple_mask, purple_components = _filter_components(
        purple_raw,
        hue,
        saturation,
        delta_a,
        delta_b,
        dist_to_boundary,
        lambda features: features["dmax"] >= PURPLE_KEEP_DMAX_MIN
        and (
            features["meanH"] >= PURPLE_KEEP_MEAN_H_MIN
            or features["meanDB"] <= PURPLE_KEEP_MEAN_DB_MAX
        ),
    )

    blue_strong_raw = (
        (candidate > 0)
        & (hue >= BLUE_STRONG_H_MIN)
        & (hue <= BLUE_STRONG_H_MAX)
        & (delta_b <= BLUE_STRONG_DB_MAX)
    ).astype(np.uint8) * 255
    blue_strong_mask, blue_strong_components = _filter_components(
        blue_strong_raw,
        hue,
        saturation,
        delta_a,
        delta_b,
        dist_to_boundary,
        lambda features: features["area"] >= BLUE_STRONG_MIN_AREA
        and features["dmax"] >= BLUE_STRONG_DMAX_MIN,
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
        and features["area"] >= WEAK_BLUE_MIN_AREA
        and features["h"] <= WEAK_BLUE_MAX_COMPONENT_H
        and features["meanDA"] <= WEAK_BLUE_MEAN_DA_MAX
        and features["meanDB"] <= WEAK_BLUE_MEAN_DB_MAX,
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
        and features["area"] >= WEAK_PURPLE_MIN_AREA
        and features["h"] <= WEAK_PURPLE_MAX_COMPONENT_H
        and features["x"] >= int(WEAK_PURPLE_MIN_X_FRAC * width)
        and features["meanDB"] <= WEAK_PURPLE_MEAN_DB_MAX
        and features["meanDA"] > WEAK_PURPLE_MEAN_DA_MIN,
    )

    blue_mask = cv2.bitwise_or(blue_strong_mask, weak_blue_mask)
    purple_mask = cv2.bitwise_or(purple_mask, weak_purple_mask)

    thin_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    blue_mask = cv2.bitwise_and(cv2.dilate(blue_mask, thin_kernel, iterations=1), dlz_mask)
    purple_mask = cv2.bitwise_and(cv2.dilate(purple_mask, thin_kernel, iterations=1), dlz_mask)

    info: dict[str, object] = {
        "orange_a0": orange_a,
        "orange_b0": orange_b,
        "orange_mask": orange_mask,
        "orange_roi_mask": orange_roi.astype(np.uint8) * 255,
        "candidate_mask": candidate,
        "green_raw_mask": green_raw,
        "purple_raw_mask": purple_raw,
        "blue_strong_raw_mask": blue_strong_raw,
        "blue_strong_mask": blue_strong_mask,
        "residual_mask": residual,
        "residual2_mask": residual2,
        "weak_blue_mask": weak_blue_mask,
        "weak_purple_mask": weak_purple_mask,
        "green_components": green_components,
        "purple_components": purple_components,
        "blue_strong_components": blue_strong_components,
        "weak_blue_components": weak_blue_components,
        "weak_purple_components": weak_purple_components,
    }
    return hull, dlz_mask, green_mask, blue_mask, purple_mask, info


def overlay_masks(
    frame: np.ndarray,
    green_mask: np.ndarray,
    blue_mask: np.ndarray,
    purple_mask: np.ndarray,
    hull: np.ndarray | None = None,
) -> np.ndarray:
    overlay = frame.copy()
    overlay[green_mask > 0] = (0, 255, 0)
    overlay[blue_mask > 0] = (255, 0, 0)
    overlay[purple_mask > 0] = (255, 0, 255)
    output = cv2.addWeighted(frame, 0.65, overlay, 0.35, 0.0)
    if hull is not None:
        cv2.polylines(output, [hull], True, (0, 255, 255), 2)
    return output


def _mask_to_bgr(mask: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


def _label_panel(image: np.ndarray, label: str) -> np.ndarray:
    panel = image.copy()
    cv2.rectangle(panel, (0, 0), (panel.shape[1], 28), (0, 0, 0), thickness=cv2.FILLED)
    cv2.putText(
        panel,
        label,
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return panel


def _fit_panel(image: np.ndarray, *, target_width: int = 420) -> np.ndarray:
    height, width = image.shape[:2]
    if width <= target_width:
        return image.copy()
    scale = target_width / float(width)
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)


def _make_grid(panels: list[tuple[str, np.ndarray]], *, columns: int = 3) -> np.ndarray:
    fitted = [(_label_panel(_fit_panel(image), label)) for label, image in panels]
    max_height = max(panel.shape[0] for panel in fitted)
    max_width = max(panel.shape[1] for panel in fitted)

    normalized: list[np.ndarray] = []
    for panel in fitted:
        canvas = np.zeros((max_height, max_width, 3), dtype=np.uint8)
        canvas[: panel.shape[0], : panel.shape[1]] = panel
        normalized.append(canvas)

    rows: list[np.ndarray] = []
    for start in range(0, len(normalized), columns):
        row = normalized[start : start + columns]
        while len(row) < columns:
            row.append(np.zeros((max_height, max_width, 3), dtype=np.uint8))
        rows.append(cv2.hconcat(row))
    return cv2.vconcat(rows)


def _format_detection_text(
    green_mask: np.ndarray,
    blue_mask: np.ndarray,
    purple_mask: np.ndarray,
) -> str:
    detections = []
    for color_name, mask in (
        ("green", green_mask),
        ("blue", blue_mask),
        ("purple", purple_mask),
    ):
        pixel_count = int(np.count_nonzero(mask))
        if pixel_count > 0:
            detections.append(f"{color_name}:{pixel_count}")
    return "none" if not detections else " ".join(detections)


def main() -> None:
    args = _parse_args()
    folder = args.folder.expanduser().resolve()
    if not folder.is_dir():
        raise SystemExit(f"{folder} is not a directory.")

    paths = _iter_image_paths(folder)
    if not paths:
        raise SystemExit(f"No image files found in {folder}.")

    save_dir = None
    if args.save_dir is not None:
        save_dir = args.save_dir.expanduser().resolve()
        save_dir.mkdir(parents=True, exist_ok=True)

    print(f"Found {len(paths)} images. Controls: Enter/Space=next, b=back, q=quit.")
    index = 0
    while 0 <= index < len(paths):
        path = paths[index]
        frame = cv2.imread(str(path))
        if frame is None:
            print(f"{path.name},load_error")
            index += 1
            continue

        hull, dlz_mask, green_mask, blue_mask, purple_mask, info = get_color_masks(frame)
        print(
            f"{path.name},"
            f"dlz={int(np.count_nonzero(dlz_mask))},"
            f"{_format_detection_text(green_mask, blue_mask, purple_mask)}"
        )

        if args.verbose:
            print(f"  green: {info.get('green_components', [])}")
            print(f"  blue strong: {info.get('blue_strong_components', [])}")
            print(f"  weak blue: {info.get('weak_blue_components', [])}")
            print(f"  purple: {info.get('purple_components', [])}")
            print(f"  weak purple: {info.get('weak_purple_components', [])}")

        overlay = overlay_masks(frame, green_mask, blue_mask, purple_mask, hull)
        overview_grid = _make_grid(
            [
                ("Original", frame),
                ("Overlay", overlay),
                ("DLZ Mask", _mask_to_bgr(dlz_mask)),
                ("Orange Mask", _mask_to_bgr(info["orange_mask"])),
                ("Orange ROI", _mask_to_bgr(info["orange_roi_mask"])),
                ("Candidate", _mask_to_bgr(info["candidate_mask"])),
            ],
            columns=3,
        )
        substeps_grid = _make_grid(
            [
                ("Green Raw", _mask_to_bgr(info["green_raw_mask"])),
                ("Green Final", _mask_to_bgr(green_mask)),
                ("Purple Raw", _mask_to_bgr(info["purple_raw_mask"])),
                ("Purple Final", _mask_to_bgr(purple_mask)),
                ("Blue Strong Raw", _mask_to_bgr(info["blue_strong_raw_mask"])),
                ("Blue Strong Final", _mask_to_bgr(info["blue_strong_mask"])),
                ("Residual", _mask_to_bgr(info["residual_mask"])),
                ("Weak Blue", _mask_to_bgr(info["weak_blue_mask"])),
                ("Residual 2", _mask_to_bgr(info["residual2_mask"])),
                ("Weak Purple", _mask_to_bgr(info["weak_purple_mask"])),
                ("Blue Final", _mask_to_bgr(blue_mask)),
                ("Purple Final", _mask_to_bgr(purple_mask)),
            ],
            columns=3,
        )

        cv2.imshow("DLZ Papers Aligned Overview", overview_grid)
        cv2.imshow("DLZ Papers Aligned Substeps", substeps_grid)

        if save_dir is not None:
            stem = path.stem
            cv2.imwrite(str(save_dir / f"{stem}_overview.png"), overview_grid)
            cv2.imwrite(str(save_dir / f"{stem}_substeps.png"), substeps_grid)

        while True:
            key = cv2.waitKey(0) & 0xFF
            if key in (13, 32):
                index += 1
                break
            if key == ord("b"):
                index = max(0, index - 1)
                break
            if key == ord("q"):
                cv2.destroyAllWindows()
                return

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
