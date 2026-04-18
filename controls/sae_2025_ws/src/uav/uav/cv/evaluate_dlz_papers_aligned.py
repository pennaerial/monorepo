from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from uav.cv.dlz_masking import detect_dlz_orange_barrier_mask
from uav.cv.dlz_relative_color_masks import detect_dlz_relative_color_masks

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
    dlz_result = detect_dlz_orange_barrier_mask(frame)
    if not np.any(dlz_result.applied_mask):
        return None
    contours, _ = cv2.findContours(
        dlz_result.applied_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        return None
    return cv2.convexHull(np.concatenate(contours, axis=0))


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
    dlz_result = detect_dlz_orange_barrier_mask(frame)
    hull = get_dlz_hull(frame)
    if np.any(dlz_result.applied_mask):
        dlz_mask = dlz_result.applied_mask
    else:
        dlz_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    result = detect_dlz_relative_color_masks(dlz_result.frame, dlz_mask=dlz_mask)
    info: dict[str, object] = {
        "orange_a0": result.orange_a0,
        "orange_b0": result.orange_b0,
        "orange_mask": result.orange_mask,
        "orange_roi_mask": result.orange_roi_mask,
        "candidate_mask": result.candidate_mask,
        "green_raw_mask": result.green_raw_mask,
        "purple_raw_mask": result.purple_raw_mask,
        "blue_strong_raw_mask": result.blue_strong_raw_mask,
        "blue_strong_mask": result.blue_strong_mask,
        "residual_mask": result.residual_mask,
        "residual2_mask": result.residual2_mask,
        "weak_blue_mask": result.weak_blue_mask,
        "weak_purple_mask": result.weak_purple_mask,
        "green_components": result.green_components,
        "purple_components": result.purple_components,
        "blue_strong_components": result.blue_strong_components,
        "weak_blue_components": result.weak_blue_components,
        "weak_purple_components": result.weak_purple_components,
    }
    return (
        hull,
        result.dlz_mask,
        result.mask_green,
        result.mask_blue,
        result.mask_purple,
        info,
    )


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
