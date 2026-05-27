from __future__ import annotations

import cv2
import numpy as np
from collections.abc import Sequence
from typing import Optional, Tuple
import os

from uav.utils import blue, green, pink, yellow

from .dlz_color_regions import detect_focused_dlz_paper_masks

_NAMED_PAYLOAD_BOUNDS = {
    "pink": pink,
    "green": green,
    "blue": blue,
    "yellow": yellow,
}
_PAPER_MASK_KEYS = {
    "green": "green_mask",
    "blue": "blue_mask",
    "purple": "purple_mask",
}
HsvBound = Sequence[int] | np.ndarray


def _hsv_bound_array(bound: HsvBound) -> np.ndarray:
    return np.asarray(bound, dtype=np.uint8)


def _normalize_payload_color(
    payload_color: str | None,
    lower_payload,
    upper_payload,
) -> str:
    if payload_color is not None:
        return str(payload_color).strip().lower()

    if lower_payload is None or upper_payload is None:
        return "unknown"

    lower = tuple(int(x) for x in np.asarray(lower_payload).reshape(-1))
    upper = tuple(int(x) for x in np.asarray(upper_payload).reshape(-1))
    for color_name, (named_lower, named_upper) in _NAMED_PAYLOAD_BOUNDS.items():
        if lower == tuple(named_lower) and upper == tuple(named_upper):
            return color_name
    return "unknown"


def _largest_contour_centroid(
    mask: np.ndarray,
) -> tuple[Optional[np.ndarray], Optional[int], Optional[int]]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None, None

    largest_contour = max(contours, key=cv2.contourArea)
    moments = cv2.moments(largest_contour)
    if moments["m00"] == 0:
        return largest_contour, None, None

    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    return largest_contour, cx, cy


def _legacy_focused_payload_mask(
    focused_bgr: np.ndarray,
    lower_payload: HsvBound | None,
    upper_payload: HsvBound | None,
) -> np.ndarray:
    if lower_payload is None or upper_payload is None:
        return np.zeros(focused_bgr.shape[:2], dtype=np.uint8)

    hsv_image = cv2.cvtColor(focused_bgr, cv2.COLOR_BGR2HSV)
    payload_mask = cv2.inRange(
        hsv_image, _hsv_bound_array(lower_payload), _hsv_bound_array(upper_payload)
    )
    kernel = np.ones((5, 5), np.uint8)
    payload_mask = cv2.morphologyEx(payload_mask, cv2.MORPH_OPEN, kernel)
    payload_mask = cv2.morphologyEx(payload_mask, cv2.MORPH_CLOSE, kernel)
    return payload_mask


def _resolved_payload_mask(
    focused_bgr: np.ndarray,
    requested_color: str,
    lower_payload,
    upper_payload,
    paper_masks: dict[str, np.ndarray],
) -> np.ndarray:
    """Use the focused-DLZ classifier first, then fall back to HSV if needed."""
    mask_key = _PAPER_MASK_KEYS.get(requested_color)
    if mask_key is not None:
        payload_mask = paper_masks[mask_key]
        if np.any(payload_mask):
            return payload_mask

    return _legacy_focused_payload_mask(focused_bgr, lower_payload, upper_payload)


def find_payload(
    image: np.ndarray,
    lower_zone: HsvBound,
    upper_zone: HsvBound,
    lower_payload: HsvBound | None,
    upper_payload: HsvBound | None,
    uuid: str,
    debug: bool = False,
    save_vision: bool = False,
    payload_color: str | None = None,
) -> Optional[Tuple[int, int, bool]]:
    """
    Detect the requested payload paper or return the DLZ centre as a fallback.

    Args:
        image (np.ndarray): Input BGR image.
        lower_zone (np.ndarray): Lower HSV threshold for zone marker.
        upper_zone (np.ndarray): Upper HSV threshold for zone marker.
        lower_payload (np.ndarray | None): Optional HSV fallback lower bound.
        upper_payload (np.ndarray | None): Optional HSV fallback upper bound.
        debug (bool): If True, return an image with visualizations.
        save_vision (bool): If True, save the visualization image.
        payload_color (str | None): Requested paper colour name.

    Returns:
        Optional[Tuple[int, int, bool]]: A tuple (cx, cy, dlz_empty) if detection
        is successful; otherwise, None.
    """
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    requested_color = _normalize_payload_color(
        payload_color, lower_payload, upper_payload
    )

    zone_mask = cv2.inRange(
        hsv_image, _hsv_bound_array(lower_zone), _hsv_bound_array(upper_zone)
    )
    kernel = np.ones((5, 5), np.uint8)
    dilated = cv2.dilate(zone_mask, kernel, iterations=3)
    zone_mask = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel)
    zone_mask = cv2.morphologyEx(zone_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(
        zone_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        vis_image = image.copy() if debug or save_vision else None
        if debug or save_vision:
            assert vis_image is not None
        if debug and vis_image is not None:
            cv2.imshow("Payload Tracking", vis_image)
            cv2.waitKey(1)
        if save_vision and vis_image is not None:
            import time

            time = int(time.time())
            path = os.path.expanduser(f"~/vision_imgs/{uuid}")
            cv2.imwrite(os.path.join(path, f"payload_{time}.png"), vis_image)
        return None

    largest_zone_contour = max(contours, key=cv2.contourArea)

    zone_filled_mask = np.zeros_like(zone_mask)
    cv2.drawContours(
        zone_filled_mask, [largest_zone_contour], -1, 255, thickness=cv2.FILLED
    )
    zone_moments = cv2.moments(zone_filled_mask)
    if zone_moments["m00"] == 0:
        return None
    zone_cx = int(zone_moments["m10"] / zone_moments["m00"])
    zone_cy = int(zone_moments["m01"] / zone_moments["m00"])

    focused_bgr = cv2.bitwise_and(image, image, mask=zone_filled_mask)
    paper_masks = None
    if requested_color == "pink":
        payload_mask = zone_filled_mask.copy()
        largest_payload_contour = largest_zone_contour
        cx, cy = zone_cx, zone_cy
        dlz_empty = False
    else:
        paper_masks = detect_focused_dlz_paper_masks(focused_bgr)
        payload_mask = _resolved_payload_mask(
            focused_bgr,
            requested_color,
            lower_payload,
            upper_payload,
            paper_masks,
        )

        largest_payload_contour, cx, cy = _largest_contour_centroid(payload_mask)
        if cx is None or cy is None:
            cx, cy = zone_cx, zone_cy
            dlz_empty = True
        else:
            dlz_empty = False

    vis_image: np.ndarray | None = None
    if debug or save_vision:
        vis_image = image.copy()

        cv2.drawContours(vis_image, [largest_zone_contour], -1, (0, 255, 0), 2)
        if largest_payload_contour is not None:
            cv2.drawContours(vis_image, [largest_payload_contour], -1, (0, 255, 0), 2)
        cv2.circle(vis_image, (cx, cy), 5, (0, 0, 255), -1)
    if debug and vis_image is not None:
        cv2.namedWindow("Payload Tracking", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("Payload Tracking", vis_image)
        cv2.imshow(f"DLZ Mask {lower_zone}, {upper_zone}", zone_mask)
        cv2.imshow(f"Payload Mask {requested_color}", payload_mask)
        if paper_masks is not None:
            cv2.imshow("Candidate Mask", paper_masks["candidate_mask"])
            cv2.imshow("Proposal Mask", paper_masks["proposal_mask"])
            cv2.imshow("Unknown Mask", paper_masks["unknown_mask"])
        cv2.waitKey(1)
    if save_vision and vis_image is not None:
        import time

        time = int(time.time())
        path = os.path.expanduser(f"~/vision_imgs/{uuid}")
        cv2.imwrite(os.path.join(path, f"payload_{time}.png"), vis_image)
        cv2.imwrite(os.path.join(path, f"zone_mask_{time}.png"), zone_mask)
        cv2.imwrite(os.path.join(path, f"payload_mask_{time}.png"), payload_mask)
        if paper_masks is not None:
            cv2.imwrite(
                os.path.join(path, f"candidate_mask_{time}.png"),
                paper_masks["candidate_mask"],
            )
            cv2.imwrite(
                os.path.join(path, f"proposal_mask_{time}.png"),
                paper_masks["proposal_mask"],
            )
            cv2.imwrite(
                os.path.join(path, f"green_mask_{time}.png"),
                paper_masks["green_mask"],
            )
            cv2.imwrite(
                os.path.join(path, f"blue_mask_{time}.png"),
                paper_masks["blue_mask"],
            )
            cv2.imwrite(
                os.path.join(path, f"purple_mask_{time}.png"),
                paper_masks["purple_mask"],
            )
            cv2.imwrite(
                os.path.join(path, f"unknown_mask_{time}.png"),
                paper_masks["unknown_mask"],
            )

    # def click_event(event, x, y, flags, param):
    #    if event == cv2.EVENT_LBUTTONDOWN:
    #        hsv_img = param['hsv']
    #        hsv_value = hsv_img[y, x]
    #        print(hsv_value)
    # cv2.namedWindow("image")
    # cv2.setMouseCallback("image", click_event, param={'hsv':hsv_image})
    # while True:
    #    cv2.imshow('image', image)
    #    cv2.waitKey(1)

    return cx, cy, dlz_empty


def rotate_image(image: np.ndarray, angle: float) -> np.ndarray:
    """
    Rotate an image by the specified angle.

    Args:
        image (np.ndarray): The input image.
        angle (float): The rotation angle in degrees. Typically, use the negative
                       of the camera yaw to compensate for rotation.

    Returns:
        np.ndarray: The rotated image.
    """
    # Get image dimensions
    (h, w) = image.shape[:2]
    # Compute the center of the image
    center = (w / 2, h / 2)
    # Compute the rotation matrix (note: negative angle to correct orientation)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    # Perform the affine transformation (rotation)
    rotated = cv2.warpAffine(image, M, (w, h))
    return rotated


def find_dlz(
    image: np.ndarray,
    lower_pink: np.ndarray,
    upper_pink: np.ndarray,
    lower_green: np.ndarray,
    upper_green: np.ndarray,
    debug: bool = False,
) -> Optional[Tuple[int, int, np.ndarray | None]]:
    """
    Detect payload in image using color thresholding.

    Args:
        image (np.ndarray): Input BGR image.
        lower_pink (np.ndarray): Lower HSV threshold for pink marker.
        upper_pink (np.ndarray): Upper HSV threshold for pink marker.
        lower_green (np.ndarray): Lower HSV threshold for green payload.
        upper_green (np.ndarray): Upper HSV threshold for green payload.

    Returns:
        Optional[Tuple[int, int, np.ndarray | None]]: A tuple
        (cx, cy, visualization_image) if detection is successful; otherwise, None.
    """
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Detect pink square
    pink_mask = cv2.inRange(hsv_image, lower_pink, upper_pink)

    kernel = np.ones((5, 5), np.uint8)
    pink_mask = cv2.morphologyEx(pink_mask, cv2.MORPH_CLOSE, kernel)
    pink_mask = cv2.morphologyEx(pink_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(
        pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None

    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)

    # Create masked region for green detection
    pink_square_mask = np.zeros_like(pink_mask)
    cv2.drawContours(pink_square_mask, [largest_contour], -1, 255, thickness=cv2.FILLED)
    M = cv2.moments(largest_contour)
    if M["m00"] == 0:
        return None
    cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    if not debug:
        return cx, cy, None
    vis_image = image.copy()
    cv2.rectangle(vis_image, (x, y), (x + w, y + h), (255, 0, 255), 2)
    cv2.drawContours(vis_image, [largest_contour], -1, (0, 255, 0), 3)
    cv2.circle(vis_image, (cx, cy), 5, (0, 255, 0), -1)
    return cx, cy, vis_image


def compute_3d_vector(
    x: float,
    y: float,
    camera_info: np.ndarray,  # expected to be a 3x3 camera intrinsic matrix
    altitude: float,
    offset_x: float = 0,  # in meters
    offset_y: float = 0,  # in meters
    offset_z: float = 0,  # in meters
) -> Tuple[float, float, float]:
    """Convert pixel coordinates to a 3D direction vector."""
    K = np.array(camera_info)
    pixel_coords = np.array([x, y, 1.0])
    cam_coords = np.linalg.inv(K) @ pixel_coords

    target_point_sensor = cam_coords * altitude

    sensor_to_center_offset = np.array([offset_x, offset_y, offset_z])

    target_point_center = target_point_sensor + sensor_to_center_offset

    return tuple(target_point_center / np.linalg.norm(target_point_center))


if __name__ == "__main__":
    # Test the functions
    image = cv2.imread(
        "/Users/ethanyu/VSCodeProjects/monorepo/good_images/image_20250322_191023.png"
    )
    if image is None:
        raise RuntimeError("Unable to read debug image.")
    print(
        find_payload(
            image,
            (140, 155, 50),
            (170, 255, 255),
            (140, 155, 50),
            (170, 255, 255),
            "debug",
            debug=True,
            payload_color="pink",
        )
    )
