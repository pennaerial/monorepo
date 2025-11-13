# tracking.py
import cv2
import numpy as np
from typing import Optional, Tuple
import os
from pathlib import Path
import math
import warnings

def find_payload(
    image: np.ndarray,
    lower_zone: np.ndarray,
    upper_zone: np.ndarray,
    lower_payload: np.ndarray,
    upper_payload: np.ndarray,
    uuid: str,
    debug: bool = False,
    save_vision: bool = False,
) -> Optional[Tuple[int, int, bool]]:
    """
    Detect payload in image using color thresholding.
    
    Args:
        image (np.ndarray): Input BGR image.
        lower_zone (np.ndarray): Lower HSV threshold for zone marker.
        upper_zone (np.ndarray): Upper HSV threshold for zone marker.
        lower_payload (np.ndarray): Lower HSV threshold for payload.
        upper_payload (np.ndarray): Upper HSV threshold for payload.
        debug (bool): If True, return an image with visualizations.
        save_vision (bool): If True, save the visualization image.
    
    Returns:
        Optional[Tuple[int, int, bool]]: A tuple (cx, cy, zone_empty) if detection is successful;
        otherwise, None.
    """
    # Convert to HSV color space.
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create zone mask and clean it using morphological operations.
    zone_mask = cv2.inRange(hsv_image, lower_zone, upper_zone)
    kernel = np.ones((5, 5), np.uint8)
    dilated = cv2.dilate(zone_mask, kernel, iterations=3)
    zone_mask = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel)
    zone_mask = cv2.morphologyEx(zone_mask, cv2.MORPH_OPEN, kernel)

    # Find all external contours in the zone mask.
    contours, _ = cv2.findContours(zone_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        if debug or save_vision:
            vis_image = image.copy()
        if debug:
            cv2.imshow("Payload Tracking", vis_image)
            cv2.waitKey(1)
        if save_vision:
            import time
            time = int(time.time())
            path = os.path.expanduser(f"~/vision_imgs/{uuid}")
            cv2.imwrite(os.path.join(path, f"payload_{time}.png"), vis_image)
        return None

    # Find the largest zone contour.
    largest_zone_contour = max(contours, key=cv2.contourArea)

    # Create a filled mask from the largest zone contour.
    zone_filled_mask = np.zeros_like(zone_mask)
    cv2.drawContours(zone_filled_mask, [largest_zone_contour], -1, 255, thickness=cv2.FILLED)

    # Detect payload areas in the original HSV image.
    payload_mask_full = cv2.inRange(hsv_image, lower_payload, upper_payload)

    # Restrict the payload mask to the zone-filled region.
    payload_mask = cv2.bitwise_and(payload_mask_full, zone_filled_mask)

    # Clean the resulting payload mask.
    payload_mask = cv2.morphologyEx(payload_mask, cv2.MORPH_OPEN, kernel)
    payload_mask = cv2.morphologyEx(payload_mask, cv2.MORPH_CLOSE, kernel)

    # Find external contours in the payload mask.
    payload_contours, _ = cv2.findContours(payload_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_payload_contour = None
    
    if payload_contours and lower_payload is not lower_zone and upper_payload is not upper_zone: 
        # If payload is detected, compute the centroid of the largest payload contour.
        largest_payload_contour = max(payload_contours, key=cv2.contourArea)
        M_payload = cv2.moments(largest_payload_contour)
        if M_payload["m00"] == 0:
            return None
        cx, cy = int(M_payload["m10"] / M_payload["m00"]), int(M_payload["m01"] / M_payload["m00"])
    else:
        # Fallback: if no payload is found, compute the centroid of the zone area.
        M_zone = cv2.moments(zone_filled_mask)
        if M_zone["m00"] == 0:
            return None
        cx, cy = int(M_zone["m10"] / M_zone["m00"]), int(M_zone["m01"] / M_zone["m00"])
    if debug or save_vision:
        vis_image = image.copy()

        cv2.drawContours(vis_image, [largest_zone_contour], -1, (0, 255, 0), 2)
        # If payload was detected, draw its contour.
        if payload_contours:
            cv2.drawContours(vis_image, [largest_payload_contour], -1, (0, 255, 0), 2)
        # Mark the detected center.
        cv2.circle(vis_image, (cx, cy), 5, (0, 0, 255), -1)
    if debug:
        cv2.namedWindow("Payload Tracking", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("Payload Tracking", vis_image)
        cv2.imshow(f"DLZ Mask {lower_zone}, {upper_zone}", zone_mask)
        cv2.imshow(f"Payload Mask {lower_payload}, {upper_payload}", payload_mask)
        cv2.waitKey(1)
    if save_vision:
        import time
        time = int(time.time())
        path = os.path.expanduser(f"~/vision_imgs/{uuid}")
        cv2.imwrite(os.path.join(path, f"payload_{time}.png"), vis_image)
        cv2.imwrite(os.path.join(path, f"zone_mask_{time}.png"), zone_mask)
        cv2.imwrite(os.path.join(path, f"payload_mask_{time}.png"), payload_mask)
    
    #def click_event(event, x, y, flags, param):
    #    if event == cv2.EVENT_LBUTTONDOWN:
    #        hsv_img = param['hsv']
    #        hsv_value = hsv_img[y, x]
    #        print(hsv_value)
    #cv2.namedWindow("image")
    #cv2.setMouseCallback("image", click_event, param={'hsv':hsv_image})
    #while True:
    #    cv2.imshow('image', image)
    #    cv2.waitKey(1)
    
    return cx, cy, not bool(payload_contours)

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
    debug: bool = False
) -> Optional[Tuple[int, int, np.ndarray]]:
    """
    Detect payload in image using color thresholding.
    
    Args:
        image (np.ndarray): Input BGR image.
        lower_pink (np.ndarray): Lower HSV threshold for pink marker.
        upper_pink (np.ndarray): Upper HSV threshold for pink marker.
        lower_green (np.ndarray): Lower HSV threshold for green payload.
        upper_green (np.ndarray): Upper HSV threshold for green payload.
    
    Returns:
        Optional[Tuple[int, int, np.ndarray]]: A tuple (cx, cy, visualization_image)
        if detection is successful; otherwise, None.
    """
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Detect pink square
    pink_mask = cv2.inRange(hsv_image, lower_pink, upper_pink)
    
    kernel = np.ones((5, 5), np.uint8)
    pink_mask = cv2.morphologyEx(pink_mask, cv2.MORPH_CLOSE, kernel)
    pink_mask = cv2.morphologyEx(pink_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
        
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    
    # Create masked region for green detection
    pink_square_mask = np.zeros_like(pink_mask)
    cv2.drawContours(pink_square_mask, [largest_contour], -1, 255, thickness=cv2.FILLED)
    hsv_masked = cv2.bitwise_and(hsv_image, hsv_image, mask=pink_square_mask)
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
    offset_x: float = 0, # in meters
    offset_y: float = 0, # in meters
    offset_z: float = 0  # in meters
) -> Tuple[float, float, float]:
    """Convert pixel coordinates to a 3D direction vector."""
    K = np.array(camera_info)
    pixel_coords = np.array([x, y, 1.0])
    cam_coords = np.linalg.inv(K) @ pixel_coords
    
    target_point_sensor = cam_coords * altitude
    
    sensor_to_center_offset = np.array([offset_x, offset_y, offset_z])
    
    target_point_center = target_point_sensor + sensor_to_center_offset
    
    return tuple(target_point_center / np.linalg.norm(target_point_center))

# def find_hoop(
#     image: np.ndarray,
#     lower_hoop1: np.ndarray,
#     upper_hoop1: np.ndarray,
#     lower_hoop2: np.ndarray,
#     upper_hoop2: np.ndarray,
#     uuid: str,
#     debug: bool = False,
#     save_vision: bool = False,
# ) -> Optional[Tuple[int, int, bool]]:
#     """
#     Detect a colored hoop in an image using two HSV ranges (for full red/orange coverage).

#     Args:
#         image (np.ndarray): Input BGR image.
#         lower_hoop1, upper_hoop1 (np.ndarray): First HSV range for hoop color.
#         lower_hoop2, upper_hoop2 (np.ndarray): Second HSV range (for hue wraparound).
#         uuid (str): Unique ID for saving vision data.
#         debug (bool): If True, show visualization windows.
#         save_vision (bool): If True, save visualization frames.

#     Returns:
#         Optional[Tuple[int, int, bool]]:
#             (cx, cy, zone_empty)
#             cx, cy = coordinates of hoop center if detected.
#             zone_empty = True if no hoop is detected, False otherwise.
#     """
#     # Convert to HSV
#     hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

#     # Create dual masks for hue wraparound (e.g. red 0–10° and 170–180°)
#     mask1 = cv2.inRange(hsv_image, lower_hoop1, upper_hoop1)
#     mask2 = cv2.inRange(hsv_image, lower_hoop2, upper_hoop2)
#     mask = cv2.bitwise_or(mask1, mask2)

#     # Morphological cleaning
#     kernel = np.ones((5, 5), np.uint8)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

#     # Find contours
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     if not contours:
#         if debug or save_vision:
#             vis_image = image.copy()
#         if debug:
#             cv2.imshow("Hoop Detection", vis_image)
#             cv2.imshow("Hoop Mask", mask)
#             cv2.waitKey(1)
#         if save_vision:
#             import time
#             t = int(time.time())
#             path = os.path.expanduser(f"~/vision_imgs/{uuid}")
#             os.makedirs(path, exist_ok=True)
#             cv2.imwrite(os.path.join(path, f"hoop_{t}.png"), vis_image)
#         return None

#     # Select largest contour (assumed hoop)
#     largest_contour = max(contours, key=cv2.contourArea)
#     if cv2.contourArea(largest_contour) < 200:  # filter out small noise
#         return None

#     # Compute centroid
#     M = cv2.moments(largest_contour)
#     if M["m00"] == 0:
#         return None
#     cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

#     # Visualization
#     if debug or save_vision:
#         vis_image = image.copy()
#         cv2.drawContours(vis_image, [largest_contour], -1, (0, 255, 0), 2)
#         cv2.circle(vis_image, (cx, cy), 6, (0, 255, 0), 2)
#         cv2.drawMarker(vis_image, (cx, cy), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)

#         if debug:
#             cv2.imshow("Hoop Detection", vis_image)
#             cv2.imshow("Hoop Mask", mask)
#             cv2.waitKey(1)

#         if save_vision:
#             import time
#             t = int(time.time())
#             path = os.path.expanduser(f"~/vision_imgs/{uuid}")
#             os.makedirs(path, exist_ok=True)
#             cv2.imwrite(os.path.join(path, f"hoop_{t}.png"), vis_image)
#             cv2.imwrite(os.path.join(path, f"hoop_mask_{t}.png"), mask)

#     return cx, cy, not bool(contours)

def find_hoop(
    image: np.ndarray,
    lower_hoop1: np.ndarray,
    upper_hoop1: np.ndarray,
    lower_hoop2: np.ndarray,
    upper_hoop2: np.ndarray,
    uuid: str,
    debug: bool = False,
    save_vision: bool = False,
    # ----- Optional tuning knobs (do not break your original calls) -----
    min_area: float = 300.0,  # ignore tiny specks; scale with resolution
    w_area: float = 3.0,      # weight: larger blob area (proxy for "closer")
    w_circ: float = 1.5,      # weight: circularity (roundness)
    w_ring: float = 1.0,      # weight: ring-ness (hollow vs solid)
    w_center: float = 1.0     # penalty weight: distance from image center
) -> Optional[Tuple[int, int, bool]]:
    """
    Robust multi-hoop detector that selects the best candidate via a weighted score.

    Pipeline:
      1) Convert BGR -> HSV and threshold with two hue bands to cover red wrap-around
      2) Morphological filtering to clean noise
      3) Find *all* external contours (candidates)
      4) For each candidate compute:
            - area            (pixel area of contour)
            - circularity     (4πA/P² ∈ [0,1], 1 = perfect circle)
            - ring_score      (hollow-ness via erosion; 1 = very ring-like)
            - center distance (normalized to [0,1] by image diagonal)
      5) Score candidates: w_area*area_norm + w_circ*circularity + w_ring*ring_score - w_center*center_dist
      6) Choose highest-scoring candidate and return its centroid (cx, cy)

    Args:
        image: BGR frame (H x W x 3) as np.uint8
        lower_hoop1 / upper_hoop1: HSV bounds for red/orange band A (e.g. [0,120,80]..[10,255,255])
        lower_hoop2 / upper_hoop2: HSV bounds for wrap-around band B (e.g. [170,120,80]..[180,255,255])
        uuid: used only when saving debug imagery
        debug: if True, show OpenCV windows with overlays
        save_vision: if True, save annotated frame & mask beside this script
        min_area: reject tiny contours (tune based on resolution)
        w_area, w_circ, w_ring, w_center: scoring weights

    Returns:
        (cx, cy, False) if at least one hoop-like blob is selected; None otherwise.
        The third value keeps your original contract (zone_empty=False when detected).
    """
    # -----------------------------
    # (1) HSV + dual mask (red wrap)
    # -----------------------------
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Two masks because hue wraps at 0/180 (red sits at both ends)
    mask1 = cv2.inRange(hsv, lower_hoop1, upper_hoop1)
    mask2 = cv2.inRange(hsv, lower_hoop2, upper_hoop2)
    mask  = cv2.bitwise_or(mask1, mask2)

    # --------------------------------------------
    # (2) Morphology: remove speckle, fill pinholes
    # --------------------------------------------
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)  # erode->dilate (removes small white noise)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # dilate->erode (fills small black gaps)

    # --------------------------------
    # (3) Contours = candidate regions
    # --------------------------------
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        # Optional visualization/saving when nothing found
        # if debug:
        #     cv2.imshow("Hoop Detection", image)
        #     cv2.imshow("Hoop Mask", mask)
        #     cv2.waitKey(1)
        # if save_vision:
        #     import time
        #     t = int(time.time())
        #     script_dir = os.path.dirname(os.path.abspath(__file__))
        #     outdir = os.path.join(script_dir, f"vision_imgs_{uuid}")
        #     os.makedirs(outdir, exist_ok=True)
        #     cv2.imwrite(os.path.join(outdir, f"hoop_none_{t}.png"), image)
        #     cv2.imwrite(os.path.join(outdir, f"mask_none_{t}.png"), mask)
        return None

    H, W = image.shape[:2]
    diag = float(np.hypot(W, H))  # used to normalize center distance
    candidates: List[Dict] = []

    # ---------------------------------------------------
    # (4) Measure each candidate to build ranking metrics
    # ---------------------------------------------------
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue  # filter tiny noise

        # Centroid via moments. If m00==0, centroid is undefined.
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = float(M["m10"] / M["m00"])
        cy = float(M["m01"] / M["m00"])

        # Circularity: 4πA/P^2, 1.0 is a perfect circle, near 0.0 is a very skinny shape.
        perim = cv2.arcLength(c, True)
        if perim <= 0:
            continue
        circularity = float(4.0 * np.pi * area / (perim * perim))
        circularity = max(0.0, min(1.0, circularity))

        # Enclosing circle radius (for scale of erosion below)
        (_, _), er = cv2.minEnclosingCircle(c)
        er = float(er)

        # Ring-ness (hollow score): fill contour -> erode -> compare area
        # A ring loses less area under erosion than a solid disk of same outline.
        single = np.zeros((H, W), dtype=np.uint8)
        cv2.drawContours(single, [c], -1, 255, thickness=-1)  # filled blob

        # Erode with kernel ~10% of enclosing radius (>=1 and odd size)
        k = max(1, int(er * 0.10))
        if k % 2 == 0:
            k += 1
        kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        eroded = cv2.erode(single, kern, iterations=1)

        filled_area = float(np.count_nonzero(single)) + 1e-9  # avoid /0
        eroded_area = float(np.count_nonzero(eroded))
        ring_score = 1.0 - (eroded_area / filled_area)        # 0..1 (higher => more hollow)
        ring_score = max(0.0, min(1.0, ring_score))

        # Distance from the image center (normalize by diagonal so it's 0..~0.5)
        dx = cx - (W * 0.5)
        dy = cy - (H * 0.5)
        center_dist_norm = float(np.hypot(dx, dy) / diag)

        candidates.append({
            "contour": c,
            "cx": int(round(cx)),
            "cy": int(round(cy)),
            "area": float(area),
            "circularity": circularity,
            "ring_score": ring_score,
            "center_dist_norm": center_dist_norm
        })

    # If everything got filtered out
    if not candidates:
        # if debug:
        #     cv2.imshow("Hoop Detection", image)
        #     cv2.imshow("Hoop Mask", mask)
        #     cv2.waitKey(1)
        return None

    # ------------------------------------------
    # (5) Score candidates & pick the best hoop
    # ------------------------------------------
    max_area = max(c["area"] for c in candidates)
    for c in candidates:
        area_norm = c["area"] / max_area if max_area > 0 else 0.0
        c["score"] = (
            w_area  * area_norm +          # prefer bigger blobs (≈ closer hoops)
            w_circ  * c["circularity"] +   # prefer round shapes
            w_ring  * c["ring_score"] -    # prefer hollow (ring-like) blobs
            w_center* c["center_dist_norm"]# penalize distance from image center
        )

    best = max(candidates, key=lambda d: d["score"])
    cx, cy = best["cx"], best["cy"]
    best_contour = best["contour"]

    # -----------------------------------------
    # (6) Show / save visualization if requested
    # -----------------------------------------
    if debug or save_vision:
        vis = image.copy()

        # Draw every candidate thin blue (good to see alternatives)
        for c in candidates:
            cv2.drawContours(vis, [c["contour"]], -1, (255, 0, 0), 1)

        # Highlight the chosen one in green, mark center
        cv2.drawContours(vis, [best_contour], -1, (0, 255, 0), 2)
        cv2.circle(vis, (cx, cy), 6, (0, 255, 0), 2)
        cv2.drawMarker(vis, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 10, 2)

        if debug:
            cv2.imshow("Hoop Detection", vis)
            # cv2.imshow("Hoop Mask", mask)
            cv2.waitKey(1)

        # if save_vision:
        #     import time
        #     t = int(time.time())
        #     script_dir = os.path.dirname(os.path.abspath(__file__))
        #     outdir = os.path.join(script_dir, f"vision_imgs_{uuid}")
        #     os.makedirs(outdir, exist_ok=True)
        #     cv2.imwrite(os.path.join(outdir, f"hoop_{t}.png"), vis)
        #     cv2.imwrite(os.path.join(outdir, f"mask_{t}.png"), mask)

    # Keep your original return contract: zone_empty is False when we detected something
    return cx, cy, False


# try:
#     import torch
#     TORCH_AVAILABLE = True
# except ImportError:
#     TORCH_AVAILABLE = False
#     warnings.warn("PyTorch not available. Using fallback detection method.")
# # --- Paste key helpers from Ellipse-YOLO (shortened) ---
# def make_red_mask(hsv_img,
#                   lower_red1=(0, 50, 50), upper_red1=(10, 255, 255),
#                   lower_red2=(170, 50, 50), upper_red2=(180, 255, 255)):
#     mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
#     mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
#     mask = cv2.bitwise_or(mask1, mask2)
#     kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
#     return mask
# # --- import or define detect_hoops_ellipse_yolo and hough fallback ---
# from .ellipse_yolo_pipeline import detect_hoops_ellipse_yolo  # if separate file
# # If you keep this standalone, paste detect_hoops_ellipse_yolo() definition here.
# # =====================================================================
# # =====================================================================
# def find_hoop_ellipse_yolo(
#     image: np.ndarray,
#     lower_hoop1: np.ndarray = np.array([0, 50, 50]),
#     upper_hoop1: np.ndarray = np.array([10, 255, 255]),
#     lower_hoop2: np.ndarray = np.array([170, 50, 50]),
#     upper_hoop2: np.ndarray = np.array([180, 255, 255]),
#     uuid: str = "default",
#     debug: bool = False,
#     save_vision: bool = False,
#     use_model: bool = True,
#     model_path: Optional[str] = None,
#     use_red_mask: bool = True,
#     use_inner_edges: bool = True,
#     use_partial_circles: bool = True,
# ) -> Optional[Tuple[int, int, bool]]:
#     """
#     Find hoop center using Ellipse-YOLO (or fallback).
#     Returns (cx, cy, False) or None if not found.
#     """
#     # -------------------------------------------------------------
#     # (1) Convert to HSV and create mask (same as original find_hoop)
#     # -------------------------------------------------------------
#     hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#     mask = make_red_mask(hsv, lower_hoop1, upper_hoop1, lower_hoop2, upper_hoop2)
#     # -------------------------------------------------------------
#     # (2) Run Ellipse-YOLO detection (model or fallback automatically)
#     # -------------------------------------------------------------
#     result_img, detected_ellipses, nearest_idx, edge_img, _ = detect_hoops_ellipse_yolo(
#         image,
#         model_path=model_path,
#         use_model=use_model,
#         use_red_mask=use_red_mask,
#         lower_red1=tuple(lower_hoop1),
#         upper_red1=tuple(upper_hoop1),
#         lower_red2=tuple(lower_hoop2),
#         upper_red2=tuple(upper_hoop2),
#         use_inner_edges=use_inner_edges,
#         use_partial_circles=use_partial_circles,
#         draw_all_ellipses=False
#     )
#     # -------------------------------------------------------------
#     # (3) Handle no detections (identical return behavior)
#     # -------------------------------------------------------------
#     if not detected_ellipses or nearest_idx is None:
#         if debug:
#             cv2.imshow("Hoop Detection", image)
#             cv2.imshow("Hoop Mask", mask)
#             cv2.waitKey(1)
#         if save_vision:
#             os.makedirs(f"vision_imgs_{uuid}", exist_ok=True)
#             cv2.imwrite(f"vision_imgs_{uuid}/ellipse_none.png", image)
#             cv2.imwrite(f"vision_imgs_{uuid}/mask_none.png", mask)
#         return None
#     # -------------------------------------------------------------
#     # (4) Extract best ellipse (same order as original)
#     # -------------------------------------------------------------
#     best_ellipse = detected_ellipses[nearest_idx]
#     cx, cy = best_ellipse["center"]
#     # -------------------------------------------------------------
#     # (5) Visualization & saving (same order & style)
#     # -------------------------------------------------------------
#     if debug or save_vision:
#         vis = result_img.copy()
#         cv2.circle(vis, (cx, cy), 6, (0, 255, 0), 2)
#         cv2.drawMarker(vis, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 10, 2)
#         if debug:
#             cv2.imshow("Hoop Detection", vis)
#             cv2.imshow("Inner Edges", edge_img)
#             cv2.waitKey(1)
#         if save_vision:
#             os.makedirs(f"vision_imgs_{uuid}", exist_ok=True)
#             cv2.imwrite(f"vision_imgs_{uuid}/ellipse_result.png", vis)
#             cv2.imwrite(f"vision_imgs_{uuid}/ellipse_edges.png", edge_img)
#     return int(cx), int(cy), False


if __name__ == "__main__":
    # Test the functions
    image = cv2.imread("/Users/ethanyu/VSCodeProjects/monorepo/good_images/image_20250322_191023.png")
    print(find_payload(image, (140, 155, 50), (170, 255, 255), (140, 155, 50), (170, 255, 255), True))
