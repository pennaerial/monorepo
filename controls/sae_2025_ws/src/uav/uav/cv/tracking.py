# tracking.py
import cv2
import numpy as np
from typing import Optional, Tuple

def find_payload(
    image: np.ndarray,
    lower_zone: np.ndarray,
    upper_zone: np.ndarray,
    lower_payload: np.ndarray,
    upper_payload: np.ndarray,
    debug: bool = False
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
    
    Returns:
        Optional[Tuple[int, int, bool]]: A tuple (cx, cy, zone_empty) if detection is successful;
        otherwise, None.
    """
    # Convert to HSV color space.
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create zone mask and clean it using morphological operations.
    zone_mask = cv2.inRange(hsv_image, lower_zone, upper_zone)
    kernel = np.ones((5, 5), np.uint8)
    zone_mask = cv2.morphologyEx(zone_mask, cv2.MORPH_CLOSE, kernel)
    zone_mask = cv2.morphologyEx(zone_mask, cv2.MORPH_OPEN, kernel)

    # Find all external contours in the zone mask.
    contours, _ = cv2.findContours(zone_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
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
    
    if debug:
        vis_image = image.copy()

        cv2.drawContours(vis_image, [largest_zone_contour], -1, (0, 255, 0), 2)
        # If payload was detected, draw its contour.
        if payload_contours:
            cv2.drawContours(vis_image, [largest_payload_contour], -1, (0, 255, 0), 2)
        # Mark the detected center.
        cv2.circle(vis_image, (cx, cy), 5, (0, 0, 255), -1)
        cv2.namedWindow("Payload Tracking", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("Payload Tracking", vis_image)
        cv2.waitKey(1)
    
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