# tracking.py
import cv2
import numpy as np
from typing import Optional, Tuple

def find_payload(
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
    
    # Detect green payload
    green_mask = cv2.inRange(hsv_masked, lower_green, upper_green)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not green_contours:
        return None
        
    largest_green = max(green_contours, key=cv2.contourArea)
    M = cv2.moments(largest_green)
    
    if M["m00"] == 0:
        return None
        
    cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    
    if not debug:
        return cx, cy, None
    
    # Create visualization image
    vis_image = image.copy()
    cv2.rectangle(vis_image, (x, y), (x + w, y + h), (255, 0, 255), 2)
    cv2.drawContours(vis_image, [largest_green], -1, (0, 255, 0), 3)
    cv2.circle(vis_image, (cx, cy), 5, (0, 255, 0), -1)
    
    return cx, cy, vis_image

def compute_3d_vector(
    x: float, 
    y: float, 
    camera_info: np.ndarray,  # expected to be a 3x3 camera intrinsic matrix
    altitude: float
) -> Tuple[float, float, float]:
    """Convert pixel coordinates to a 3D direction vector."""
    K = np.array(camera_info)
    pixel_coords = np.array([x, y, 1.0])
    cam_coords = np.linalg.inv(K) @ pixel_coords
    
    # Convert to unit vector
    # direction = cam_coords / np.linalg.norm(cam_coords)
    real_world_vector = cam_coords * altitude
    return tuple(real_world_vector / np.linalg.norm(real_world_vector))