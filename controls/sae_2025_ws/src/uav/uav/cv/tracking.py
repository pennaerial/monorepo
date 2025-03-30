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
    Detect payload in image using color thresholding with improved shadow resistance.
    """
    blurred = cv2.GaussianBlur(image, (7, 7), 0)
    
    # Apply LAB-based CLAHE enhancement
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    
    # Apply CLAHE only to the L channel (luminance)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    l = clahe.apply(l)
    
    # Merge back the channels
    lab = cv2.merge([l, a, b])
    enhanced_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    
    # Convert to HSV for color thresholding
    hsv_image = cv2.cvtColor(enhanced_image, cv2.COLOR_BGR2HSV)
    
    # Create zone mask
    zone_mask = cv2.inRange(hsv_image, lower_zone, upper_zone)
    
    # Sequential morphological operations 
    small_kernel = np.ones((5, 5), np.uint8)
    medium_kernel = np.ones((9, 9), np.uint8)
    
    # First remove small noise
    zone_mask = cv2.morphologyEx(zone_mask, cv2.MORPH_OPEN, small_kernel)
    
    # Then close small gaps
    zone_mask = cv2.morphologyEx(zone_mask, cv2.MORPH_CLOSE, medium_kernel)
    
    # Find contours
    contours, _ = cv2.findContours(zone_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # Remove contour size filtering

    largest_zone_contour = max(contours, key=cv2.contourArea)
    
    # Use convex hull for more stable shape
    hull = cv2.convexHull(largest_zone_contour)
    
    # Create a filled mask from the hull
    zone_filled_mask = np.zeros_like(zone_mask)
    cv2.drawContours(zone_filled_mask, [hull], -1, 255, thickness=cv2.FILLED)
    
    # Detect payload areas in the HSV image
    payload_mask_full = cv2.inRange(hsv_image, lower_payload, upper_payload)
    
    # Restrict the payload mask to the zone-filled region
    payload_mask = cv2.bitwise_and(payload_mask_full, zone_filled_mask)
    
    # Clean the resulting payload mask
    payload_mask = cv2.morphologyEx(payload_mask, cv2.MORPH_OPEN, small_kernel)
    payload_mask = cv2.morphologyEx(payload_mask, cv2.MORPH_CLOSE, small_kernel)
    
    # Find external contours in the payload mask
    payload_contours, _ = cv2.findContours(payload_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_payload_contour = None
    
    if payload_contours and lower_payload is not lower_zone and upper_payload is not upper_zone: 
        # Use the largest payload contour without area filtering
        largest_payload_contour = max(payload_contours, key=cv2.contourArea)
        M_payload = cv2.moments(largest_payload_contour)
        if M_payload["m00"] == 0:
            return None
        cx, cy = int(M_payload["m10"] / M_payload["m00"]), int(M_payload["m01"] / M_payload["m00"])
    else:
        # Use distance transform for better center estimation
        dist_transform = cv2.distanceTransform(zone_filled_mask, cv2.DIST_L2, 5)
        _, max_val, _, max_loc = cv2.minMaxLoc(dist_transform)
        cx, cy = max_loc
    
    # Debug section remains the same
    if debug:
        # Create visualization image
        vis_image = image.copy()
        
        # Draw the enhanced image for debugging
        h, w = image.shape[:2]
        debug_vis = np.zeros((h, w*2, 3), dtype=np.uint8)
        debug_vis[:, :w] = vis_image
        debug_vis[:, w:] = enhanced_image
        
        # Draw zone contour
        cv2.drawContours(vis_image, [hull], -1, (0, 255, 0), 2)
        
        # If payload was detected, draw its contour
        if payload_contours and largest_payload_contour is not None:
            cv2.drawContours(vis_image, [largest_payload_contour], -1, (0, 0, 255), 2)
        
        # Mark the detected center
        cv2.circle(vis_image, (cx, cy), 5, (0, 0, 255), -1)
        
        # Show masks for debugging
        cv2.imshow("Original Image", image)
        cv2.imshow("Enhanced Image", enhanced_image)
        cv2.imshow("Zone Mask", cv2.cvtColor(zone_mask, cv2.COLOR_GRAY2BGR))
        cv2.imshow("Payload Mask", cv2.cvtColor(payload_mask, cv2.COLOR_GRAY2BGR))
        cv2.imshow("Payload Tracking Result", vis_image)
        cv2.waitKey(0)
    
    return cx, cy, not bool(payload_contours)

def find_dlz(
    image: np.ndarray,
    lower_pink: np.ndarray,
    upper_pink: np.ndarray,
    lower_green: np.ndarray,
    upper_green: np.ndarray,
    debug: bool = False
) -> Optional[Tuple[int, int, np.ndarray]]:
    """
    Detect payload in image using color thresholding with improved shadow resistance.
    """
    # Use Gaussian blur instead of bilateral filter for better noise reduction
    blurred = cv2.GaussianBlur(image, (7, 7), 0)
    
    # Apply LAB-based CLAHE enhancement
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    
    # Apply CLAHE only to the L channel (luminance)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(16, 16))
    l = clahe.apply(l)
    
    # Merge back the channels
    lab = cv2.merge([l, a, b])
    enhanced_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    
    # Convert to HSV for color thresholding
    hsv_image = cv2.cvtColor(enhanced_image, cv2.COLOR_BGR2HSV)
    
    # Create pink mask
    pink_mask = cv2.inRange(hsv_image, lower_pink, upper_pink)
    
    # Removed the connected component analysis and filtering
    
    # Apply morphological operations
    medium_kernel = np.ones((9, 9), np.uint8)
    pink_mask = cv2.morphologyEx(pink_mask, cv2.MORPH_CLOSE, medium_kernel)
    pink_mask = cv2.morphologyEx(pink_mask, cv2.MORPH_OPEN, medium_kernel)
    
    # Find contours
    contours, _ = cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    
    # Use the largest contour without area filtering
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Use convex hull for more stable shape
    hull = cv2.convexHull(largest_contour)
    
    # Create masked region for potential green detection
    pink_mask_filled = np.zeros_like(pink_mask)
    cv2.drawContours(pink_mask_filled, [hull], -1, 255, thickness=cv2.FILLED)
    
    # Get bounding box for visualization
    x, y, w, h = cv2.boundingRect(hull)
    
    # Calculate moments for centroid
    M = cv2.moments(hull)
    if M["m00"] == 0:
        return None
    
    cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    
    # Check for green inside the pink area if thresholds are provided
    green_detected = False
    if lower_green is not None and upper_green is not None:
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        green_in_pink = cv2.bitwise_and(green_mask, pink_mask_filled)
        
        # Check if green is detected within pink area (removed min_component_area)
        green_pixels = cv2.countNonZero(green_in_pink)
        green_detected = green_pixels > 0
    
    
    if not debug:
        return cx, cy, None
    
    # Create visualization
    vis_image = image.copy()
    
    # Draw contour and center point
    cv2.rectangle(vis_image, (x, y), (x + w, y + h), (255, 0, 255), 2)
    cv2.drawContours(vis_image, [hull], -1, (0, 255, 0), 3)
    cv2.circle(vis_image, (cx, cy), 5, (0, 0, 255), -1)
    
    # Add text indicating if green was detected
    if green_detected:
        cv2.putText(vis_image, "Green detected", (x, y-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Create a side-by-side visualization
    h, w = image.shape[:2]
    debug_vis = np.zeros((h, w*2, 3), dtype=np.uint8)
    debug_vis[:, :w] = vis_image
    debug_vis[:, w:] = enhanced_image
    
    # Show masks for debugging
    if debug:
        cv2.imshow("Original Image", image)
        cv2.imshow("Enhanced Image", enhanced_image)
        cv2.imshow("Pink Mask", cv2.cvtColor(pink_mask, cv2.COLOR_GRAY2BGR))
        cv2.imshow("DLZ Detection Result", vis_image)
        cv2.waitKey(0)
    
    return cx, cy, debug_vis

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

if __name__ == "__main__":
    # Test the functions
    image = cv2.imread("/Users/arjunverma/ProjE/image_20250322_191030.png")
    
    # Define HSV thresholds for pink
    # Note: HSV values in OpenCV are H: 0-179, S: 0-255, V: 0-255
    # Pink typically has H values around 140-170 in OpenCV's HSV space
    lower_pink = np.array([140, 100, 100])
    upper_pink = np.array([170, 255, 255])
    
    # Call with debug mode to see visualizations
    result = find_payload(image, lower_pink, upper_pink, lower_pink, upper_pink, True)
    
    if result:
        cx, cy, zone_empty = result
        print(f"Detected center at ({cx}, {cy}), Zone empty: {zone_empty}")
    else:
        

        print("Detection failed")
