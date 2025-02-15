# tracking.py
import cv2
import numpy as np

def find_payload(image, lower_pink, upper_pink, lower_green, upper_green):
    """
    Detect payload in image using color thresholding.
    Returns raw detection coordinates and visualization image.
    
    Args:
        image: Input BGR image
        lower_pink/upper_pink: HSV threshold values for pink marker
        lower_green/upper_green: HSV threshold values for green payload
    
    Returns:
        Tuple of (cx, cy, visualization_image) or None if no detection
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
    
    # Create visualization image
    vis_image = image.copy()
    cv2.rectangle(vis_image, (x, y), (x + w, y + h), (255, 0, 255), 2)
    cv2.drawContours(vis_image, [largest_green], -1, (0, 255, 0), 3)
    cv2.circle(vis_image, (cx, cy), 5, (0, 255, 0), -1)
    
    return cx, cy, vis_image
