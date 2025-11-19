import cv2
import numpy as np
from typing import Optional, Tuple
import os


def find_hoop(
    image: np.ndarray,
    uuid: str,
    debug: bool = False,
    save_vision: bool = False,
) -> Optional[Tuple[int, int]]:
    """
    Detect hoop in image using edge detection and contour analysis.
    
    Args:
        image (np.ndarray): Input BGR image.
        uuid (str): Unique identifier for saving vision images.
        debug (bool): If True, display visualization windows.
        save_vision (bool): If True, save the visualization image.
    
    Returns:
        Optional[Tuple[int, int]]: A tuple (cx, cy) if detection is successful;
        otherwise, None.
    """
    # Convert to grayscale and preprocess
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    
    # Multi-threshold edge detection
    edges1 = cv2.Canny(blurred, 30, 100)
    edges2 = cv2.Canny(blurred, 50, 150)
    edges = cv2.bitwise_or(edges1, edges2)
    
    # Morphological operations to close gaps
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
    dilated = cv2.dilate(closed, kernel, iterations=1)

    # Find contours
    contours, _ = cv2.findContours(dilated, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        if debug or save_vision:
            vis_image = image.copy()
        if debug:
            cv2.imshow("Hoop Tracking", vis_image)
            cv2.waitKey(1)
        if save_vision:
            import time
            timestamp = int(time.time())
            path = os.path.expanduser(f"~/vision_imgs/{uuid}")
            os.makedirs(path, exist_ok=True)
            cv2.imwrite(os.path.join(path, f"hoop_{timestamp}.png"), vis_image)
        return None

    # Analyze contours to find hoop candidates
    hoop_candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 1000 or area > w * h * 0.5:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter < 100:
            continue

        circularity = 4 * np.pi * area / (perimeter ** 2) if perimeter > 0 else 0
        
        # Fit ellipse if contour has enough points
        if len(contour) >= 5:
            try:
                ellipse = cv2.fitEllipse(contour)
                (x, y), (MA, ma), angle = ellipse
                
                # Validate ellipse is within image bounds
                if x < 0 or x >= w or y < 0 or y >= h:
                    continue
                    
                if MA > 0:
                    aspect_ratio = ma / MA
                else:
                    continue

                # Filter by circularity and aspect ratio
                if 0.4 < aspect_ratio < 1.6 and circularity > 0.3:
                    center = (int(x), int(y))
                    size_score = (MA + ma) / 2

                    hoop_candidates.append({
                        'contour': contour,
                        'ellipse': ellipse,
                        'center': center,
                        'circularity': circularity,
                        'aspect_ratio': aspect_ratio,
                        'area': area,
                        'size': size_score
                    })
            except:
                continue

    # Return None if no candidates found
    if not hoop_candidates:
        if debug or save_vision:
            vis_image = image.copy()
        if debug:
            cv2.imshow("Hoop Tracking", vis_image)
            cv2.imshow("Hoop Edges", dilated)
            cv2.waitKey(1)
        if save_vision:
            import time
            timestamp = int(time.time())
            path = os.path.expanduser(f"~/vision_imgs/{uuid}")
            os.makedirs(path, exist_ok=True)
            cv2.imwrite(os.path.join(path, f"hoop_{timestamp}.png"), vis_image)
            cv2.imwrite(os.path.join(path, f"hoop_edges_{timestamp}.png"), dilated)
        return None
    
    # Sort by size and select the largest hoop
    hoop_candidates.sort(key=lambda x: x['size'], reverse=True)
    closest_hoop = hoop_candidates[0]
    center = closest_hoop['center']

    # Visualization
    if debug or save_vision:
        vis_image = image.copy()
        
        # Draw all candidates if there are multiple
        if len(hoop_candidates) > 1:
            for idx, hoop in enumerate(hoop_candidates[1:]):
                cv2.ellipse(vis_image, hoop['ellipse'], (255, 165, 0), 2)
        
        # Draw the selected hoop
        cv2.ellipse(vis_image, closest_hoop['ellipse'], (0, 255, 0), 4)
        cv2.circle(vis_image, center, 10, (0, 0, 255), -1)
        
        # Draw crosshair
        cv2.line(vis_image, (center[0] - 30, center[1]), (center[0] + 30, center[1]), (0, 0, 255), 3)
        cv2.line(vis_image, (center[0], center[1] - 30), (center[0], center[1] + 30), (0, 0, 255), 3)
        
        # Add text label
        text = f"Center: ({center[0]}, {center[1]})"
        cv2.putText(vis_image, text, (center[0] + 40, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    
    if debug:
        cv2.namedWindow("Hoop Tracking", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("Hoop Tracking", vis_image)
        cv2.imshow("Hoop Edges", dilated)
        cv2.waitKey(1)
    
    if save_vision:
        import time
        timestamp = int(time.time())
        path = os.path.expanduser(f"~/vision_imgs/{uuid}")
        os.makedirs(path, exist_ok=True)
        cv2.imwrite(os.path.join(path, f"hoop_{timestamp}.png"), vis_image)
        cv2.imwrite(os.path.join(path, f"hoop_edges_{timestamp}.png"), dilated)
    
    return center