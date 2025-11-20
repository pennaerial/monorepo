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
    # First, filter out very dark (black) regions so we ignore blades/arms.
    # Convert to HSV and build a mask of non-black pixels based on value (V).
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    v_channel = hsv[:, :, 2]

    # Pixels with V below ~40 are considered "black" and removed.
    non_black_mask = cv2.inRange(v_channel, 40, 255)

    # Clean up the non-black mask.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    non_black_mask = cv2.morphologyEx(non_black_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    non_black_mask = cv2.morphologyEx(non_black_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    # (Mask is used internally; debug visualization is handled at the end as a single view.)

    # Convert to grayscale and restrict processing to non-black regions only.
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.bitwise_and(gray, gray, mask=non_black_mask)

    h, w = gray.shape
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Multi-threshold edge detection (on non-black regions)
    edges1 = cv2.Canny(blurred, 30, 100)
    edges2 = cv2.Canny(blurred, 50, 150)
    edges = cv2.bitwise_or(edges1, edges2)
    
    # Morphological operations to close gaps
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
    dilated = cv2.dilate(closed, kernel, iterations=1)

    # Find contours
    contours, _ = cv2.findContours(dilated, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        if debug or save_vision:
            vis_image = image.copy()
        if save_vision:
            import time
            timestamp = int(time.time())
            path = os.path.expanduser(f"~/vision_imgs/{uuid}")
            os.makedirs(path, exist_ok=True)
            cv2.imwrite(os.path.join(path, f"hoop_none_{timestamp}.png"), vis_image)
        # Single debug window is handled uniformly at the end when a hoop is found.
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
        if save_vision:
            import time
            timestamp = int(time.time())
            path = os.path.expanduser(f"~/vision_imgs/{uuid}")
            os.makedirs(path, exist_ok=True)
            cv2.imwrite(os.path.join(path, f"hoop_none_{timestamp}.png"), vis_image)
        return None
    
    # Sort by size and select the largest hoop
    hoop_candidates.sort(key=lambda x: x['size'], reverse=True)
    closest_hoop = hoop_candidates[0]
    center = closest_hoop['center']

    # Visualization: single debug view with outlines over the original image.
    if debug or save_vision:
        vis_image = image.copy()

        # Draw the selected hoop in green.
        cv2.ellipse(vis_image, closest_hoop['ellipse'], (0, 255, 0), 3)
        cv2.circle(vis_image, center, 6, (0, 0, 255), -1)

        # Optional: draw other candidates in orange.
        if len(hoop_candidates) > 1:
            for hoop in hoop_candidates[1:]:
                cv2.ellipse(vis_image, hoop['ellipse'], (0, 165, 255), 1)

        # Draw a crosshair at the center.
        cv2.line(vis_image, (center[0] - 20, center[1]), (center[0] + 20, center[1]), (0, 0, 255), 2)
        cv2.line(vis_image, (center[0], center[1] - 20), (center[0], center[1] + 20), (0, 0, 255), 2)

        # Add text label
        text = f"Hoop: ({center[0]}, {center[1]})"
        cv2.putText(vis_image, text, (center[0] + 25, center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    if debug:
        cv2.imshow("Hoop Debug", vis_image)
        cv2.waitKey(1)

    if save_vision:
        import time
        timestamp = int(time.time())
        path = os.path.expanduser(f"~/vision_imgs/{uuid}")
        os.makedirs(path, exist_ok=True)
        cv2.imwrite(os.path.join(path, f"hoop_{timestamp}.png"), vis_image)
    
    return center