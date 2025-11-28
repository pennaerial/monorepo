# TEAM 6 IN PROGRESS HOOP DETECTION
import cv2
import numpy as np
from typing import Optional, Tuple
import os

def detect_hoop(
    image: np.ndarray,
    uuid: str,
    debug: bool = False,
    save_vision: bool = False
) -> Optional[Tuple[int, int]]:
    """
    Detect hoop in image using edge detection and ellipse fitting.
    
    Args:
        image (np.ndarray): Input BGR image.
        uuid (str): Unique identifier for saving debug images.
        debug (bool): If True, display debug windows.
        save_vision (bool): If True, save visualization images.
    
    Returns:
        Optional[Tuple[int, int]]: A tuple (cx, cy) if detection is successful;
        otherwise, None.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    edges1 = cv2.Canny(blurred, 30, 100)
    edges2 = cv2.Canny(blurred, 50, 150)
    edges = cv2.bitwise_or(edges1, edges2)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
    dilated = cv2.dilate(closed, kernel, iterations=1)

    contours, hierarchy = cv2.findContours(dilated, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    hoop_candidates = []
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area < 1000 or area > w * h * 0.5:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter < 100:
            continue

        circularity = 4 * np.pi * area / (perimeter ** 2) if perimeter > 0 else 0
        if len(contour) >= 5:
            try:
                ellipse = cv2.fitEllipse(contour)
                (x, y), (MA, ma), angle = ellipse
                if x < 0 or x >= w or y < 0 or y >= h:
                    continue
                if MA > 0:
                    aspect_ratio = ma / MA
                else:
                    continue

                if 0.4 < aspect_ratio < 1.6 and circularity > 0.3:
                    center = (int(x), int(y))
                    cam_pos = (w // 2, h)
                    # Calculate Euclidean distance without scipy
                    dist = np.sqrt((center[0] - cam_pos[0])**2 + (center[1] - cam_pos[1])**2)
                    size_score = (MA + ma) / 2

                    hoop_candidates.append({
                        'contour': contour,
                        'ellipse': ellipse,
                        'center': center,
                        'distance': dist,
                        'circularity': circularity,
                        'aspect_ratio': aspect_ratio,
                        'area': area,
                        'size': size_score
                    })
            except:
                continue

    if not hoop_candidates:
        if debug or save_vision:
            vis_image = image.copy()
            large_contours = [c for c in contours if cv2.contourArea(c) > 500]
            cv2.drawContours(vis_image, large_contours, -1, (0, 0, 255), 2)
            cv2.putText(vis_image, "No Hoop Detected", (50, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if debug:
            cv2.namedWindow("Hoop Tracking", cv2.WINDOW_AUTOSIZE)
            cv2.imshow("Hoop Tracking", vis_image)
            cv2.imshow("Hoop Edges", dilated)
            cv2.waitKey(1)
        if save_vision:
            import time
            time_stamp = int(time.time())
            path = os.path.expanduser(f"~/vision_imgs/{uuid}")
            cv2.imwrite(os.path.join(path, f"hoop_{time_stamp}.png"), vis_image)
            cv2.imwrite(os.path.join(path, f"hoop_edges_{time_stamp}.png"), dilated)
        return None

    # Sort by size and select the largest
    hoop_candidates.sort(key=lambda x: x['size'], reverse=True)
    closest_hoop = hoop_candidates[0]
    center = closest_hoop['center']

    if debug or save_vision:
        vis_image = image.copy()
        # Draw all candidates
        for idx, hoop in enumerate(hoop_candidates):
            color = (0, 255, 0) if idx == 0 else (255, 165, 0)
            cv2.ellipse(vis_image, hoop['ellipse'], color, 2)
        
        # Highlight the selected hoop
        cv2.ellipse(vis_image, closest_hoop['ellipse'], (0, 255, 0), 4)
        cv2.circle(vis_image, center, 10, (0, 0, 255), -1)
        cv2.line(vis_image, (center[0] - 30, center[1]), (center[0] + 30, center[1]), (0, 0, 255), 3)
        cv2.line(vis_image, (center[0], center[1] - 30), (center[0], center[1] + 30), (0, 0, 255), 3)
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
        time_stamp = int(time.time())
        path = os.path.expanduser(f"~/vision_imgs/{uuid}")
        cv2.imwrite(os.path.join(path, f"hoop_{time_stamp}.png"), vis_image)
        cv2.imwrite(os.path.join(path, f"hoop_edges_{time_stamp}.png"), dilated)

    return center[0], center[1]