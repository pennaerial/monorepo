# hoop_detection.py
import cv2
import numpy as np
from typing import Optional, Tuple, List

def detect_hoops(
    image: np.ndarray,
    debug: bool = False,
    save_vision: bool = False,
    uuid: str = "",
    detect_all: bool = False,
    return_visualization: bool = False
) -> Tuple[Optional[Tuple[int, int]], List[Tuple[int, int]], Optional[np.ndarray]]:
    """
    Detect hoops (circles) in an image using edge detection and contour analysis.
    
    Args:
        image (np.ndarray): Input BGR image.
        debug (bool): If True, print debug information.
        save_vision (bool): If True, save visualization images.
        uuid (str): Unique identifier for saving images.
        detect_all (bool): If True, return all detected hoops; if False, return only closest.
        return_visualization (bool): If True, return visualization image with annotations.
    
    Returns:
        Tuple[Optional[Tuple[int, int]], List[Tuple[int, int]], Optional[np.ndarray]]:
            - First element: Center of closest hoop (x, y) or None if no hoops detected
            - Second element: List of all detected hoop centers [(x, y), ...]
            - Third element: Visualization image with annotations (if return_visualization=True)
    """
    img_rgb = image.copy()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    
    # Apply bilateral filter for noise reduction
    filtered = cv2.bilateralFilter(gray, 9, 75, 75)
    blurred = cv2.GaussianBlur(filtered, (5, 5), 1)
    
    # Dual Canny edge detection
    edges1 = cv2.Canny(blurred, 40, 120)
    edges2 = cv2.Canny(blurred, 60, 180)
    edges = cv2.bitwise_or(edges1, edges2)
    
    # Morphological operations
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel_close, iterations=1)
    dilated = cv2.dilate(closed, kernel_dilate, iterations=1)
    
    # Find contours
    contours, hierarchy = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if debug:
        print(f"Found {len(contours)} total contours")
    
    # Hough circle detection as additional method
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=100,
        param1=50,
        param2=50,
        minRadius=50,
        maxRadius=min(h, w) // 3
    )
    
    hough_candidates = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            x, y, r = circle
            if 0 <= x < w and 0 <= y < h:
                hough_candidates.append({
                    'center': (int(x), int(y)),
                    'radius': int(r),
                    'method': 'hough'
                })
        if debug:
            print(f"Hough detected {len(hough_candidates)} circles")
    
    # Contour-based detection
    contour_candidates = []
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
                
                if 0.5 < aspect_ratio < 1.5 and circularity > 0.4:
                    center = (int(x), int(y))
                    cam_pos = (w // 2, h)
                    dist = np.linalg.norm(np.array(center) - np.array(cam_pos))
                    size_score = (MA + ma) / 2
                    has_parent = hierarchy[0][i][3] != -1
                    
                    contour_candidates.append({
                        'contour': contour,
                        'ellipse': ellipse,
                        'center': center,
                        'distance': dist,
                        'circularity': circularity,
                        'aspect_ratio': aspect_ratio,
                        'area': area,
                        'size': size_score,
                        'has_parent': has_parent,
                        'index': i,
                        'method': 'contour'
                    })
                    
                    if debug:
                        print(f"Contour candidate {len(contour_candidates)}: center={center}, "
                              f"area={area:.0f}, circ={circularity:.2f}, "
                              f"aspect={aspect_ratio:.2f}")
            except:
                continue
    
    # Combine Hough and contour candidates
    all_candidates = contour_candidates.copy()
    for hc in hough_candidates:
        mask = np.zeros_like(gray)
        cv2.circle(mask, hc['center'], hc['radius'], 255, 2)
        overlap = cv2.bitwise_and(edges, mask)
        edge_pixels = np.count_nonzero(overlap)
        circle_perimeter = 2 * np.pi * hc['radius']
        edge_coverage = edge_pixels / circle_perimeter if circle_perimeter > 0 else 0
        
        if edge_coverage > 0.3:
            overlaps = False
            for cc in contour_candidates:
                dist = np.linalg.norm(np.array(hc['center']) - np.array(cc['center']))
                if dist < hc['radius'] * 0.5:
                    overlaps = True
                    break
            
            if not overlaps:
                cam_pos = (w // 2, h)
                all_candidates.append({
                    'center': hc['center'],
                    'distance': np.linalg.norm(np.array(hc['center']) - np.array(cam_pos)),
                    'size': hc['radius'] * 2,
                    'circularity': edge_coverage,
                    'radius': hc['radius'],
                    'method': 'hough',
                    'index': -1
                })
    
    if debug:
        print(f"Total candidates: {len(all_candidates)} (contour: {len(contour_candidates)}, hough: {len(hough_candidates)})")
    
    if not all_candidates:
        if debug:
            print("No hoops detected!")
        return None, []
    
    # Filter overlapping candidates
    filtered_candidates = []
    used_indices = set()
    all_candidates.sort(key=lambda x: (x['circularity'], x['size']), reverse=True)
    
    for hoop in all_candidates:
        hoop_idx = hoop.get('index', -1)
        if hoop_idx in used_indices and hoop_idx != -1:
            continue
        
        is_overlapping = False
        selected_to_replace = None
        
        for idx, selected in enumerate(filtered_candidates):
            center_dist = np.linalg.norm(np.array(hoop['center']) - np.array(selected['center']))
            avg_radius = (hoop['size'] + selected['size']) / 4
            
            if center_dist < avg_radius * 0.8:
                is_overlapping = True
                if hoop['circularity'] > selected['circularity']:
                    selected_to_replace = idx
                break
        
        if selected_to_replace is not None:
            filtered_candidates[selected_to_replace] = hoop
            if hoop_idx != -1:
                used_indices.add(hoop_idx)
        elif not is_overlapping:
            filtered_candidates.append(hoop)
            if hoop_idx != -1:
                used_indices.add(hoop_idx)
    
    if debug:
        print(f"After filtering: {len(filtered_candidates)} unique hoops")
    
    if not filtered_candidates:
        if debug:
            print("No hoops after filtering!")
        vis_img = img_rgb.copy() if return_visualization else None
        return None, [], vis_img
    
    # Sort by size (largest/closest first)
    filtered_candidates.sort(key=lambda x: x['size'], reverse=True)
    
    # Extract centers
    all_centers = [hoop['center'] for hoop in filtered_candidates]
    closest_center = all_centers[0] if all_centers else None
    
    # Create visualization image
    vis_img = None
    if return_visualization or save_vision:
        vis_img = img_rgb.copy()
        
        # Draw all detected hoops
        for idx, hoop in enumerate(filtered_candidates):
            # Color: green for closest, orange for others
            color = (0, 255, 0) if idx == 0 else (255, 165, 0)
            thickness = 4 if idx == 0 else 2
            
            # Draw contour if available (more detailed visualization)
            if 'contour' in hoop:
                cv2.drawContours(vis_img, [hoop['contour']], -1, color, thickness)
            
            if hoop['method'] == 'hough':
                cv2.circle(vis_img, hoop['center'], hoop['radius'], color, thickness)
            else:
                cv2.ellipse(vis_img, hoop['ellipse'], color, thickness)
            
            # Draw center point
            cv2.circle(vis_img, hoop['center'], 10, (255, 0, 0), -1)
            
            # Draw crosshair at center
            center = hoop['center']
            cv2.line(vis_img, (center[0] - 30, center[1]), (center[0] + 30, center[1]), (255, 0, 0), 2)
            cv2.line(vis_img, (center[0], center[1] - 30), (center[0], center[1] + 30), (255, 0, 0), 2)
            
            # Add label
            label = f"Hoop {idx+1}" if idx > 0 else "Target"
            cv2.putText(vis_img, label, (center[0] + 40, center[1] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Draw image center (camera center)
        h, w = vis_img.shape[:2]
        cam_center = (w // 2, h)
        cv2.circle(vis_img, cam_center, 5, (0, 0, 255), -1)
        cv2.line(vis_img, (cam_center[0] - 20, cam_center[1]), (cam_center[0] + 20, cam_center[1]), (0, 0, 255), 2)
        cv2.line(vis_img, (cam_center[0], cam_center[1] - 20), (cam_center[0], cam_center[1] + 20), (0, 0, 255), 2)
        
        # Save if requested
        if save_vision:
            import time
            import os
            timestamp = int(time.time())
            path = os.path.expanduser(f"~/vision_imgs/{uuid}")
            os.makedirs(path, exist_ok=True)
            cv2.imwrite(os.path.join(path, f"hoop_detection_{timestamp}.png"), vis_img)
    
    if detect_all:
        return closest_center, all_centers, vis_img
    else:
        return closest_center, [], vis_img

