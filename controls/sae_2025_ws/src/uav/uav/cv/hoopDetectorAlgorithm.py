import numpy as np
import cv2
import math
from typing import Optional, Tuple, List, Dict, Any, Union
import os
from collections import deque
import random

# --- Configuration ---
RING_RAD_M_DEFAULT = 0.100  # Basketball hoop radius
BASE_W, BASE_H = 960, 540
HFOV_DEG_DEFAULT = 78.0

# --- Temporal Tracking Parameters (REMOVED) ---

# --- HSV Filter Parameters ---
# Assuming hoop is RED/ORANGE, Hue is around 0/360
HUE_TOLERANCE_DEG = 10            # Tolerance around 0 degree for red color
HUE_LOW_1 = 0                     # Part 1: 0 to HUE_TOLERANCE_DEG
HUE_HIGH_1 = HUE_TOLERANCE_DEG
HUE_LOW_2 = 180 - HUE_TOLERANCE_DEG # Part 2: 360-TOLERANCE_DEG (OpenCV maps 0-360 to 0-180)
HUE_HIGH_2 = 180                  # Max hue value in OpenCV
SAT_LOW = 50                      # Keeping Saturation/Value wide for shadows
SAT_HIGH = 255
VAL_LOW = 50
VAL_HIGH = 255

# --- Ellipse Detection Parameters ---
MIN_CONTOUR_POINTS = 30
MIN_CONTOUR_AREA = 25
MIN_AXIS_LENGTH = 5
MAX_AXIS_LENGTH = 1500
ELLIPSE_FIT_ERROR_THRESHOLD = 10.0
RANSAC_ITERATIONS = 300
RANSAC_MIN_POINTS = 5
RANSAC_SAMPLE_POINTS = 10
RANSAC_INLIER_THRESHOLD = 7.0

# --- HoopTrackerState Class REMOVED as requested ---


# --- Geometry Helpers ---
def get_default_K(W=BASE_W, H=BASE_H, hfov_deg=HFOV_DEG_DEFAULT):
    """Calculates K matrix."""
    hfov = math.radians(hfov_deg)
    fx = (W/2) / math.tan(hfov/2)
    fy = fx
    cx, cy = W/2, H/2
    return np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], np.float64)

# --- Ellipse/Pose Drawing Helper (NEW) ---
def _draw_hoop_pose_on_frame(frame, K, center_3d, normal_3d, ellipse, color, text_info=None):
    """Internal helper to draw 2D ellipse, center, and projected normal."""
    # Draw ellipse
    cv2.ellipse(frame, ellipse, color, 2, cv2.LINE_AA)
    
    # Draw center
    cx, cy = int(round(ellipse[0][0])), int(round(ellipse[0][1]))
    cv2.circle(frame, (cx, cy), 5, color, -1)
    
    # Draw normal line
    NORMAL_LINE_LENGTH = 0.5
    P_end_3d = center_3d + NORMAL_LINE_LENGTH * normal_3d
    
    rvec_zero = np.zeros((3,1), dtype=np.float64)
    tvec_zero = np.zeros((3,1), dtype=np.float64)
    points_3d = np.vstack([center_3d, P_end_3d]).reshape(-1, 1, 3)
    
    pts2d, _ = cv2.projectPoints(points_3d, rvec_zero, tvec_zero, K, None)
    
    p_center_2d = tuple(np.int32(pts2d[0, 0]))
    p_end_2d = tuple(np.int32(pts2d[1, 0]))
    
    cv2.line(frame, p_center_2d, p_end_2d, color, 2, cv2.LINE_AA)
    
    if text_info:
        for i, text in enumerate(text_info):
            # Draw text near the center
            cv2.putText(frame, text, (cx + 10, cy + i * 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

# --- Simple Ellipse Fitting with Validation (Unchanged) ---
def fit_ellipse_with_validation(points):
    """
    Try multiple ellipse fitting methods and return the best one.
    """
    if len(points) < 5:
        return None, float('inf'), "too_few_points"
    
    best_ellipse = None
    best_error = float('inf')
    best_method = None
    
    # Method 1: Standard fitEllipse
    try:
        ellipse = cv2.fitEllipse(points)
        error = calculate_ellipse_error_simple(points, ellipse)
        if error < best_error:
            best_error = error
            best_ellipse = ellipse
            best_method = "fitEllipse"
    except:
        pass
    
    # Method 2: fitEllipseAMS (more robust)
    try:
        ellipse = cv2.fitEllipseAMS(points)
        error = calculate_ellipse_error_simple(points, ellipse)
        if error < best_error:
            best_error = error
            best_ellipse = ellipse
            best_method = "fitEllipseAMS"
    except:
        pass
    
    # Method 3: fitEllipseDirect (least squares)
    try:
        ellipse = cv2.fitEllipseDirect(points)
        error = calculate_ellipse_error_simple(points, ellipse)
        if error < best_error:
            best_error = error
            best_ellipse = ellipse
            best_method = "fitEllipseDirect"
    except:
        pass
    
    return best_ellipse, best_error, best_method

def calculate_ellipse_error_simple(points, ellipse):
    """Simplified error calculation."""
    (cx, cy), (MA, ma), angle = ellipse
    
    # Basic validation
    if MA < 1e-6 or ma < 1e-6:
        return float('inf')
    
    # Sample some points on the ellipse and find nearest distances
    num_samples = min(36, len(points))
    angles = np.linspace(0, 2*np.pi, num_samples)
    
    # Generate ellipse points
    a, b = MA/2, ma/2
    ellipse_points = []
    angle_rad = np.radians(angle)
    cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
    
    for theta in angles:
        x = a * np.cos(theta)
        y = b * np.sin(theta)
        # Rotate
        x_rot = x * cos_a - y * sin_a + cx
        y_rot = x * sin_a + y * cos_a + cy
        ellipse_points.append([x_rot, y_rot])
    
    ellipse_points = np.array(ellipse_points)
    
    # For each contour point, find distance to nearest ellipse point
    total_error = 0
    for pt in points:
        distances = np.linalg.norm(ellipse_points - pt, axis=1)
        total_error += np.min(distances)
    
    return total_error / len(points)

# --- UPDATED Detection using HSV Mask with Erosion/Dilation (Minor changes for consistent debug output) ---
def detect_ellipses_fast_tuned(frame_bgr, debug=False) -> Tuple[List, Dict[str, np.ndarray]]:
    """
    Detection using HSV color filter with morphological operations.
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    
    # --- 1. HSV Masking ---
    # OpenCV H scale is 0-179 (for 0-360 degrees)
    lower_bound_1 = np.array([HUE_LOW_1, SAT_LOW, VAL_LOW])
    upper_bound_1 = np.array([HUE_HIGH_1, SAT_HIGH, VAL_HIGH])
    mask_1 = cv2.inRange(hsv, lower_bound_1, upper_bound_1)
    
    lower_bound_2 = np.array([HUE_LOW_2, SAT_LOW, VAL_LOW])
    upper_bound_2 = np.array([HUE_HIGH_2, SAT_HIGH, VAL_HIGH])
    mask_2 = cv2.inRange(hsv, lower_bound_2, upper_bound_2)
    
    mask = cv2.bitwise_or(mask_1, mask_2)
    
    # --- 2. Morphological Operations (Erode/Dilate for cleaning) ---
    
    # Erode to remove small specks and thin connections (like propeller blades intersecting)
    k_erode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    mask_eroded = cv2.erode(mask, k_erode, iterations=1)

    # Dilate to restore size and fill small gaps/holes (after propeller removal)
    k_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask_dilated = cv2.dilate(mask_eroded, k_dilate, iterations=1)
    
    # Close to further smooth contours
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask_closed = cv2.morphologyEx(mask_dilated, cv2.MORPH_CLOSE, k, iterations=2)
    
    # Create rim band (gradient) for better edge detection
    k_grad = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    band = cv2.morphologyEx(mask_closed, cv2.MORPH_GRADIENT, k_grad)
    
    # --- 3. Find and Filter Contours ---
    contours, _ = cv2.findContours(band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # Visualize sampled points
    sampled_points_vis = frame_bgr.copy()
    overlay_candidates = frame_bgr.copy()
    
    ellipses = []
    rejection_reasons = {
        'too_small': 0,
        'too_few_points': 0,
        'fit_failed': 0,
        'bad_axes': 0,
        'high_error': 0,
        'accepted': 0
    }
    
    # Base drawing for accepted candidates on the overlay_candidates frame (will be overwritten by FINAL)
    # This loop is kept clean to generate the base candidate visualization
    for idx, c in enumerate(contours):
        area = cv2.contourArea(c)
        
        # Filtering
        if len(c) < MIN_CONTOUR_POINTS:
            rejection_reasons['too_few_points'] += 1
            continue
            
        if area < MIN_CONTOUR_AREA:
            rejection_reasons['too_small'] += 1
            continue
        
        points = c.squeeze()
        if points.ndim != 2:
            rejection_reasons['fit_failed'] += 1
            continue
        
        # Sample points for visualization
        num_samples = min(20, len(points))
        sample_indices = np.linspace(0, len(points)-1, num_samples, dtype=int)
        sampled = points[sample_indices]
        
        # Draw sampled points for debug output 06
        for pt in sampled:
            cv2.circle(sampled_points_vis, tuple(pt), 3, (0, 255, 0), -1)
        
        # Try to fit ellipse
        ellipse, error, method = fit_ellipse_with_validation(points)
        
        if ellipse is None:
            rejection_reasons['fit_failed'] += 1
            cv2.drawContours(overlay_candidates, [c], -1, (0, 0, 255), 1)
            continue
        
        (cx, cy), (MA, ma), ang = ellipse
        
        # Check axes
        if MA < MIN_AXIS_LENGTH or ma < MIN_AXIS_LENGTH:
            rejection_reasons['bad_axes'] += 1
            cv2.ellipse(overlay_candidates, ellipse, (0, 128, 255), 1, cv2.LINE_AA)
            continue
            
        if MA > MAX_AXIS_LENGTH or ma > MAX_AXIS_LENGTH:
            rejection_reasons['bad_axes'] += 1
            continue
            
        # Check error
        if error > ELLIPSE_FIT_ERROR_THRESHOLD:
            rejection_reasons['high_error'] += 1
            cv2.ellipse(overlay_candidates, ellipse, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(overlay_candidates, f"e={error:.1f}", (int(cx-20), int(cy)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
            continue
        
        # Accepted - Draw the pre-selected candidate on the debug output 04 in Green
        rejection_reasons['accepted'] += 1
        ellipses.append(ellipse)
        
        cv2.ellipse(overlay_candidates, ellipse, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(overlay_candidates, f"{method}:{error:.1f}", (int(cx-20), int(cy)),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        for pt in sampled:
            cv2.circle(sampled_points_vis, tuple(pt), 4, (0, 0, 255), -1)
    
    # Add rejection stats to overlay
    y_pos = 60
    for reason, count in rejection_reasons.items():
        text = f"{reason}: {count}"
        cv2.putText(overlay_candidates, text, (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_pos += 20
    
    intermediate_frames = {
        '02_Binary_Mask': cv2.cvtColor(mask_closed, cv2.COLOR_GRAY2BGR),
        '04_Ellipse_Candidates': overlay_candidates,
        '06_Sampled_Points': sampled_points_vis
    }
    
    return ellipses, intermediate_frames

# --- PnP with Multiple Sizes (Unchanged) ---
def solve_ellipse_pnp_approx(K, ellipse, ring_radius_m) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Simple PnP for single ring size."""
    K = np.array([
    [539.9363327,   0.0,          640.0],
    [0.0,           539.9363708,  480.0],
    [0.0,           0.0,            1.0]
    ])

    fx, fy = K[0, 0], K[1, 1]
    cx0, cy0 = K[0, 2], K[1, 2]
    (cx, cy), (MA, ma), ang = ellipse
    diameter_world = 2.0 * ring_radius_m
    
    if MA < 1e-6:
        return None
    
    Z_c = fx * diameter_world / MA
    if Z_c < 0.1 or Z_c > 50.0:  # Reasonable range
        return None
    
    X_c = (Z_c / fx) * (cx - cx0)
    Y_c = (Z_c / fy) * (cy - cy0)
    center_3d = np.array([X_c, Y_c, Z_c], dtype=np.float64)
    
    # Normal approximation - This is where the normal is calculated
    ratio = min(1.0, ma / MA)
    alpha = math.acos(ratio)
    ang_rad = math.radians(ang + 90)
    dir_x = math.cos(ang_rad)
    dir_y = math.sin(ang_rad)
    normal_dir_3d = np.array([dir_x, dir_y, 0.0]) * math.sin(alpha)
    normal_dir_3d[2] = math.cos(alpha)
    normal_3d = normal_dir_3d / np.linalg.norm(normal_dir_3d)
    
    return center_3d, normal_3d

# --- Main Detection Function (Modified to draw final pose on debug frame) ---
def find_nearest_hoop_pose(bgr_image, K, ring_radius_m):
    """
    Main detection function that finds the nearest valid hoop pose.
    """
    ellipses, intermediate_frames = detect_ellipses_fast_tuned(bgr_image, debug=True)
    if not ellipses:
        return None, intermediate_frames
    
    hoop_candidates = []
    
    if isinstance(ring_radius_m, list):
        radii_to_try = ring_radius_m
    else:
        radii_to_try = [ring_radius_m]

    for ellipse in ellipses:
        for radius in radii_to_try:
            pose_result = solve_ellipse_pnp_approx(K, ellipse, radius)
            if pose_result is not None:
                center_3d, normal_3d = pose_result
                hoop_candidates.append({
                    'center_3d': center_3d,
                    'normal_3d': normal_3d,
                    'depth': center_3d[2],
                    'ellipse': ellipse,
                    'used_radius': radius
                })
    

    
    if not hoop_candidates:
        return None, intermediate_frames
    
    # Choose nearest
    nearest_hoop = min(hoop_candidates, key=lambda p: p['depth'])
    
    # --- NEW: Draw the final selected hoop pose (including normal) onto the ellipse candidates frame ---
    draw_frame = intermediate_frames['04_Ellipse_Candidates']
    
    # Draw the best candidate in a distinct color (e.g., Blue: 255, 0, 0 in BGR)
    _draw_hoop_pose_on_frame(
        draw_frame,
        K,
        nearest_hoop['center_3d'],
        nearest_hoop['normal_3d'],
        nearest_hoop['ellipse'],
        color=(255, 0, 0), # Blue
        text_info=[f"FINAL D:{nearest_hoop['depth']:.1f}", 
                   f"N Z:{nearest_hoop['normal_3d'][2]:.2f}"]
    )
    
    return (nearest_hoop['center_3d'], nearest_hoop['normal_3d'], 
            nearest_hoop['ellipse'], nearest_hoop['used_radius'], intermediate_frames)

# --- Visualization (Updated to use helper) ---
def draw_hoop_pose_overlay(frame, K, center_3d, normal_3d, ellipse, used_radius=None):
    """Draw detection overlay on the final frame."""
    text_info = [
        f"D:{center_3d[2]:.2f}m",
        f"N:Z:{normal_3d[2]:.2f}"
    ]
    if used_radius is not None:
        text_info.append(f"R:{used_radius:.3f}m")
        
    # Draw the pose with Green color (0, 255, 0)
    _draw_hoop_pose_on_frame(frame, K, center_3d, normal_3d, ellipse, (0, 255, 0), text_info)