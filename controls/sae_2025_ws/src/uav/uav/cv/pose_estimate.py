import cv2
import numpy as np

def estimate_hoop_pose_from_contour(
    contour: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    hoop_radius: float = 0.63825,
    num_samples: int = 8,
    reprojection_error: float = 3.0,
    max_iterations: int = 100,
    confidence: float = 0.99,
    min_contour_area: float = 100.0
):
    """
    
    Args:
        contour: Contour points of the hoop in image space (numpy array of shape (N, 1, 2))
        camera_matrix: Camera's intrinsic matrix (3x3 numpy array)
        dist_coeffs: Camera's distortion coefficients (numpy array)
        hoop_radius: Radius of the hoop in meters (default: 0.63825)
        num_samples: Number of points to sample from fitted ellipse (default: 8)
        reprojection_error: Maximum reprojection error threshold for RANSAC in pixels (default: 3.0)
        max_iterations: Maximum iterations for RANSAC (default: 100)
        confidence: Confidence level for RANSAC 0-1 (default: 0.99)
        min_contour_area: Minimum contour area required (default: 100.0)
    
    Returns:
        success: Boolean indicating if pose estimation was successful
        rvec: Rotation vector from camera to hoop (3x1 numpy array) or None
        tvec: Translation vector from camera to hoop (3x1 numpy array) or None
        c_obj: Camera position in hoop coordinates (3x1 numpy array) or None
        ellipse: Fitted ellipse parameters ((center), (major, minor), angle) or None
    """
    # Validate contour
    if contour is None or len(contour) < 5:
        return False, None, None, None, None
    
    area = cv2.contourArea(contour)
    if area < min_contour_area:
        return False, None, None, None, None
    
    # Preprocess contour to reduce noise using Douglas-Peucker approximation
    if len(contour) >= 5:
        perimeter = cv2.arcLength(contour, True)
        epsilon = 0.01 * perimeter
        preprocessed_contour = cv2.approxPolyDP(contour, epsilon, True)
        if len(preprocessed_contour) >= 5:
            contour = preprocessed_contour
    
    # Fit ellipse to contour points (handles noise and outliers)
    try:
        ellipse = cv2.fitEllipse(contour)
        (cx, cy), (major, minor), angle = ellipse
    except cv2.error:
        # If ellipse fitting fails, contour may be too incomplete or irregular
        return False, None, None, None, None
    
    # Sample points along the fitted ellipse in image space
    angle_radians = np.deg2rad(angle)
    r_ellipse = np.array([[np.cos(angle_radians), -np.sin(angle_radians)],
                          [np.sin(angle_radians),  np.cos(angle_radians)]])
    a, b = major / 2.0, minor / 2.0
    
    # Get evenly spaced sample points on ellipse
    thetas = np.linspace(0, 2*np.pi, num_samples, endpoint=False)
    ellipse_points = np.stack([a * np.cos(thetas), b * np.sin(thetas)], axis=1)
    image_points = (ellipse_points @ r_ellipse.T) + np.array([cx, cy])
    image_points = image_points.astype(np.float32)
    
    # Get corresponding 3D points on hoop circle
    object_points = np.stack([
        hoop_radius * np.cos(thetas),
        hoop_radius * np.sin(thetas),
        np.zeros_like(thetas)
    ], axis=1).astype(np.float32)
    
    # Solve PnP RANSAC
    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        objectPoints=object_points,
        imagePoints=image_points,
        cameraMatrix=camera_matrix,
        distCoeffs=dist_coeffs,
        flags=cv2.SOLVEPNP_EPNP,  # Efficient PnP solver
        reprojectionError=reprojection_error,
        iterationsCount=max_iterations,
        confidence=confidence
    )
    
    if not success or rvec is None or tvec is None:
        return False, None, None, None, None
    
    # Verify we have enough inliers (at least 50% of points)
    if inliers is not None and len(inliers) < num_samples * 0.5:
        return False, None, None, None, None
    
    # Calculate camera position in hoop frame
    R, _ = cv2.Rodrigues(rvec)
    c_obj = -R.T @ tvec
    
    return True, rvec, tvec, c_obj, ellipse


def estimate_rectangle_pose_from_contour(
    contour: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rectangle_width: float = 0.5,  # Width of landing pad in meters (adjust as needed)
    rectangle_height: float = 0.5,  # Height of landing pad in meters (adjust as needed)
    reprojection_error: float = 3.0,
    max_iterations: int = 100,
    confidence: float = 0.99,
    min_contour_area: float = 100.0
):
    """
    Estimate pose of a rectangular landing pad from its contour.
    
    Args:
        contour: Contour points of the rectangle in image space (numpy array, should have 4 corners)
        camera_matrix: Camera's intrinsic matrix (3x3 numpy array)
        dist_coeffs: Camera's distortion coefficients (numpy array)
        rectangle_width: Width of the landing pad in meters (default: 0.5)
        rectangle_height: Height of the landing pad in meters (default: 0.5)
        reprojection_error: Maximum reprojection error threshold for RANSAC in pixels (default: 3.0)
        max_iterations: Maximum iterations for RANSAC (default: 100)
        confidence: Confidence level for RANSAC 0-1 (default: 0.99)
        min_contour_area: Minimum contour area required (default: 100.0)
    
    Returns:
        success: Boolean indicating if pose estimation was successful
        rvec: Rotation vector from camera to rectangle (3x1 numpy array) or None
        tvec: Translation vector from camera to rectangle center (3x1 numpy array) or None
        c_obj: Camera position in rectangle coordinates (3x1 numpy array) or None
        rectangle: Rectangle parameters (center, size, angle) or None
    """
    # Validate contour
    if contour is None or len(contour) < 4:
        return False, None, None, None, None
    
    area = cv2.contourArea(contour)
    if area < min_contour_area:
        return False, None, None, None, None
    
    # Ensure we have exactly 4 corners
    if len(contour) != 4:
        # Approximate contour to get 4 corners
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
        if len(approx) != 4:
            # If still not 4, try fitting a minimum area rectangle
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            approx = np.int0(box).reshape(-1, 1, 2)
        
        if len(approx) != 4:
            return False, None, None, None, None
        contour = approx
    
    # Reshape contour to (4, 2) format
    if contour.shape[1] == 1:
        image_points = contour.reshape(-1, 2).astype(np.float32)
    else:
        image_points = contour.astype(np.float32)
    
    # Define 3D object points for a rectangle centered at origin
    # Rectangle in object frame: centered at origin, lying in XY plane
    half_w = rectangle_width / 2.0
    half_h = rectangle_height / 2.0
    
    object_points = np.array([
        [-half_w, -half_h, 0.0],  # Bottom-left
        [half_w, -half_h, 0.0],   # Bottom-right
        [half_w, half_h, 0.0],    # Top-right
        [-half_w, half_h, 0.0]    # Top-left
    ], dtype=np.float32)
    
    # Order the image points to match object points
    # Use the center and sort points by angle to get consistent ordering
    center = image_points.mean(axis=0)
    angles = np.arctan2(image_points[:, 1] - center[1], image_points[:, 0] - center[0])
    sorted_indices = np.argsort(angles)
    
    # Reorder image points (starting from the point with smallest angle)
    ordered_image_points = image_points[sorted_indices].astype(np.float32)
    
    # Solve PnP
    success, rvec, tvec = cv2.solvePnP(
        objectPoints=object_points,
        imagePoints=ordered_image_points,
        cameraMatrix=camera_matrix,
        distCoeffs=dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    
    if not success or rvec is None or tvec is None:
        return False, None, None, None, None
    
    # Calculate camera position in rectangle frame
    R, _ = cv2.Rodrigues(rvec)
    c_obj = -R.T @ tvec
    
    # Get rectangle parameters for return
    rect = cv2.minAreaRect(contour)
    
    return True, rvec, tvec, c_obj, rect
