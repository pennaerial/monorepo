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
