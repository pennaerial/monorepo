# apriltag_detection.py
import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
import os

def detect_apriltags(
    image: np.ndarray,
    camera_matrix: np.ndarray,
    tag_size: float = 0.165,  # Tag size in meters (default: 165mm)
    tag_family: str = "tag36h11",
    uuid: str = "",
    debug: bool = False,
    save_vision: bool = False,
) -> Optional[List[Dict]]:
    """
    Detect AprilTags in an image and estimate their 6DOF pose.

    Args:
        image (np.ndarray): Input BGR image.
        camera_matrix (np.ndarray): 3x3 camera intrinsic matrix.
        tag_size (float): Physical size of the AprilTag in meters.
        tag_family (str): AprilTag family (e.g., "tag36h11", "tag25h9", "tag16h5").
        uuid (str): Unique identifier for saving images.
        debug (bool): If True, display visualization.
        save_vision (bool): If True, save visualization images.

    Returns:
        Optional[List[Dict]]: List of detected tags with their properties, or None if no tags detected.
        Each dict contains:
            - 'id': Tag ID
            - 'center': (x, y) pixel coordinates of tag center
            - 'corners': 4x2 array of corner pixel coordinates
            - 'distance': Estimated distance to tag in meters
            - 'rotation': (roll, pitch, yaw) in radians
            - 'tvec': Translation vector [x, y, z]
            - 'rvec': Rotation vector
    """
    try:
        from dt_apriltags import Detector
    except ImportError:
        raise ImportError(
            "dt_apriltags not installed. Install with: pip install dt-apriltags"
        )

    # Convert to grayscale for AprilTag detection
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Initialize detector
    detector = Detector(
        families=tag_family,
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )

    # Extract camera parameters
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    camera_params = [fx, fy, cx, cy]

    # Detect tags
    detections = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=tag_size
    )

    if not detections:
        if debug or save_vision:
            vis_image = image.copy()
            cv2.putText(vis_image, "No AprilTags detected", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        if debug:
            cv2.imshow("AprilTag Detection", vis_image)
            cv2.waitKey(1)
        if save_vision:
            import time
            timestamp = int(time.time())
            path = os.path.expanduser(f"~/vision_imgs/{uuid}")
            os.makedirs(path, exist_ok=True)
            cv2.imwrite(os.path.join(path, f"apriltag_{timestamp}.png"), vis_image)
        return None

    # Process detections
    results = []
    vis_image = image.copy() if (debug or save_vision) else None

    for detection in detections:
        # Get tag center
        center = detection.center.astype(int)

        # Get corners
        corners = detection.corners.astype(int)

        # Get pose estimation
        tvec = detection.pose_t.flatten()  # Translation vector [x, y, z]
        rvec = detection.pose_R  # Rotation matrix

        # Calculate distance (using z component of translation vector)
        distance = tvec[2]

        # Convert rotation matrix to Euler angles (roll, pitch, yaw)
        # Using cv2.Rodrigues to get rotation vector first
        rvec_rodrigues, _ = cv2.Rodrigues(rvec)

        # Convert to Euler angles
        # This follows the convention: yaw (z), pitch (y), roll (x)
        sy = np.sqrt(rvec[0, 0] * rvec[0, 0] + rvec[1, 0] * rvec[1, 0])
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(rvec[2, 1], rvec[2, 2])
            pitch = np.arctan2(-rvec[2, 0], sy)
            yaw = np.arctan2(rvec[1, 0], rvec[0, 0])
        else:
            roll = np.arctan2(-rvec[1, 2], rvec[1, 1])
            pitch = np.arctan2(-rvec[2, 0], sy)
            yaw = 0

        # Store results
        tag_data = {
            'id': detection.tag_id,
            'center': tuple(center),
            'corners': corners,
            'distance': distance,
            'rotation': (roll, pitch, yaw),
            'tvec': tvec,
            'rvec': rvec_rodrigues.flatten()
        }
        results.append(tag_data)

        # Visualization
        if vis_image is not None:
            # Draw tag boundary
            cv2.polylines(vis_image, [corners], True, (0, 255, 0), 2)

            # Draw tag center
            cv2.circle(vis_image, tuple(center), 5, (0, 0, 255), -1)

            # Draw tag ID and distance
            label = f"ID: {detection.tag_id}"
            dist_label = f"Dist: {distance:.2f}m"
            cv2.putText(vis_image, label, (center[0] - 20, center[1] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.putText(vis_image, dist_label, (center[0] - 20, center[1] - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

            # Draw coordinate axes
            axis_length = tag_size * 0.5
            _draw_axis(vis_image, camera_matrix, rvec, tvec, axis_length)

    # Display or save visualization
    if debug and vis_image is not None:
        cv2.putText(vis_image, f"Detected {len(results)} tag(s)", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("AprilTag Detection", vis_image)
        cv2.waitKey(1)

    if save_vision and vis_image is not None:
        import time
        timestamp = int(time.time())
        path = os.path.expanduser(f"~/vision_imgs/{uuid}")
        os.makedirs(path, exist_ok=True)
        cv2.imwrite(os.path.join(path, f"apriltag_{timestamp}.png"), vis_image)

    return results


def _draw_axis(image, camera_matrix, rvec, tvec, length):
    """
    Draw 3D coordinate axes on the image for visualization.

    Args:
        image: Image to draw on
        camera_matrix: Camera intrinsic matrix
        rvec: Rotation vector
        tvec: Translation vector
        length: Length of axes in meters
    """
    # Define 3D points for axes
    axis_points = np.float32([
        [0, 0, 0],           # Origin
        [length, 0, 0],      # X-axis (red)
        [0, length, 0],      # Y-axis (green)
        [0, 0, -length]      # Z-axis (blue) - negative because camera looks down +Z
    ]).reshape(-1, 3)

    # Project 3D points to 2D
    imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, None)
    imgpts = imgpts.astype(int)

    origin = tuple(imgpts[0].ravel())

    # Draw axes
    try:
        image = cv2.line(image, origin, tuple(imgpts[1].ravel()), (0, 0, 255), 2)  # X - Red
        image = cv2.line(image, origin, tuple(imgpts[2].ravel()), (0, 255, 0), 2)  # Y - Green
        image = cv2.line(image, origin, tuple(imgpts[3].ravel()), (255, 0, 0), 2)  # Z - Blue
    except:
        pass  # Ignore if points are out of image bounds

    return image


def rotate_image(image: np.ndarray, angle_deg: float) -> np.ndarray:
    """
    Rotate image by given angle in degrees.

    Args:
        image: Input image
        angle_deg: Rotation angle in degrees

    Returns:
        Rotated image
    """
    h, w = image.shape[:2]
    center = (w // 2, h // 2)
    rotation_matrix = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    rotated = cv2.warpAffine(image, rotation_matrix, (w, h))
    return rotated
