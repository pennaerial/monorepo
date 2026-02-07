# helipad_tracking_node.py
import cv2
import numpy as np
from uav.vision_nodes import VisionNode
from uav_interfaces.srv import HelipadTracking
from uav.cv.tracking import compute_3d_vector, rotate_image
import rclpy


def find_helipad(frame: np.ndarray, debug: bool = False) -> tuple:
    """
    Detect an H-shaped helipad in the frame.

    Uses color thresholding (white H on dark background) and contour analysis.

    Args:
        frame: BGR image from camera
        debug: Whether to show debug visualization

    Returns:
        (center_x, center_y, detected) or None if not found
    """
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Threshold to find white regions (helipad H is typically white)
    # Adjust these values based on your helipad appearance
    _, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

    # Also try adaptive threshold for varying lighting
    adaptive_thresh = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 11, 2
    )

    # Combine both thresholds
    combined = cv2.bitwise_or(thresh, adaptive_thresh)

    # Morphological operations to clean up
    kernel = np.ones((3, 3), np.uint8)
    combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
    combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)

    # Find contours
    contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    # Filter contours by area and aspect ratio
    frame_area = frame.shape[0] * frame.shape[1]
    min_area = frame_area * 0.001  # At least 0.1% of frame
    max_area = frame_area * 0.5    # At most 50% of frame

    best_candidate = None
    best_score = 0

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < min_area or area > max_area:
            continue

        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h if h > 0 else 0

        # H shape should be roughly square (aspect ratio near 1)
        if not (0.5 < aspect_ratio < 2.0):
            continue

        # Check rectangularity (how much the contour fills its bounding box)
        rect_area = w * h
        extent = float(area) / rect_area if rect_area > 0 else 0

        # H shape typically has extent around 0.3-0.6
        if not (0.2 < extent < 0.8):
            continue

        # Score based on how centered and large the detection is
        center_x = x + w // 2
        center_y = y + h // 2
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2

        # Prefer detections closer to center
        distance_to_center = np.sqrt(
            (center_x - frame_center_x) ** 2 +
            (center_y - frame_center_y) ** 2
        )
        max_distance = np.sqrt(frame_center_x ** 2 + frame_center_y ** 2)
        center_score = 1 - (distance_to_center / max_distance)

        # Prefer larger detections
        size_score = area / max_area

        score = center_score * 0.5 + size_score * 0.5

        if score > best_score:
            best_score = score
            best_candidate = (center_x, center_y, contour)

    if best_candidate is None:
        return None

    center_x, center_y, contour = best_candidate

    if debug:
        debug_frame = frame.copy()
        cv2.drawContours(debug_frame, [contour], -1, (0, 255, 0), 2)
        cv2.circle(debug_frame, (center_x, center_y), 10, (0, 0, 255), -1)
        cv2.imshow('Helipad Detection', debug_frame)
        cv2.waitKey(1)

    return (center_x, center_y, True)


class HelipadTrackingNode(VisionNode):
    """
    ROS node for helipad (H marker) tracking.
    """
    srv = HelipadTracking

    def __init__(self):
        super().__init__(HelipadTracking)
        self.create_service(HelipadTracking, self.service_name(), self.service_callback)
        self.get_logger().info("HelipadTrackingNode initialized")

    def service_callback(self, request: HelipadTracking.Request,
                        response: HelipadTracking.Response):
        """Process tracking service request"""
        image, camera_info = self.request_data(cam_image=True, cam_info=True)

        if image is None:
            response.detected = False
            response.x = 0.0
            response.y = 0.0
            response.direction = [0.0, 0.0, 0.0]
            return response

        frame = self.convert_image_msg_to_frame(image)
        frame = rotate_image(frame, -np.rad2deg(request.yaw))

        detection = find_helipad(frame, debug=self.debug)

        if detection is not None:
            cx, cy, detected = detection
            response.detected = True
            response.x = float(cx)
            response.y = float(cy)

            # Compute 3D direction vector to the helipad
            direction = compute_3d_vector(
                cx, cy,
                np.array(camera_info.k).reshape(3, 3),
                request.altitude
            )
            response.direction = direction
        else:
            # No detection - return center of frame
            response.detected = False
            response.x = float(frame.shape[1] // 2)
            response.y = float(frame.shape[0] // 2)
            response.direction = [0.0, 0.0, 0.0]

        return response


def main():
    rclpy.init()
    node = HelipadTrackingNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()
    rclpy.shutdown()
