#!/usr/bin/env python3
# hoop_center_node.py

import cv2
import numpy as np
import rclpy

from uav.vision_nodes import VisionNode
from uav_interfaces.srv import HoopCenter

# ---------------- Tunables ----------------
HSV_RANGES = {
    "red1": ((0, 100, 100), (10, 255, 255)),
    "red2": ((160, 100, 100), (179, 255, 255)),
}

MORPH_KERNEL = (5, 5)
MORPH_ITERS = 2

MIN_AXIS_PIX = 40           # minimum major/minor axis to consider "big enough"
LINE_RATIO_THRESHOLD = 0.18 # if min/maj < this => treat as line
AREA_COVERAGE_MIN = 0.6     # contour area / fitted ellipse area

# If the hoop looks like a line, we can return a "full right" vector as a hint
FULL_RIGHT_VECTOR = np.array([1.0, 0.0, 0.0], dtype=np.float32)


def rotate_image(frame, yaw_deg: float):
    """Rotate image around its center by yaw_deg degrees."""
    if abs(yaw_deg) < 1e-6:
        return frame
    h, w = frame.shape[:2]
    M = cv2.getRotationMatrix2D((w / 2, h / 2), yaw_deg, 1.0)
    return cv2.warpAffine(frame, M, (w, h), flags=cv2.INTER_LINEAR)


def pixel_to_ray(u: float, v: float, K: np.ndarray) -> np.ndarray:
    """Convert pixel (u, v) to a normalized ray using intrinsics K."""
    invK = np.linalg.inv(K)
    uv1 = np.array([u, v, 1.0], dtype=np.float32)
    ray = invK @ uv1
    ray = ray / (np.linalg.norm(ray) + 1e-9)
    return ray.astype(np.float32)


def ellipse_area(a: float, b: float) -> float:
    """Area of ellipse with major/minor axes a, b."""
    return np.pi * (0.5 * a) * (0.5 * b)


class HoopCenterNode(VisionNode):
    """
    ROS node for hoop tracking - detects red hoops and returns center vector.
    """
    srv = HoopCenter

    def __init__(self):
        # Node name and associated service type
        super().__init__('hoop_center', self.__class__.srv)

        # Advertise the service using VisionNode.service_name()
        self.create_service(
            HoopCenter,
            self.service_name(),
            self.service_callback
        )

    # ---------------- Image processing helpers ----------------

    def mask_red(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Combine two HSV masks for red."""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lower1, upper1 = HSV_RANGES["red1"]
        lower2, upper2 = HSV_RANGES["red2"]

        mask1 = cv2.inRange(
            hsv,
            np.array(lower1, np.uint8),
            np.array(upper1, np.uint8)
        )
        mask2 = cv2.inRange(
            hsv,
            np.array(lower2, np.uint8),
            np.array(upper2, np.uint8)
        )

        mask = cv2.bitwise_or(mask1, mask2)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, MORPH_KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=MORPH_ITERS)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=MORPH_ITERS)
        return mask

    def find_ellipse(self, frame_bgr: np.ndarray):
        """
        Find the largest red contour and fit an ellipse.

        Returns:
            found (bool)
            (cx, cy)    - center
            (a, b)      - major/minor axes
            is_line (bool)
            is_partial (bool)
        """
        mask = self.mask_red(frame_bgr)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        if not contours:
            return False, (0.0, 0.0), (0.0, 0.0), False, False

        cnt = max(contours, key=cv2.contourArea)

        if len(cnt) < 5:
            # Not enough points to fit an ellipse
            return False, (0.0, 0.0), (0.0, 0.0), False, True

        (cx, cy), (a, b), _angle = cv2.fitEllipse(cnt)

        # Determine if the hoop looks like a line (very oblique)
        ratio = min(a, b) / (max(a, b) + 1e-9)
        is_line = ratio < LINE_RATIO_THRESHOLD

        # Coverage: how much of the fitted ellipse is actually present
        cnt_area = cv2.contourArea(cnt)
        ell_area = ellipse_area(a, b)
        coverage = (cnt_area / (ell_area + 1e-9)) if ell_area > 1e-6 else 0.0
        is_partial = coverage < AREA_COVERAGE_MIN

        return True, (float(cx), float(cy)), (float(a), float(b)), is_line, is_partial

    # ---------------- Service callback ----------------

    def service_callback(self, request: HoopCenter.Request,
                         response: HoopCenter.Response):
        """
        Process hoop tracking service request.

        Input:
            request.altitude
            request.yaw (radians)

        Output:
            response.x, response.y           (pixel center used)
            response.direction[3]           (3D ray in camera frame)
            response.dlz_empty              (whether we consider the zone "empty")
            response.is_line, is_partial    (shape diagnostics)
            response.major_axis, minor_axis
        """
        # Get image and camera info from VisionNode
        image_msg, camera_info = self.request_data(cam_image=True, cam_info=True)
        frame_bgr = self.convert_image_msg_to_frame(image_msg)

        # Rotate image based on yaw (similar to payload)
        frame_bgr = rotate_image(frame_bgr, -np.rad2deg(request.yaw))

        # Camera intrinsics
        K = np.array(camera_info.k, dtype=np.float32).reshape(3, 3)

        # Detect hoop ellipse
        found, (cx, cy), (a, b), is_line, is_partial = self.find_ellipse(frame_bgr)

        h, w = frame_bgr.shape[:2]
        used_x, used_y = (w / 2.0, h / 2.0)  # default to image center
        dlz_empty = True

        if found:
            if is_line:
                # Hoop looks like a line -> usually very oblique.
                # We still report where we saw it, but direction is "full right"
                used_x, used_y = cx, cy
                dir_vec = FULL_RIGHT_VECTOR.copy()
                dlz_empty = False
            else:
                # Full-ish ellipse. Only trust it if big enough and not too partial.
                if max(a, b) >= MIN_AXIS_PIX and not is_partial:
                    used_x, used_y = cx, cy
                    dlz_empty = False
                # Either way, direction is from used pixel
                dir_vec = pixel_to_ray(used_x, used_y, K)
        else:
            # No contour at all: ray from image center
            dir_vec = pixel_to_ray(used_x, used_y, K)

        # Populate response
        response.x = float(used_x)
        response.y = float(used_y)
        response.direction = [float(dir_vec[0]), float(dir_vec[1]), float(dir_vec[2])]
        response.dlz_empty = bool(dlz_empty)
        response.is_line = bool(is_line)
        response.is_partial = bool(is_partial)
        response.major_axis = float(a)
        response.minor_axis = float(b)

        return response


def main():
    rclpy.init()
    node = HoopCenterNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
