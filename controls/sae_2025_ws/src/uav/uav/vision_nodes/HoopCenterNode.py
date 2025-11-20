#!/usr/bin/env python3
# hoop_center_node.py
import cv2
import numpy as np
import rclpy
from uav.vision_nodes import VisionNode

# ---------------- Tunables ----------------
HSV_RANGES = {
    "red1": ((0, 100, 100), (10, 255, 255)),
    "red2": ((160, 100, 100), (179, 255, 255)),
}

MORPH_KERNEL = (5, 5)
MORPH_ITERS = 2

MIN_AXIS_PIX = 40
LINE_RATIO_THRESHOLD = 0.18
AREA_COVERAGE_MIN = 0.6

FULL_RIGHT_VECTOR = np.array([1.0, 0.0, 0.0], dtype=np.float32)


def rotate_image(frame, yaw_deg: float):
    if abs(yaw_deg) < 1e-6:
        return frame
    h, w = frame.shape[:2]
    M = cv2.getRotationMatrix2D((w / 2, h / 2), yaw_deg, 1.0)
    return cv2.warpAffine(frame, M, (w, h), flags=cv2.INTER_LINEAR)


def pixel_to_ray(u: float, v: float, K: np.ndarray) -> np.ndarray:
    invK = np.linalg.inv(K)
    uv1 = np.array([u, v, 1.0], dtype=np.float32)
    ray = invK @ uv1
    ray = ray / (np.linalg.norm(ray) + 1e-9)
    return ray.astype(np.float32)


def ellipse_area(a: float, b: float) -> float:
    return np.pi * (0.5 * a) * (0.5 * b)


class HoopCenterNode(VisionNode):
    """
    ROS node for hoop tracking - detects red hoops and returns center vector.
    """
    srv = HoopCenterNode
    
    def __init__(self):
        super().__init__('hoop_center_node', self.__class__.srv)
        self.create_service(HoopCenterNode, self.service_name(), self.service_callback)

    def mask_red(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Combine two HSV masks for red."""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lower1, upper1 = HSV_RANGES["red1"]
        lower2, upper2 = HSV_RANGES["red2"]
        mask1 = cv2.inRange(hsv, np.array(lower1, np.uint8), np.array(upper1, np.uint8))
        mask2 = cv2.inRange(hsv, np.array(lower2, np.uint8), np.array(upper2, np.uint8))
        mask = cv2.bitwise_or(mask1, mask2)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, MORPH_KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=MORPH_ITERS)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=MORPH_ITERS)
        return mask

    def find_ellipse(self, frame_bgr: np.ndarray):
        mask = self.mask_red(frame_bgr)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            return False, (0.0, 0.0), (0.0, 0.0), False, False

        cnt = max(contours, key=cv2.contourArea)
        if len(cnt) < 5:
            return False, (0.0, 0.0), (0.0, 0.0), False, True

        (cx, cy), (a, b), _angle = cv2.fitEllipse(cnt)

        ratio = min(a, b) / (max(a, b) + 1e-9)
        is_line = ratio < LINE_RATIO_THRESHOLD

        cnt_area = cv2.contourArea(cnt)
        ell_area = ellipse_area(a, b)
        coverage = (cnt_area / (ell_area + 1e-9)) if ell_area > 1e-6 else 0.0
        is_partial = coverage < AREA_COVERAGE_MIN

        return True, (float(cx), float(cy)), (float(a), float(b)), is_line, is_partial

    def service_callback(self, request: HoopCenterNode.Request, 
                        response: HoopCenterNode.Response):
        """Process hoop tracking service request"""
        # Get image and camera info from VisionNode
        image, camera_info = self.request_data(cam_image=True, cam_info=True)
        frame_bgr = self.convert_image_msg_to_frame(image)
        
        # Rotate image based on yaw
        frame_bgr = rotate_image(frame_bgr, -np.rad2deg(request.yaw))
        
        # Get camera matrix
        K = np.array(camera_info.k).reshape(3, 3)
        
        # Find hoop ellipse
        found, (cx, cy), (a, b), is_line, is_partial = self.find_ellipse(frame_bgr)
        
        h, w = frame_bgr.shape[:2]
        used_x, used_y = (w / 2.0, h / 2.0)
        dlz_empty = True

        if found:
            if is_line:
                dir_vec = FULL_RIGHT_VECTOR.copy()
                used_x, used_y = cx, cy
                dlz_empty = False
            else:
                if max(a, b) >= MIN_AXIS_PIX and not is_partial:
                    used_x, used_y = cx, cy
                    dlz_empty = False
                dir_vec = pixel_to_ray(used_x, used_y, K)
        else:
            dir_vec = pixel_to_ray(used_x, used_y, K)

        # Populate response
        response.x = float(used_x)
        response.y = float(used_y)
        response.direction = dir_vec
        response.dlz_empty = dlz_empty
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