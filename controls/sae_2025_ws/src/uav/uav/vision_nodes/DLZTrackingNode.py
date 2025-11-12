#!/usr/bin/env python3
import cv2
import numpy as np
from uav.vision_nodes import VisionNode
from uav_interfaces.srv import DLZTracking        # make sure you have this .srv defined
from uav.cv.tracking import compute_3d_vector, rotate_image
import rclpy


# ---------- CV UTILITIES (from landingzone_outline.py) ----------
def mask_magenta(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, (135, 70, 70), (170, 255, 255))
    m2 = cv2.inRange(hsv, (170, 70, 70), (179, 255, 255))
    mask = cv2.bitwise_or(m1, m2)
    k = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, 2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, 1)
    return mask

def mask_white(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, (0, 0, 200), (179, 60, 255))
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), 1)
    return white

def detect_lz_outline(bgr, min_area_px=800, simplify_eps_frac=0.004):
    """Return (cx, cy) of largest magenta/white outline contour or None."""
    pink = mask_magenta(bgr)
    white = mask_white(bgr)

    touch = cv2.dilate(pink, np.ones((9,9), np.uint8), 1)
    rim = cv2.bitwise_and(white, white, mask=touch)
    rim = cv2.bitwise_and(rim, cv2.bitwise_not(pink))

    contours, _ = cv2.findContours(rim, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        contours, _ = cv2.findContours(pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < min_area_px:
            return None
    else:
        c = max(contours, key=cv2.contourArea)

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
    return cx, cy


# ---------- ROS2 NODE ----------
class LandingZoneTrackingNode(VisionNode):
    """
    ROS2 node for landing-zone (DLZ) tracking with Kalman filtering.
    """
    srv = DLZTracking

    def __init__(self):
        super().__init__('landing_zone_tracking', self.__class__.srv)

        # Simple 2D-pos / velocity Kalman filter
        self.kalman = cv2.KalmanFilter(4, 2)
        self._setup_kalman_filter()

        self.create_service(DLZTracking, self.service_name(), self.service_callback)

    def _setup_kalman_filter(self):
        dt = 1.0
        self.kalman.transitionMatrix = np.array([
            [1,0,dt,0],
            [0,1,0,dt],
            [0,0,1,0],
            [0,0,0,1]
        ], np.float32)
        self.kalman.measurementMatrix = np.array([
            [1,0,0,0],
            [0,1,0,0]
        ], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32)

    # --------- Main service callback ----------
    def service_callback(self, request: DLZTracking.Request, response: DLZTracking.Response):
        # 1. Get camera frame
        img_msg, cam_info = self.request_data(cam_image=True, cam_info=True)
        frame = self.convert_image_msg_to_frame(img_msg)
        frame = rotate_image(frame, -np.rad2deg(request.yaw))

        # 2. Predict next state (optional)
        # pred = self.kalman.predict()

        # 3. Detect landing zone center
        detection = detect_lz_outline(frame)

        if detection:
            cx, cy = detection
            meas = np.array([[np.float32(cx)], [np.float32(cy)]])
            self.kalman.correct(meas)
            lz_visible = True
        else:
            # fallback: center of frame
            h, w = frame.shape[:2]
            cx, cy = w/2, h/2
            lz_visible = False

        # 4. Compute 3-D direction vector
        direction = compute_3d_vector(cx, cy, np.array(cam_info.k).reshape(3,3), request.altitude)

        # 5. Fill response
        response.x = float(cx)
        response.y = float(cy)
        response.direction = direction
        response.lz_visible = lz_visible
        return response


def main():
    rclpy.init()
    node = LandingZoneTrackingNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print("LandingZoneTrackingNode exception:", e)
        node.publish_failsafe()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
