# # hoop_tracking_node.py
# import cv2
# import numpy as np
# from uav.cv.tracking import find_payload, compute_3d_vector, find_hoop
# from uav.vision_nodes import VisionNode
# from uav_interfaces.srv import HoopTracking
# import rclpy
# from uav.utils import pink, green, blue, yellow, red, orange

# class HoopTrackingNode(VisionNode):
#     """
#     ROS node for payload tracking with Kalman filtering.
#     """
#     srv = HoopTracking
#     def __init__(self):
#         super().__init__('hoop_tracking', self.__class__.srv)

#         self.color_map = {
#             'pink': pink, 
#             'green': green,
#             'blue': blue,
#             'yellow': yellow,
#             'red': red,
#             'orange': orange
#         }
        
#         # Initialize Kalman filter
#         self.kalman = cv2.KalmanFilter(4, 2)
#         self._setup_kalman_filter()

#         self.create_service(HoopTracking, self.service_name(), self.service_callback)
        
#     def _setup_kalman_filter(self):
#         """Initialize Kalman filter matrices"""
#         dt = 1  
        
#         # State transition matrix [x, y, vx, vy]
#         self.kalman.transitionMatrix = np.array([
#             [1, 0, dt, 0],
#             [0, 1, 0, dt],
#             [0, 0, 1, 0],
#             [0, 0, 0, 1]
#         ], dtype=np.float32)
        
#         # Measurement matrix [x, y]
#         self.kalman.measurementMatrix = np.array([
#             [1, 0, 0, 0],
#             [0, 1, 0, 0]
#         ], dtype=np.float32)
        
#         # Tune these covariances idk
#         self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
#         self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
#         self.kalman.errorCovPost = np.eye(4, dtype=np.float32)
        
#     def service_callback(self, request: HoopTracking.Request,
#                         response: HoopTracking.Response):
#         """Process tracking service request with Kalman filtering"""
#         image, camera_info = self.request_data(cam_image=True, cam_info=True)
#         image = self.convert_image_msg_to_frame(image)
#         # No image rotation needed for forward-facing camera (yaw just changes pointing direction)

#         # Predict next state
#         # prediction = self.kalman.predict()
#         # predicted_x, predicted_y = prediction[0, 0], prediction[1, 0]
#         # Get raw detection
#         detection = find_hoop(
#             image,
#             *red,
#             *orange,
#             self.uuid,
#             self.debug,
#             self.save_vision
#         )
#         dlz_empty = False
#         if detection is not None:
#             cx, cy, dlz_empty = detection
#             # Update Kalman filter with measurement
#             # measurement = np.array([[np.float32(cx)], [np.float32(cy)]])
#             # corrected_state = self.kalman.correct(measurement)
#             # x, y = corrected_state[0, 0], corrected_state[1, 0]
#             x, y = cx, cy
#         else:
#             # # Use prediction if no detection
#             # x, y = predicted_x, predicted_y
#             x, y = image.shape[:2]
#             x /= 2
#             y /= 2
            
        
#         # Compute 3D direction vector
#         direction = compute_3d_vector(x, y, np.array(camera_info.k, ).reshape(3, 3), request.altitude)
            
#         # Populate response
#         response.x = float(x)
#         response.y = float(y)
#         response.direction = direction
#         response.dlz_empty = dlz_empty
#         return response

# def main():
#     rclpy.init()
#     node = HoopTrackingNode()
#     try:    
#         rclpy.spin(node)
#     except Exception as e:
#         print(e)
#         node.publish_failsafe()

#     rclpy.shutdown()
# hoop_tracking_node.py
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from uav.cv.tracking import find_payload, compute_3d_vector, find_hoop
from uav.vision_nodes import VisionNode
from uav_interfaces.srv import HoopTracking
from uav.utils import pink, green, blue, yellow, red, orange

# Optional debug publisher dependencies
try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
except Exception:
    CvBridge = None
    Image = None


class HoopTrackingNode(VisionNode):
    """
    ROS2 node for hoop tracking with (optional) Kalman filter and a
    professional OpenCV HUD for real-time debugging.
    """
    srv = HoopTracking

    def __init__(self):
        super().__init__('hoop_tracking', self.__class__.srv)

        # Color map if you use it elsewhere
        self.color_map = {
            'pink': pink,
            'green': green,
            'blue': blue,
            'yellow': yellow,
            'red': red,
            'orange': orange,
        }

        # Optional Kalman filter (currently not used; left in place if you enable later)
        self.kalman = cv2.KalmanFilter(4, 2)
        self._setup_kalman_filter()

        # Debug window + publisher setup
        self.window_created = False
        self._bridge = CvBridge() if CvBridge is not None else None
        self.debug_pub = None
        if Image is not None and self._bridge is not None:
            # Publish the debug HUD image so GCS can view with rqt_image_view
            self.debug_pub = self.create_publisher(Image, '/debug/hoop_tracking', 10)

        # Parameters (could also be declared as ROS params)
        self.center_threshold_px = 50  # pixel threshold for "CENTERED" status

        # Service
        self.create_service(HoopTracking, self.service_name(), self.service_callback)

    # ---------------- Kalman (placeholder; keep if you plan to enable) ----------------
    def _setup_kalman_filter(self):
        """Initialize Kalman filter matrices."""
        dt = 1.0  # if you enable, use actual dt
        self.kalman.transitionMatrix = np.array(
            [[1, 0, dt, 0],
             [0, 1, 0, dt],
             [0, 0, 1,  0],
             [0, 0, 0,  1]], dtype=np.float32
        )
        self.kalman.measurementMatrix = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0]], dtype=np.float32
        )
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32)

    # --------------------------------- Main service -----------------------------------
    def service_callback(self, request: HoopTracking.Request,
                    response: HoopTracking.Response):
        """Process tracking service request with Kalman filtering"""
        image, camera_info = self.request_data(cam_image=True, cam_info=True)
        image = self.convert_image_msg_to_frame(image)
        
        # CRITICAL FIX: Convert RGB to BGR for OpenCV
        # ROS2 camera gives RGB, but OpenCV expects BGR
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        print(f"[HoopTrackingNode] Image shape: {image.shape}, dtype: {image.dtype}")
        print(f"[HoopTrackingNode] Image min/max: {image.min()}/{image.max()}")

        # Use exact same ranges as teammate's working ellipse_yolo algorithm
        # Key: Lower saturation threshold (50 instead of 120) to detect pale/pink hoops
        red_lower1 = np.array([0, 50, 50], dtype=np.uint8)      # deep red range
        red_upper1 = np.array([10, 255, 255], dtype=np.uint8)
        red_lower2 = np.array([170, 50, 50], dtype=np.uint8)    # orange-red range
        red_upper2 = np.array([180, 255, 255], dtype=np.uint8)

        try:
            detection = find_hoop(
                image,
                red_lower1,  # same range, no wraparound
                red_upper1,
                red_lower2,
                red_upper2,
                self.uuid,
                self.debug,
                self.save_vision
            )
            print(f"[HoopTrackingNode] find_hoop returned: {detection}")
        except Exception as e:
            print(f"[HoopTrackingNode] ERROR in find_hoop: {e}")
            import traceback
            traceback.print_exc()
            detection = None

        dlz_empty = False
        if detection is not None:
            cx, cy, dlz_empty, hoop_contour = detection
            x, y = cx, cy
        else:
            h, w = image.shape[:2]
            x, y = w / 2, h / 2
            
        # Compute 3D direction vector
        direction = compute_3d_vector(x, y, np.array(camera_info.k).reshape(3, 3), request.altitude)
            
        # Populate response
        response.x = float(x)
        response.y = float(y)
        response.direction = direction
        response.dlz_empty = dlz_empty
        return response

    # ------------------------------ Debug HUD rendering -------------------------------
    def _render_debug_hud(self, image, x, y, hoop_contour, direction, altitude, K):
        """
        Draw a professional HUD overlay with target/center/offset/angles, optional hoop contour,
        and publish to a debug topic + optional OpenCV window.
        """
        vis = image.copy()
        h, w = vis.shape[:2]
        center_x, center_y = w // 2, h // 2
        cx_img, cy_img = int(round(x)), int(round(y))

        # 1) crosshair at image center (BLUE)
        cv2.line(vis, (center_x - 30, center_y), (center_x + 30, center_y), (255, 0, 0), 2)
        cv2.line(vis, (center_x, center_y - 30), (center_x, center_y + 30), (255, 0, 0), 2)
        cv2.circle(vis, (center_x, center_y), 4, (255, 0, 0), -1)

        # 2) target marker + line from center (GREEN if detected, RED otherwise)
        detected = not (cx_img == center_x and cy_img == center_y)  # simple proxy
        # Prefer detection status from contour or (x,y) if you want stricter logic:
        # detected = hoop_contour is not None
        color_track = (0, 255, 0) if hoop_contour is not None else (0, 0, 255)
        cv2.circle(vis, (cx_img, cy_img), 8, color_track, -1)
        cv2.circle(vis, (cx_img, cy_img), 20, color_track, 2)
        cv2.line(vis, (center_x, center_y), (cx_img, cy_img), color_track, 2)

        # 2a) draw hoop contour if provided
        if hoop_contour is not None and isinstance(hoop_contour, np.ndarray) and hoop_contour.shape[0] >= 3:
            cv2.drawContours(vis, [hoop_contour], -1, (0, 255, 0), 2)

        # 3) pixel offsets
        off_x = cx_img - center_x
        off_y = cy_img - center_y
        off_r = float(np.hypot(off_x, off_y))

        # 4) angle errors from intrinsics (yaw/pitch)
        fx, fy, cx0, cy0 = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
        u, v = float(x), float(y)
        xn = (u - cx0) / max(1e-6, fx)
        yn = (v - cy0) / max(1e-6, fy)
        # small-angle approx: yaw ~ atan2(xn, 1), pitch ~ atan2(yn, 1)
        yaw_err_rad = np.arctan2(xn, 1.0)
        pitch_err_rad = np.arctan2(yn, 1.0)

        # 5) info panel overlay
        overlay = vis.copy()
        panel_w = 520
        panel_h = 250
        cv2.rectangle(overlay, (10, 10), (10 + panel_w, 10 + panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, vis, 0.5, 0, vis)

        y0, dy = 35, 28
        status = "TRACKING" if hoop_contour is not None else "SEARCHING"
        status_color = (0, 255, 0) if hoop_contour is not None else (0, 0, 255)
        cv2.putText(vis, f"Status: {status}", (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2); y0 += dy
        cv2.putText(vis, f"Target px: ({cx_img}, {cy_img})", (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2); y0 += dy
        cv2.putText(vis, f"Offset px: ({off_x:+d}, {off_y:+d}) | R={off_r:.1f}", (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2); y0 += dy
        cv2.putText(vis, f"Yaw/Pitch err: ({np.degrees(yaw_err_rad):+.2f} deg, {np.degrees(pitch_err_rad):+.2f} deg)",
                    (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2); y0 += dy
        cv2.putText(vis, f"Direction (cam): [{direction[0]:+.2f}, {direction[1]:+.2f}, {direction[2]:+.2f}]",
                    (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2); y0 += dy
        cv2.putText(vis, f"Altitude: {altitude:.2f} m", (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2); y0 += dy

        # Centered indicator
        centered = off_r < float(self.center_threshold_px)
        center_text = "CENTERED" if centered else f"OFF BY {off_r:.0f}px"
        center_color = (0, 255, 0) if centered else (0, 165, 255)
        cv2.putText(vis, center_text, (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.9, center_color, 2)

        # 6) corner ticks
        cs = 20
        cv2.line(vis, (0, 0), (cs, 0), (255, 255, 0), 2); cv2.line(vis, (0, 0), (0, cs), (255, 255, 0), 2)
        cv2.line(vis, (w - 1, 0), (w - 1 - cs, 0), (255, 255, 0), 2); cv2.line(vis, (w - 1, 0), (w - 1, cs), (255, 255, 0), 2)
        cv2.line(vis, (0, h - 1), (cs, h - 1), (255, 255, 0), 2); cv2.line(vis, (0, h - 1), (0, h - 1 - cs), (255, 255, 0), 2)
        cv2.line(vis, (w - 1, h - 1), (w - 1 - cs, h - 1), (255, 255, 0), 2); cv2.line(vis, (w - 1, h - 1), (w - 1, h - 1 - cs), (255, 255, 0), 2)

        # Publish debug image (for ground station)
        if self.debug_pub is not None and self._bridge is not None:
            try:
                msg = self._bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                self.debug_pub.publish(msg)
            except Exception:
                # Fail silently in debug path
                pass

        # Optional local window (can fail in headless setups)
        try:
            if not self.window_created:
                cv2.namedWindow("Hoop Tracking - Drone Camera", cv2.WINDOW_NORMAL)
                self.window_created = True
            cv2.imshow("Hoop Tracking - Drone Camera", vis)
            cv2.waitKey(1)
        except Exception:
            # Ignore GUI errors (e.g., no DISPLAY)
            pass

    # ------------------------------- Cleanup (optional) -------------------------------
    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = HoopTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("HoopTrackingNode exception:", e)
        node.publish_failsafe()
    finally:
        node.destroy_node()
        rclpy.shutdown()
