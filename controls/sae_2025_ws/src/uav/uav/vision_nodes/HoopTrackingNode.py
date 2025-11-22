#!/usr/bin/env python3
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

# -----------------------------
# CONFIG: physical hoop radius
# -----------------------------
# Set this to the real hoop radius in meters (match your Gazebo model)
HOOP_RADIUS_M = 0.5


# -----------------------------
# PnP helper functions
# -----------------------------
def extract_hoop_keypoints_from_contour(contour: np.ndarray) -> np.ndarray | None:
    """
    Given a hoop contour from OpenCV (shape Nx1x2 or Nx2),
    return 4 keypoints on the rim:
        [rightmost, leftmost, topmost, bottommost]
    as a (4, 2) float32 array of pixel coordinates [u, v].
    """
    if contour is None or len(contour) == 0:
        return None

    # Ensure shape (N, 2)
    contour_xy = contour.reshape(-1, 2)

    # Rightmost (max x)
    right = contour_xy[np.argmax(contour_xy[:, 0])]
    # Leftmost (min x)
    left = contour_xy[np.argmin(contour_xy[:, 0])]
    # Topmost (min y)
    top = contour_xy[np.argmin(contour_xy[:, 1])]
    # Bottommost (max y)
    bottom = contour_xy[np.argmax(contour_xy[:, 1])]

    keypoints = np.stack([right, left, top, bottom], axis=0).astype(np.float32)
    return keypoints


def estimate_hoop_pose_pnp(
    keypoints_2d: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    hoop_radius_m: float,
    flags: int = cv2.SOLVEPNP_ITERATIVE,
):
    """
    Estimate hoop pose in camera frame using PnP.

    Args:
        keypoints_2d: (4, 2) array of pixel coords [u, v]
                      must correspond to [right, left, top, bottom] on the hoop rim.
        camera_matrix: 3x3 intrinsic matrix
        dist_coeffs:   distortion coefficients (1xN)
        hoop_radius_m: physical hoop radius in meters

    Returns:
        (rvec, tvec, R) or None on failure.
    """
    if keypoints_2d is None or keypoints_2d.shape != (4, 2):
        return None

    # Known 3D points in hoop frame (XY plane, Z=0)
    # Order must match keypoints_2d: [right, left, top, bottom]
    object_points_3d = np.array(
        [
            [hoop_radius_m, 0.0, 0.0],   # right
            [-hoop_radius_m, 0.0, 0.0],  # left
            [0.0, hoop_radius_m, 0.0],   # top
            [0.0, -hoop_radius_m, 0.0],  # bottom
        ],
        dtype=np.float32,
    )

    obj = object_points_3d.reshape(-1, 1, 3)
    img = keypoints_2d.reshape(-1, 1, 2)

    success, rvec, tvec = cv2.solvePnP(
        obj,
        img,
        camera_matrix,
        dist_coeffs,
        flags=flags,
    )
    if not success:
        return None

    R, _ = cv2.Rodrigues(rvec)
    return rvec, tvec, R


def estimate_hoop_pose_from_contour(
    contour: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    hoop_radius_m: float,
    flags: int = cv2.SOLVEPNP_ITERATIVE,
):
    """
    Convenience wrapper:
      contour → 4 keypoints → PnP → (keypoints_2d, rvec, tvec, R)

    Returns:
        (keypoints_2d, rvec, tvec, R) or None if anything fails.
    """
    keypoints_2d = extract_hoop_keypoints_from_contour(contour)
    if keypoints_2d is None:
        return None

    pose = estimate_hoop_pose_pnp(
        keypoints_2d,
        camera_matrix,
        dist_coeffs,
        hoop_radius_m,
        flags=flags,
    )
    if pose is None:
        return None

    rvec, tvec, R = pose
    return keypoints_2d, rvec, tvec, R


class HoopTrackingNode(VisionNode):
    """
    ROS2 node for hoop tracking with optional PnP pose estimation.
    """
    srv = HoopTracking

    def __init__(self):
        super().__init__('hoop_tracking', self.__class__.srv)

        self.color_map = {
            'pink': pink,
            'green': green,
            'blue': blue,
            'yellow': yellow,
            'red': red,
            'orange': orange,
        }

        self.kalman = cv2.KalmanFilter(4, 2)
        self._setup_kalman_filter()

        # Debug window + publisher setup
        self.window_created = False
        self._bridge = CvBridge() if CvBridge is not None else None
        self.debug_pub = None
        if Image is not None and self._bridge is not None:
            self.debug_pub = self.create_publisher(Image, '/debug/hoop_tracking', 10)

        self.center_threshold_px = 50

        self.create_service(HoopTracking, self.service_name(), self.service_callback)

    # ---------------- Kalman (placeholder) ----------------
    def _setup_kalman_filter(self):
        dt = 1.0
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
        """
        Process tracking service request:
        - find hoop in image
        - compute direction vector (legacy)
        - use PnP to estimate hoop pose in camera frame
        - output distance drone↔hoop center
        """
        image_msg, camera_info = self.request_data(cam_image=True, cam_info=True)
        image = self.convert_image_msg_to_frame(image_msg)

        # ROS image is RGB; OpenCV expects BGR
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # HSV ranges for red/orange hoop
        red_lower1 = np.array([0, 50, 50], dtype=np.uint8)
        red_upper1 = np.array([10, 255, 255], dtype=np.uint8)
        red_lower2 = np.array([170, 50, 50], dtype=np.uint8)
        red_upper2 = np.array([180, 255, 255], dtype=np.uint8)

        try:
            detection = find_hoop(
                image,
                red_lower1,
                red_upper1,
                red_lower2,
                red_upper2,
                self.uuid,
                self.debug,
                self.save_vision,
            )
            # Expecting: (cx, cy, dlz_empty, hoop_contour)
            print(f"[HoopTrackingNode] find_hoop returned: {detection}")
        except Exception as e:
            print(f"[HoopTrackingNode] ERROR in find_hoop: {e}")
            import traceback
            traceback.print_exc()
            detection = None

        dlz_empty = False
        hoop_contour = None

        if detection is not None:
            cx, cy, dlz_empty, hoop_contour = detection
            x, y = cx, cy
        else:
            # No detection: fall back to image center
            h, w = image.shape[:2]
            x, y = w / 2.0, h / 2.0

        # Legacy direction vector (camera → hoop) using your existing helper
        K = np.array(camera_info.k, dtype=np.float32).reshape(3, 3)
        direction = compute_3d_vector(x, y, K, request.altitude)

        # ---------------------- PnP distance computation ----------------------
        # Default: no pose
        has_pose = False
        hoop_cam_x = hoop_cam_y = hoop_cam_z = distance_m = 0.0

        if hoop_contour is not None:
            # For simulation, we can assume zero distortion
            dist_coeffs = np.zeros((5, 1), dtype=np.float32)

            pnp_result = estimate_hoop_pose_from_contour(
                hoop_contour,
                K,
                dist_coeffs,
                HOOP_RADIUS_M,
            )

            if pnp_result is not None:
                keypoints_2d, rvec, tvec, R = pnp_result
                t_cam = tvec.reshape(3)  # [X, Y, Z] hoop center in camera frame (meters)

                hoop_cam_x = float(t_cam[0])
                hoop_cam_y = float(t_cam[1])
                hoop_cam_z = float(t_cam[2])
                distance_m = float(np.linalg.norm(t_cam))
                has_pose = True

                print(
                    f"[HoopTrackingNode] PnP hoop pose (cam): "
                    f"X={hoop_cam_x:.2f}, Y={hoop_cam_y:.2f}, Z={hoop_cam_z:.2f}, "
                    f"Dist={distance_m:.2f} m"
                )

        # ---------------------- Populate response ----------------------
        response.x = float(x)
        response.y = float(y)
        response.direction = direction
        response.dlz_empty = dlz_empty

        # New pose/distance fields (ensure these exist in HoopTracking.srv)
        response.has_pose = bool(has_pose)
        response.hoop_cam_x = float(hoop_cam_x)
        response.hoop_cam_y = float(hoop_cam_y)
        response.hoop_cam_z = float(hoop_cam_z)
        response.distance = float(distance_m)

        # Optional: render debug HUD with direction, etc.
        try:
            self._render_debug_hud(
                image,
                x,
                y,
                hoop_contour,
                direction,
                request.altitude,
                K,
            )
        except Exception:
            # Don't let debug drawing crash the service
            pass

        return response

    # ------------------------------ Debug HUD rendering -------------------------------
    # (unchanged from your version; omitted here for brevity, but keep your existing
    #  _render_debug_hud implementation)
    # ...

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