from typing import Optional

import numpy as np
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from payload_interfaces.msg import DriveCommand
from uav.autonomous_modes import Mode
from uav import UAV
from cv_bridge import CvBridge

try:
    import apriltag
except ImportError:
    apriltag = None

# Tag family for detection (tag36h11 is common and robust)
DEFAULT_TAG_FAMILY = "tag36h11"

# Object points for solvePnP: tag frame, order TL, TR, BR, BL (y up).
# Built at runtime from tag_size_m: half = tag_size_m/2.
def _object_points_for_tag_size(tag_size_m: float) -> np.ndarray:
    half = tag_size_m / 2.0
    return np.array(
        [
            [-half, half, 0],   # Top-left
            [half, half, 0],    # Top-right
            [half, -half, 0],   # Bottom-right
            [-half, -half, 0],  # Bottom-left
        ],
        dtype=np.float32,
    )


class PayloadDriveToAprilTagMode(Mode):
    """
    Mode for driving the payload using its camera to find an AprilTag and
    stopping at a given distance in front of it. VTOL remains stationary.
    If tag_id is set, only that tag is targeted; if None, any detected tag is used.
    Uses tag36h11 family and pose estimation matching common AprilTag scripts.
    """

    def __init__(
        self,
        node: Node,
        uav: UAV,
        payload_name: str = "payload_0",
        tag_id: Optional[int] = None,
        stop_distance_m: float = 0.01,
        tag_size_m: float = 0.1,  # 100 mm x 100 mm
        tag_family: str = DEFAULT_TAG_FAMILY,
        linear_gain: float = 0.3,
        angular_gain: float = 0.002,
    ):
        super().__init__(node, uav)
        self.payload_name = str(payload_name)
        self.tag_id = tag_id
        self.stop_distance_m = float(stop_distance_m)
        self.tag_size_m = float(tag_size_m)
        self.tag_family = str(tag_family) if tag_family else DEFAULT_TAG_FAMILY
        self.linear_gain = linear_gain
        self.angular_gain = angular_gain
        self.done = False
        self._image = None
        self._camera_info = None
        self._bridge = CvBridge()
        self._detector = None
        if apriltag is not None:
            try:
                options = apriltag.DetectorOptions(
                    families=self.tag_family,
                    refine_edges=True,
                )
                self._detector = apriltag.Detector(options)
            except (AttributeError, TypeError):
                self._detector = apriltag.Detector()

    def on_enter(self) -> None:
        self._image = None
        self._camera_info = None
        self.done = False
        cam_topic = f"/{self.payload_name}/camera"
        info_topic = f"/{self.payload_name}/camera_info"
        self.node.get_logger().info(
            f"PayloadDriveToAprilTagMode: subscribing to {cam_topic}, {info_topic}"
        )
        self._image_sub = self.node.create_subscription(
            Image,
            cam_topic,
            self._image_cb,
            10,
        )
        self._info_sub = self.node.create_subscription(
            CameraInfo,
            info_topic,
            self._info_cb,
            10,
        )
        drive_topic = f"/{self.payload_name}/cmd_drive"
        self._drive_pub = self.node.create_publisher(
            DriveCommand,
            drive_topic,
            10,
        )
        self._first_image_logged = False
        self._last_wait_log_time = 0.0
        self._last_no_tag_log_time = 0.0
        self._last_drive_log_time = 0.0
        self.node.get_logger().info(
            f"PayloadDriveToAprilTagMode: publishing drive commands to {drive_topic}"
        )

    def _image_cb(self, msg: Image) -> None:
        self._image = msg

    def _info_cb(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def on_exit(self) -> None:
        if hasattr(self, "_drive_pub"):
            self._drive_pub.publish(DriveCommand(linear=0.0, angular=0.0))

    def on_update(self, time_delta: float) -> None:
        import time as _time
        now = _time.time()
        if self.done:
            return
        if self._detector is None:
            self.log("PayloadDriveToAprilTagMode: apriltag not installed; cannot detect tags.")
            return
        if self._image is None or self._camera_info is None:
            if now - getattr(self, "_last_wait_log_time", 0) >= 2.0:
                self._last_wait_log_time = now
                self.log(
                    f"PayloadDriveToAprilTagMode: waiting for payload camera "
                    f"(image={self._image is not None}, camera_info={self._camera_info is not None})"
                )
            return
        if not getattr(self, "_first_image_logged", False):
            self._first_image_logged = True
            self.log("PayloadDriveToAprilTagMode: first image received from payload camera")
        try:
            gray = self._bridge.imgmsg_to_cv2(self._image, desired_encoding="mono8")
        except Exception as e:
            self.node.get_logger().warn(f"Could not convert image: {e}")
            return
        K = np.array(self._camera_info.k, dtype=np.float64).reshape(3, 3)
        d = getattr(self._camera_info, "d", None) or []
        dist_coeffs = np.array(d, dtype=np.float64)
        if dist_coeffs.size == 0:
            dist_coeffs = np.zeros(5)
        if dist_coeffs.shape != (5,):
            dist_coeffs = np.resize(np.asarray(dist_coeffs).reshape(-1), 5)

        detections = self._detector.detect(gray)
        target = None
        if self.tag_id is not None:
            for d in detections:
                if d.tag_id == self.tag_id:
                    target = d
                    break
        elif detections:
            target = detections[0]
        if target is None:
            if now - getattr(self, "_last_no_tag_log_time", 0) >= 2.0:
                self._last_no_tag_log_time = now
                if self.tag_id is not None:
                    self.log("PayloadDriveToAprilTagMode: no tag id=%s, sending angular=0.2" % self.tag_id)
                else:
                    self.log("PayloadDriveToAprilTagMode: no AprilTag in view, sending angular=0.2")
            self._drive_pub.publish(DriveCommand(linear=0.0, angular=0.2))
            return

        # 3D object points in tag frame (TL, TR, BR, BL), matching reference script
        obj_pts = _object_points_for_tag_size(self.tag_size_m)
        corners = np.array(target.corners, dtype=np.float32)
        if len(corners) != 4:
            return
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts,
            corners,
            K,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return
        # Forward distance (Z) in camera frame, matching reference script
        tvec_flat = np.asarray(tvec).ravel()
        distance = float(tvec_flat[2]) if len(tvec_flat) >= 3 else 0.0
        if distance <= self.stop_distance_m:
            self._drive_pub.publish(DriveCommand(linear=0.0, angular=0.0))
            self.done = True
            self.log(f"PayloadDriveToAprilTagMode: done, stopped at {distance:.3f} m in front of tag {target.tag_id}")
            return

        h, w = gray.shape[:2]
        cx_img = w / 2.0
        err_x = target.center[0] - cx_img
        linear = np.clip(
            self.linear_gain * (distance - self.stop_distance_m),
            0.0,
            0.5,
        )
        angular = np.clip(-self.angular_gain * err_x, -0.5, 0.5)
        if now - getattr(self, "_last_drive_log_time", 0) >= 1.0:
            self._last_drive_log_time = now
            self.log(
                f"PayloadDriveToAprilTagMode: tag_id={target.tag_id} distance={distance:.3f} "
                f"linear={linear:.2f} angular={angular:.2f}"
            )
        self._drive_pub.publish(DriveCommand(linear=linear, angular=angular))

    def check_status(self) -> str:
        if self.done:
            return "terminate"
        return "continue"
