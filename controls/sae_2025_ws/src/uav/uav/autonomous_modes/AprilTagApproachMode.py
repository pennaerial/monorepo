from typing import Optional

import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from payload_interfaces.msg import DriveCommand
from cv_bridge import CvBridge

from uav.autonomous_modes import Mode
from uav import UAV

try:
    import apriltag
except ImportError:
    apriltag = None


def _object_points(tag_size_m: float) -> np.ndarray:
    """
    3D corner positions of the tag in tag-local frame (X-right, Y-up, Z-out).
    Order matches the apriltag library's corner output: TL, TR, BR, BL.
    Used by solvePnP to estimate camera-to-tag pose.
    """
    h = tag_size_m / 2.0
    return np.array(
        [[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32
    )


class AprilTagApproachMode(Mode):
    """
    Drives the payload straight toward a visible AprilTag.

    Assumes the payload starts roughly perpendicular to and facing the target
    (pre-aligned), so the tag should already be near-centered in the camera.
    A proportional steering correction keeps the tag centered while driving
    forward at a fixed speed. Stops when the tag is within stop_distance_m.

    Sim/real transparency: publishes DriveCommand to /{payload_name}/cmd_drive.
    The payload node routes this to SimController (Gazebo) or GPIOController
    (hardware) — this mode never needs to know which one is running.
    """

    def __init__(
        self,
        node: Node,
        uav: UAV,
        payload_name: str = "payload_0",
        # Set to a specific tag ID to track only that tag; None = any visible tag.
        tag_id: Optional[int] = None,
        tag_size_m: float = 0.1,
        tag_family: str = "tag36h11",
        # Forward speed in m/s while approaching.
        forward_speed: float = 0.15,
        # Steering gain: rad/s of yaw rate per pixel of lateral error.
        # Tune this to reduce oscillation — lower = smoother but slower correction.
        angular_gain: float = 0.003,
        # Distance (meters) at which to declare "arrived" and stop.
        stop_distance_m: float = 0.3,
        # How long (seconds) to coast after losing the tag before stopping.
        # Gives the detector a moment to recover from a brief dropout.
        tag_lost_coast_s: float = 0.5,
    ):
        super().__init__(node, uav)
        self.payload_name = payload_name
        self.tag_id = tag_id
        self.tag_size_m = tag_size_m
        self.forward_speed = forward_speed
        self.angular_gain = angular_gain
        self.stop_distance_m = stop_distance_m
        self.tag_lost_coast_s = tag_lost_coast_s

        self._bridge = CvBridge()
        self._camera_info: Optional[CameraInfo] = None
        self._done = False
        self._last_tag_time: Optional[float] = None

        # Subscriptions and publisher created in on_enter, destroyed in on_exit.
        self._image_sub = None
        self._info_sub = None
        self._drive_pub = None

        # Build the AprilTag detector once at init.
        self._detector = None
        if apriltag is None:
            self.node.get_logger().warn(
                "AprilTagApproachMode: 'apriltag' library not found — detection disabled"
            )
        else:
            try:
                options = apriltag.DetectorOptions(
                    families=tag_family, refine_edges=True
                )
                self._detector = apriltag.Detector(options)
            except (AttributeError, TypeError):
                self._detector = apriltag.Detector()

    # ------------------------------------------------------------------
    # Mode lifecycle
    # ------------------------------------------------------------------

    def on_enter(self) -> None:
        self._done = False
        self._camera_info = None
        self._last_tag_time = None

        cam_topic = f"/{self.payload_name}/camera"
        info_topic = f"/{self.payload_name}/camera_info"
        drive_topic = f"/{self.payload_name}/cmd_drive"

        self._image_sub = self.node.create_subscription(
            Image, cam_topic, self._image_cb, 10
        )
        self._info_sub = self.node.create_subscription(
            CameraInfo, info_topic, self._info_cb, 10
        )
        self._drive_pub = self.node.create_publisher(DriveCommand, drive_topic, 10)

        self.log(f"Subscribed to {cam_topic}, publishing to {drive_topic}")

    def on_exit(self) -> None:
        # Always stop the payload when leaving this mode.
        self._publish_drive(0.0, 0.0)

        self.node.destroy_subscription(self._image_sub)
        self.node.destroy_subscription(self._info_sub)
        self.node.destroy_publisher(self._drive_pub)

    def on_update(self, time_delta: float) -> None:
        # The approach loop runs reactively in _image_cb (once per camera frame).
        # on_update handles the tag-lost timeout: if we haven't seen the tag in
        # tag_lost_coast_s seconds, stop rather than driving blind.
        if self._last_tag_time is None:
            return  # Haven't acquired the tag yet — wait.

        now = self.node.get_clock().now().nanoseconds * 1e-9
        if now - self._last_tag_time > self.tag_lost_coast_s:
            self.log("Tag lost — stopping")
            self._publish_drive(0.0, 0.0)

    def check_status(self) -> str:
        # ModeManager convention: "continue" = stay in this mode.
        # "complete" must match the transition key in the mission YAML.
        return "complete" if self._done else "continue"

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _info_cb(self, msg: CameraInfo) -> None:
        # Store camera intrinsics for use in solvePnP distance estimation.
        # In sim: provided automatically by Gazebo bridge.
        # In real: provided by v4l2_camera or a calibration node.
        self._camera_info = msg

    def _image_cb(self, msg: Image) -> None:
        if self._detector is None or self._camera_info is None:
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        detections = self._detector.detect(frame)

        # Filter to the target tag id if one was specified.
        if self.tag_id is not None:
            detections = [d for d in detections if d.tag_id == self.tag_id]

        if not detections:
            return  # on_update will handle the coast/stop timeout.

        # If multiple tags are visible, use the largest (closest) one.
        det = max(
            detections,
            key=lambda d: cv2.contourArea(d.corners.reshape(-1, 1, 2).astype(np.float32)),
        )

        # --- Distance estimation via solvePnP ---
        # Extract intrinsics from camera_info K matrix:
        #   K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]  (row-major, 9 elements)
        fx = self._camera_info.k[0]
        fy = self._camera_info.k[4]
        cx = self._camera_info.k[2]
        cy = self._camera_info.k[5]
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        dist_coeffs = np.zeros((4, 1), dtype=np.float64)

        ok, _, tvec = cv2.solvePnP(
            _object_points(self.tag_size_m),
            det.corners.astype(np.float32),
            camera_matrix,
            dist_coeffs,
        )
        if not ok:
            return

        # tvec[2] = forward distance in camera frame (meters).
        distance = float(tvec[2])

        # --- Lateral steering correction ---
        # lateral_error > 0 means the tag is to the right of center.
        # DriveCommand.angular > 0 turns left, so we negate to steer toward the tag.
        tag_center_x = det.center[0]
        lateral_error_px = tag_center_x - cx
        angular = -self.angular_gain * lateral_error_px

        self._last_tag_time = self.node.get_clock().now().nanoseconds * 1e-9

        # --- Stop condition ---
        if distance <= self.stop_distance_m:
            self.log(f"Reached tag (distance={distance:.2f}m) — complete")
            self._publish_drive(0.0, 0.0)
            self._done = True
            return

        self._publish_drive(self.forward_speed, angular)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_drive(self, linear: float, angular: float) -> None:
        if self._drive_pub is None:
            return
        cmd = DriveCommand()    
        cmd.linear = float(linear)
        cmd.angular = float(angular)
        self._drive_pub.publish(cmd)
