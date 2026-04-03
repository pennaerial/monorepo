import math
from typing import Optional

import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
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
        tag_size_m: float = 0.0508,
        tag_family: str = "tag36h11",
        # Maximum forward speed in m/s — hard cap regardless of distance.
        max_forward_speed: float = 0.2,
        # Proportional forward gain: speed = forward_gain * distance, capped at max_forward_speed.
        # Gives smooth slow-down as the payload closes in on the tag.
        forward_gain: float = 0.5,
        # Steering gain: rad/s of yaw rate per pixel of lateral error.
        # Tune this to reduce oscillation — lower = smoother but slower correction.
        angular_gain: float = 0.003,
        # Orthogonal-approach gain: rad/s per radian of yaw misalignment vs tag normal.
        # Set to 0.0 to disable (pure lateral correction only). Start at 0 to verify signs
        # in the HUD before enabling. Negate if correction curves the wrong direction.
        yaw_gain: float = 0.0,
        # Distance (meters) at which to declare "arrived" and stop.
        stop_distance_m: float = 0.2,
        # How long (seconds) to coast after losing the tag before stopping.
        # Gives the detector a moment to recover from a brief dropout.
        tag_lost_coast_s: float = 0.5,
        # Show a live cv2 debug window with tag overlay, steering arrow, and pose info.
        camera_debug: bool = False,
        compressed_image: bool = False,  # Whether the camera topic is compressed (affects CvBridge encoding)
    ):
        super().__init__(node, uav)
        self.payload_name = payload_name
        self.tag_id = tag_id
        self.tag_size_m = tag_size_m
        self.max_forward_speed = max_forward_speed
        self.forward_gain = forward_gain
        self.angular_gain = angular_gain
        self.yaw_gain = yaw_gain
        self.stop_distance_m = stop_distance_m
        self.tag_lost_coast_s = tag_lost_coast_s
        self.camera_debug = camera_debug
        self.compressed_image = compressed_image

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

        # Compressed images live on a /compressed sub-topic (image_transport convention).
        # Sim always uses raw Image — compressed_image should be False in sim.
        cam_topic = f"/{self.payload_name}/camera"
        info_topic = f"/{self.payload_name}/camera_info"
        drive_topic = f"/{self.payload_name}/cmd_drive"

        if self.compressed_image:
            self._image_sub = self.node.create_subscription(
                CompressedImage, cam_topic, self._image_cb, 10
            )
        else:
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

        if self.camera_debug and not self._done:
            cv2.destroyWindow("AprilTagApproachMode")

        if not self._done:
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

    def _image_cb(self, msg) -> None:
        if self._detector is None or self._camera_info is None:
            return

        # Decode the incoming message into a BGR color frame and a grayscale frame.
        # CompressedImage (real hardware) uses np.frombuffer + cv2.imdecode.
        # Raw Image (sim) uses CvBridge.
        if self.compressed_image:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if self.camera_debug:
                frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                frame = None
                gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        else:
            if self.camera_debug:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                frame = None
                gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        detections = self._detector.detect(gray)

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

        ok, rvec, tvec = cv2.solvePnP(
            _object_points(self.tag_size_m),
            det.corners.astype(np.float32),
            camera_matrix,
            dist_coeffs,
        )
        if not ok:
            return

        # tvec[2] = forward distance in camera frame (meters).
        distance = float(tvec[2])
        self.node.get_logger().info(f"Tag detected at distance {distance:.3f} m (stop at {self.stop_distance_m:.3f} m)")

        # --- Lateral steering correction ---
        # lateral_error > 0 means the tag is to the right of center.
        # DriveCommand.angular > 0 turns left, so we negate to steer toward the tag.
        tag_center_x = det.center[0]
        lateral_error_px = tag_center_x - cx
        lateral_correction = -self.angular_gain * lateral_error_px

        # --- Yaw (orthogonal approach) correction ---
        # Compute the tag's surface normal in camera frame from the rotation matrix.
        # When facing the tag head-on, the normal points toward -Z of camera → nx ≈ 0.
        # nx > 0: approaching from payload's left → turn right (negative correction).
        # nx < 0: approaching from payload's right → turn left (positive correction).
        # yaw_gain defaults to 0.0 so this has no effect until you verify signs in the HUD.
        R, _ = cv2.Rodrigues(rvec)
        yaw_error = math.atan2(R[0, 2], -R[2, 2])  # radians; 0 = perfectly orthogonal
        yaw_correction = -self.yaw_gain * yaw_error

        angular = lateral_correction + yaw_correction

        self._last_tag_time = self.node.get_clock().now().nanoseconds * 1e-9

        if self.camera_debug:
            self._draw_debug(frame, gray, det, rvec, tvec, camera_matrix, cx, cy, lateral_error_px, angular, distance, yaw_error, yaw_correction)

        # --- Stop condition ---
        if distance <= self.stop_distance_m:
            if not self._done:
                self.log(f"Reached tag (distance={distance:.3f}m) — complete")
                self._publish_drive(0.0, 0.0)
                self._done = True
            return  # Keep debug feed running, just don't drive


        linear = min(self.forward_gain * distance, self.max_forward_speed)
        self.node.get_logger().info(f"linear={linear:.3f} m/s, angular={angular:.6f} rad/s")

        self._publish_drive(linear, angular)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _draw_debug(
        self,
        color_frame: Optional[np.ndarray],
        gray: np.ndarray,
        det,
        rvec: np.ndarray,
        tvec: np.ndarray,
        camera_matrix: np.ndarray,
        cx: float,
        cy: float,
        lateral_error_px: float,
        angular: float,
        distance: float,
        yaw_error: float = 0.0,
        yaw_correction: float = 0.0,
    ) -> None:
        # Use the color frame if available (real camera), otherwise convert gray to BGR.
        vis = color_frame.copy() if color_frame is not None else cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        h, w = vis.shape[:2]

        corners = det.corners.astype(int)

        # --- Tag border and corners ---
        for i in range(4):
            cv2.line(vis, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)

        # Corner pixel coordinates as small labels
        for i, (px, py) in enumerate(corners):
            cv2.circle(vis, (px, py), 4, (0, 255, 0), -1)
            cv2.putText(vis, f"({px},{py})", (px + 4, py - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

        # --- Tag center dot and ID ---
        tag_cx, tag_cy = int(det.center[0]), int(det.center[1])
        cv2.circle(vis, (tag_cx, tag_cy), 6, (0, 0, 255), -1)
        cv2.putText(vis, f"id={det.tag_id}", (tag_cx + 8, tag_cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # --- Image center crosshair ---
        img_cx, img_cy = int(cx), int(cy)
        cv2.drawMarker(vis, (img_cx, img_cy), (255, 255, 255),
                       cv2.MARKER_CROSS, 20, 1)

        # --- Steering arrow: image center → tag center ---
        # Direction shows where angular correction is pointing the robot.
        cv2.arrowedLine(vis, (img_cx, img_cy), (tag_cx, tag_cy),
                        (0, 165, 255), 2, tipLength=0.15)
        cv2.putText(vis, "steer", ((img_cx + tag_cx) // 2 + 4, (img_cy + tag_cy) // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)

        # --- 3D pose axes projected onto image (X=red, Y=green, Z=blue) ---
        # Axes arms are half the tag size long.
        axis_len = self.tag_size_m * 0.5
        axes_3d = np.float32([
            [0, 0, 0],
            [axis_len, 0, 0],  # X
            [0, axis_len, 0],  # Y
            [0, 0, axis_len],  # Z (toward camera = into the tag)
        ])
        projected, _ = cv2.projectPoints(axes_3d, rvec, tvec, camera_matrix,
                                         np.zeros((4, 1)))
        projected = projected.reshape(-1, 2).astype(int)
        origin = tuple(projected[0])
        cv2.arrowedLine(vis, origin, tuple(projected[1]), (0, 0, 255), 2, tipLength=0.3)   # X red
        cv2.arrowedLine(vis, origin, tuple(projected[2]), (0, 255, 0), 2, tipLength=0.3)   # Y green
        cv2.arrowedLine(vis, origin, tuple(projected[3]), (255, 0, 0), 2, tipLength=0.3)   # Z blue

        # --- HUD: distance (large), then small detail lines ---
        tx, ty, tz = float(tvec[0]), float(tvec[1]), float(tvec[2])
        cv2.putText(vis, f"dist: {distance:.3f} m  (stop @ {self.stop_distance_m:.3f} m)",
                    (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 255), 2)
        small_lines = [
            f"tvec:     x={tx:.3f}  y={ty:.3f}  z={tz:.3f}",
            f"lat err:  {lateral_error_px:+.1f} px",
            f"angular:  {angular:+.4f} rad/s",
            f"yaw err:  {yaw_error:+.3f} rad  (0=orthogonal)",
            f"yaw cmd:  {yaw_correction:+.4f} rad/s  (gain={self.yaw_gain})",
        ]
        for i, line in enumerate(small_lines):
            cv2.putText(vis, line, (8, 60 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        cv2.imshow("AprilTagApproachMode", vis)
        cv2.waitKey(1)

    def _publish_drive(self, linear: float, angular: float) -> None:
        if self._drive_pub is None:
            return
        cmd = DriveCommand()    
        cmd.linear = float(linear)
        cmd.angular = float(angular)
        self._drive_pub.publish(cmd)
