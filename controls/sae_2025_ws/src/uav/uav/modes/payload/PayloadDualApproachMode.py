"""
PayloadDualApproachMode: single-mode approach that fuses AprilTag tracking
with HSV colour-blob tracking for a string obstacle.

Every frame the mode attempts AprilTag detection first (provides yaw
correction).  When no tag is visible it falls back to HSV colour tracking
of the string.  Termination is based on the pixel height of the colour
blob's bounding box exceeding ``stop_height_px``.

If both the tag and the colour target are lost for ``lost_timeout_s``
seconds the payload stops.
"""

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from uav.vehicles.Payload import Payload
from uav.vision_nodes.payload_perception_common import (
    DEFAULT_TAG_FAMILY,
    AprilTagDetectorCache,
    detect_payload_apriltags,
)

from ..Mode import Mode


def _color_blob_info(
    mask: np.ndarray,
    min_area: float,
) -> Optional[Tuple[float, float, int, int]]:
    """Largest contour centroid, area, and bounding-box height.

    Returns ``(cx, cy, area, bbox_height)`` or ``None``.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in contours:
        a = cv2.contourArea(cnt)
        if a < min_area:
            continue
        if best is None or a > best[0]:
            best = (a, cnt)
    if best is None:
        return None
    a, cnt = best
    M = cv2.moments(cnt)
    if M["m00"] <= 0:
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    _, _, _, bh = cv2.boundingRect(cnt)
    return (cx, cy, int(a), bh)


class PayloadDualApproachMode(Mode[Payload]):
    """Approach using AprilTag when visible, HSV colour fallback otherwise."""

    required_vision_nodes = ()
    requires_camera = True
    transition_labels = ()

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        # --- AprilTag params ---
        tag_id: Optional[int] = 0,
        tag_size_m: float = 0.0508,
        tag_family: str = DEFAULT_TAG_FAMILY,
        tag_forward_gain: float = 0.5,
        tag_max_forward_speed: float = 0.2,
        tag_angular_gain: float = 0.003,
        tag_yaw_gain: float = 0.6,
        # --- Colour params ---
        lower_hsv: tuple[int, int, int] = (20, 100, 100),
        upper_hsv: tuple[int, int, int] = (35, 255, 255),
        color_forward_speed: float = 0.015,
        color_angular_gain: float = 0.003,
        color_centering_threshold_px: int = 15,
        color_drive_angular_scale: float = 0.0,
        color_min_detect_pixels: int = 50,
        # --- Termination / lost ---
        prime_height_px: int = 100,
        stop_height_px: int = 300,
        post_prime_lost_frames: int = 5,
        lost_timeout_s: float = 5.0,
        # --- Common ---
        compressed_image: bool = False,
    ):
        super().__init__(node, vehicle)

        # AprilTag
        self.tag_id = None if tag_id is None else int(tag_id)
        self.tag_size_m = float(tag_size_m)
        self.tag_family = str(tag_family) if tag_family else DEFAULT_TAG_FAMILY
        self.tag_forward_gain = float(tag_forward_gain)
        self.tag_max_forward_speed = float(tag_max_forward_speed)
        self.tag_angular_gain = float(tag_angular_gain)
        self.tag_yaw_gain = float(tag_yaw_gain)

        # Colour
        self._lower_hsv = np.array(lower_hsv, dtype=np.uint8)
        self._upper_hsv = np.array(upper_hsv, dtype=np.uint8)
        self.color_forward_speed = float(color_forward_speed)
        self.color_angular_gain = float(color_angular_gain)
        self.color_centering_threshold_px = int(color_centering_threshold_px)
        self.color_drive_angular_scale = float(color_drive_angular_scale)
        self.color_min_detect_pixels = int(color_min_detect_pixels)

        # Termination
        self.prime_height_px = int(prime_height_px)
        self.stop_height_px = int(stop_height_px)
        self.post_prime_lost_frames = int(post_prime_lost_frames)
        self.lost_timeout_s = float(lost_timeout_s)

        # Internals
        self._bridge = CvBridge()
        self._compressed_image = bool(compressed_image)
        self._detector_cache = AprilTagDetectorCache()
        self._detector = self._detector_cache.get(self.tag_family)

        self._latest_image: Optional[Image | CompressedImage] = None
        self._latest_camera_info: Optional[CameraInfo] = None

        cam_topic = vehicle.namespaced_path("camera")
        if self._compressed_image:
            self.node.create_subscription(
                CompressedImage,
                f"{cam_topic}/compressed",
                self._on_image,
                1,
            )
        else:
            self.node.create_subscription(Image, cam_topic, self._on_image, 1)

        self.node.create_subscription(
            CameraInfo, vehicle.camera_info_topic, self._on_camera_info, 1
        )

        self._debug_pub = None
        if getattr(self.node, "vision_debug", False):
            self._debug_pub = self.node.create_publisher(
                CompressedImage,
                vehicle.namespaced_path("annotated_image/compressed"),
                1,
            )

        self._done = False
        self._image_width = 0.0
        self._last_seen_time: Optional[float] = None
        self._last_log_time = 0.0
        self._primed = False
        self._lost_frames_after_prime = 0
        self._last_phase: Optional[str] = None

    # ---- subscriptions ----

    def _on_image(self, msg) -> None:
        self._latest_image = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._latest_camera_info is None:
            self._latest_camera_info = msg

    def _get_bgr(self) -> Optional[np.ndarray]:
        msg = self._latest_image
        if msg is None:
            return None
        if self._compressed_image:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    # ---- debug visualisation ----

    def _publish_debug(
        self,
        bgr: np.ndarray,
        mask: np.ndarray,
        source: str,
        tag,
        color_cx: Optional[float],
        color_cy: Optional[float],
        color_height_px: int,
    ) -> None:
        if self._debug_pub is None or self._debug_pub.get_subscription_count() == 0:
            return
        debug = bgr.copy()

        # Draw HSV colour contours (green) and centroid (red)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, contours, -1, (0, 255, 0), 1)
        if color_cx is not None and color_cy is not None:
            cv2.circle(debug, (int(color_cx), int(color_cy)), 5, (0, 0, 255), -1)
            cv2.putText(
                debug,
                f"color height: {color_height_px}px",
                (int(color_cx) + 8, int(color_cy)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )

        # Draw apriltag detection (cyan crosshair + label)
        if tag is not None:
            tx, ty = int(tag.center_x), int(tag.center_y)
            r = max(5, int(tag.area**0.5 / 4))
            cv2.circle(debug, (tx, ty), r, (255, 255, 0), 2)
            cv2.drawMarker(debug, (tx, ty), (255, 255, 0), cv2.MARKER_CROSS, r * 2, 1)
            cv2.putText(
                debug,
                f"id{tag.tag_id} {tag.tvec_z:.2f}m",
                (tx + r + 4, ty - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
            )

        # Image centre line
        center_x = (
            int(self._image_width / 2.0)
            if self._image_width > 0
            else debug.shape[1] // 2
        )
        cv2.line(debug, (center_x, 0), (center_x, debug.shape[0]), (255, 0, 0), 1)

        # Active source label
        cv2.putText(
            debug, source, (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2
        )
        msg = self._bridge.cv2_to_compressed_imgmsg(debug, dst_format="jpeg")
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self._debug_pub.publish(msg)

    # ---- apriltag detection ----

    def _detect_tag(self, bgr: np.ndarray):
        """Return the target TagObservation or None."""
        if self._detector is None or self._latest_camera_info is None:
            return None
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        observations = detect_payload_apriltags(
            gray,
            self._latest_camera_info,
            self._detector,
            self._detector_cache.backend,
            self.tag_size_m,
        )
        obs_map = {obs.tag_id: obs for obs in observations}
        if self.tag_id is not None:
            return obs_map.get(int(self.tag_id))
        if not obs_map:
            return None
        return max(obs_map.values(), key=lambda o: o.area)

    # ---- colour detection ----

    def _detect_color(
        self, bgr: np.ndarray
    ) -> Tuple[np.ndarray, Optional[float], Optional[float], int, int]:
        """Return (mask, cx, cy, area, bbox_height_px)."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower_hsv, self._upper_hsv)
        result = _color_blob_info(mask, float(self.color_min_detect_pixels))
        if result is None:
            return mask, None, None, 0, 0
        cx, cy, area, bh = result
        return mask, cx, cy, area, bh

    # ---- mode lifecycle ----

    def on_enter(self) -> None:
        self._done = False
        self._image_width = 0.0
        self._last_seen_time = None
        self._last_log_time = 0.0
        self._primed = False
        self._lost_frames_after_prime = 0
        self._last_phase = None
        self.vehicle.set_servo(180.0)
        self.log("PayloadDualApproachMode: started, servo set to 180")

    def on_update(self, time_delta: float) -> None:
        if self._done:
            return

        now = self._now()
        bgr = self._get_bgr()
        if bgr is None:
            if now - self._last_log_time >= 2.0:
                self._last_log_time = now
                self.log("PayloadDualApproachMode: waiting for camera frame")
            return

        self._image_width = float(bgr.shape[1])
        image_center = self._image_width / 2.0

        # --- detect colour (needed for stop check regardless of source) ---
        mask, color_cx, color_cy, color_area, color_height = self._detect_color(bgr)

        # Hoisted so the prime gate and the colour controller below share them.
        if color_cx is not None:
            lateral_error = color_cx - image_center
            centered = abs(lateral_error) <= self.color_centering_threshold_px
        else:
            lateral_error = 0.0
            centered = False

        # Arm only when close AND centered: in that state the controller drives
        # straight (color_drive_angular_scale = 0), so trajectory is
        # deterministic and any later contour loss is genuine overshoot.
        if not self._primed and color_height >= self.prime_height_px and centered:
            self._primed = True
            self.log(
                f"PayloadDualApproachMode: primed "
                f"(height={color_height}px >= {self.prime_height_px}px, centered)"
            )

        # Only tick the lost-frames counter when we were actually driving
        # straight — rotation during ALIGN can lose the contour without any
        # overshoot having occurred.
        if self._primed:
            if color_cx is None:
                if self._last_phase == "COLOR_DRIVE":
                    self._lost_frames_after_prime += 1
            else:
                self._lost_frames_after_prime = 0

        stop_reason: Optional[str] = None
        if self._primed:
            if color_height >= self.stop_height_px:
                stop_reason = f"stop_height hit (height={color_height}px)"
            elif self._lost_frames_after_prime >= self.post_prime_lost_frames:
                stop_reason = (
                    f"overshoot ({self._lost_frames_after_prime} "
                    f"lost frames after prime)"
                )

        if stop_reason is not None:
            self.vehicle.set_servo(0.0)
            self.vehicle.stop()
            self._done = True
            self.log(f"PayloadDualApproachMode: {stop_reason} — servo set to 0, done")
            self._publish_debug(
                bgr, mask, "STOP", None, color_cx, color_cy, color_height
            )
            return

        # --- try apriltag first (priority) ---
        tag = self._detect_tag(bgr)

        if tag is not None:
            self._last_seen_time = now
            distance = float(tag.tvec_z)
            lateral_error_px = float(tag.center_x - image_center)
            linear = min(self.tag_forward_gain * distance, self.tag_max_forward_speed)
            angular = (-self.tag_angular_gain * lateral_error_px) - (
                self.tag_yaw_gain * tag.yaw_error
            )
            self.vehicle.drive(linear, angular)
            self._last_phase = "TAG"
            self._publish_debug(bgr, mask, "TAG", tag, color_cx, color_cy, color_height)
            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self.log(
                    f"PayloadDualApproachMode: TAG dist={distance:.3f}m "
                    f"linear={linear:.2f} angular={angular:.2f}"
                )
            return

        # --- fallback to colour ---
        if color_cx is not None and color_area >= self.color_min_detect_pixels:
            self._last_seen_time = now
            angular = -self.color_angular_gain * lateral_error

            if not centered:
                self.vehicle.drive(0.0, angular)
                phase = "COLOR_ALIGN"
            else:
                drive_angular = angular * self.color_drive_angular_scale
                self.vehicle.drive(self.color_forward_speed, drive_angular)
                phase = "COLOR_DRIVE"

            self._last_phase = phase
            self._publish_debug(
                bgr, mask, phase, None, color_cx, color_cy, color_height
            )
            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self.log(
                    f"PayloadDualApproachMode: {phase} height={color_height}px "
                    f"cx={color_cx:.0f} err={lateral_error:.0f}"
                )
            return

        # --- nothing visible ---
        self._last_phase = "LOST"
        self._publish_debug(bgr, mask, "LOST", None, color_cx, color_cy, color_height)
        if (
            self._last_seen_time is not None
            and (now - self._last_seen_time) > self.lost_timeout_s
        ):
            self.vehicle.stop()
            self._last_seen_time = None
            if now - self._last_log_time >= 2.0:
                self._last_log_time = now
                self.log("PayloadDualApproachMode: both targets lost — stopped")

    def check_status(self) -> str:
        return "terminate" if self._done else "continue"

    def on_exit(self) -> None:
        self.vehicle.stop()
