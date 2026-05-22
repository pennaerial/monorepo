"""
PayloadColorStringApproachMode: drive the payload toward a coloured target
using HSV blob detection and a two-phase approach.

By default only **vertically elongated** blobs are tracked: the axis-aligned
bounding box must satisfy ``height >= vertical_blob_min_hw_ratio * width``.
The largest qualifying contour supplies centroid and area; other HSV
regions are ignored.  Set ``use_global_hsv_centroid`` to use the full mask
as before.
Phase 1 (ALIGN): rotate in place until the blob centroid is within
``centering_threshold_px`` of the image centre.

Phase 2 (DRIVE): move forward with a reduced angular correction to stay
on target.  If the centroid drifts outside the threshold the mode falls
back to Phase 1.

Terminates when the blob area exceeds ``stop_area`` pixels.
"""

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from payload.payload import Payload

from vehicle_core.mode import Mode
from vehicle_core.runtime.plugin_loader import register_plugin


def _vertical_blob_centroid_area(
    mask: np.ndarray,
    min_hw_ratio: float,
    min_area: float,
    top_frac: float = 0.3,
) -> Optional[Tuple[float, float, int]]:
    """Largest external contour with bbox height >= min_hw_ratio * width.

    ``cx`` is computed from only the top ``top_frac`` of the bounding box so
    that steering targets the far end of the string rather than its
    perspective-distorted base.  Area is computed from the full contour.

    Returns ``(cx, cy, area)`` in pixel units, or ``None`` if no contour qualifies.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_area: float = -1.0
    best_cx: float = 0.0
    best_cy: float = 0.0
    best_area_i: int = 0

    for cnt in contours:
        a = cv2.contourArea(cnt)
        if a < min_area:
            continue
        bx, by, bw, bh = cv2.boundingRect(cnt)
        if bw <= 0:
            continue
        if float(bh) < min_hw_ratio * float(bw):
            continue

        top_cutoff = by + max(1, int(bh * top_frac))
        top_mask = np.zeros_like(mask)
        top_mask[by:top_cutoff, bx : bx + bw] = mask[by:top_cutoff, bx : bx + bw]
        M_top = cv2.moments(top_mask)
        if M_top["m00"] <= 0:
            M_full = cv2.moments(cnt)
            if M_full["m00"] <= 0:
                continue
            cx = M_full["m10"] / M_full["m00"]
            cy = M_full["m01"] / M_full["m00"]
        else:
            cx = M_top["m10"] / M_top["m00"]
            cy = M_top["m01"] / M_top["m00"]

        if a > best_area:
            best_area = a
            best_cx = cx
            best_cy = cy
            best_area_i = int(a)

    if best_area < 0:
        return None
    return (best_cx, best_cy, best_area_i)


@register_plugin(name="payload.PayloadColorStringApproachMode", base_cls=Mode)
class PayloadColorStringApproachMode(Mode):
    """Drive toward a coloured target using HSV blob detection."""

    mission_target = "payload"
    required_vision_nodes = ()
    requires_camera = True
    transition_labels = ()

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        lower_hsv: list[int] = (0, 0, 30),
        upper_hsv: list[int] = (179, 40, 120),
        forward_speed: float = 0.08,
        angular_gain: float = 0.003,
        centering_threshold_px: int = 15,
        drive_angular_scale: float = 0.3,
        min_detect_pixels: int = 50,
        stop_area: int = 5000,
        lost_timeout_s: float = 3.0,
        compressed: bool = False,
        vertical_blob_min_hw_ratio: float = 2.0,
        top_centroid_frac: float = 0.3,
        use_global_hsv_centroid: bool = False,
    ):
        super().__init__(node, vehicle)
        self.vehicle: Payload = vehicle

        self._lower_hsv = np.array(lower_hsv, dtype=np.uint8)
        self._upper_hsv = np.array(upper_hsv, dtype=np.uint8)
        self.forward_speed = float(forward_speed)
        self.angular_gain = float(angular_gain)
        self.centering_threshold_px = int(centering_threshold_px)
        self.drive_angular_scale = float(drive_angular_scale)
        self.min_detect_pixels = int(min_detect_pixels)
        self.stop_area = int(stop_area)
        self.lost_timeout_s = float(lost_timeout_s)
        self.vertical_blob_min_hw_ratio = float(vertical_blob_min_hw_ratio)
        self.top_centroid_frac = float(top_centroid_frac)
        self.use_global_hsv_centroid = bool(use_global_hsv_centroid)

        self._bridge = CvBridge()
        self._latest_image: Optional[Image | CompressedImage] = None

        if bool(compressed):
            self.node.create_subscription(
                CompressedImage,
                f"{vehicle.image_topic}/compressed",
                self._on_image,
                1,
            )
        else:
            self.node.create_subscription(Image, vehicle.image_topic, self._on_image, 1)

        self._debug_pub = None
        if getattr(self.node, "vision_debug", False):
            self._debug_pub = self.node.create_publisher(
                Image, f"{vehicle.camera_namespace}/color_string_debug", 1
            )

        self._done = False
        self._stopping = False
        self._aligned = False
        self._image_width = 0.0
        self._last_seen_time: Optional[float] = None
        self._last_log_time = 0.0

    def _on_image(self, msg) -> None:
        self._latest_image = msg

    def _get_bgr(self) -> Optional[np.ndarray]:
        msg = self._latest_image
        if msg is None:
            return None
        if isinstance(msg, CompressedImage):
            return self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _publish_debug(
        self,
        bgr: np.ndarray,
        mask: np.ndarray,
        cx: Optional[float],
        cy: Optional[float],
    ) -> None:
        if self._debug_pub is None or self._debug_pub.get_subscription_count() == 0:
            return
        debug = bgr.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, contours, -1, (0, 255, 0), 1)
        if cx is not None and cy is not None:
            cv2.circle(debug, (int(cx), int(cy)), 5, (0, 0, 255), -1)
        center_x = (
            int(self._image_width / 2.0)
            if self._image_width > 0
            else debug.shape[1] // 2
        )
        cv2.line(debug, (center_x, 0), (center_x, debug.shape[0]), (255, 0, 0), 1)
        self._debug_pub.publish(self._bridge.cv2_to_imgmsg(debug, encoding="bgr8"))

    def on_enter(self) -> None:
        self._done = False
        self._stopping = False
        self._aligned = False
        self._image_width = 0.0
        self._last_seen_time = None
        self._last_log_time = 0.0
        if self.use_global_hsv_centroid:
            self.log("PayloadColorStringApproachMode: started — global HSV centroid")
        else:
            self.log(
                "PayloadColorStringApproachMode: started — vertical blob "
                f"(min h/w≥{self.vertical_blob_min_hw_ratio})"
            )

    def on_update(self, time_delta: float) -> None:
        if self._done:
            return

        if self._stopping:
            self.vehicle.stop()
            self._done = True
            self.log("PayloadColorStringApproachMode: extra frame done — stopping")
            return

        now = self._now()
        bgr = self._get_bgr()
        if bgr is None:
            if now - self._last_log_time >= 2.0:
                self._last_log_time = now
                self.log("PayloadColorStringApproachMode: waiting for camera frame")
            return

        self._image_width = float(bgr.shape[1])
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower_hsv, self._upper_hsv)

        cy: Optional[float] = None
        if self.use_global_hsv_centroid:
            M = cv2.moments(mask)
            area = int(M["m00"] / 255) if M["m00"] > 0 else 0
            cx: Optional[float] = (M["m10"] / M["m00"]) if M["m00"] > 0 else None
            if M["m00"] > 0:
                cy = M["m01"] / M["m00"]
        else:
            blob = _vertical_blob_centroid_area(
                mask,
                self.vertical_blob_min_hw_ratio,
                float(self.min_detect_pixels),
                self.top_centroid_frac,
            )
            if blob is None:
                cx = None
                area = 0
            else:
                cx, cy, area = blob

        self._publish_debug(bgr, mask, cx, cy)

        if cx is None or area < self.min_detect_pixels:
            if (
                self._last_seen_time is not None
                and (now - self._last_seen_time) > self.lost_timeout_s
            ):
                self.vehicle.stop()
                self._last_seen_time = None
                if now - self._last_log_time >= 2.0:
                    self._last_log_time = now
                    self.log("PayloadColorStringApproachMode: target lost — stopped")
            return

        self._last_seen_time = now

        if area >= self.stop_area:
            self._stopping = True
            self.log(
                f"PayloadColorStringApproachMode: stop_area hit (area={area}), driving one more frame"
            )

        image_center = self._image_width / 2.0
        lateral_error = cx - image_center
        angular = -self.angular_gain * lateral_error

        centered = abs(lateral_error) <= self.centering_threshold_px

        if not centered:
            # Phase 1: rotate in place to centre the target
            self._aligned = False
            self.vehicle.drive(0.0, angular)
            phase = "ALIGN"
        else:
            # Phase 2: drive forward with reduced angular correction
            self._aligned = True
            drive_angular = angular * self.drive_angular_scale
            self.vehicle.drive(self.forward_speed, drive_angular)
            phase = "DRIVE"

        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self.log(
                f"PayloadColorStringApproachMode: {phase} area={area} cx={cx:.0f} "
                f"err={lateral_error:.0f} angular={angular:.3f}"
            )

    def check_status(self) -> str:
        return "terminate" if self._done else "continue"

    def on_exit(self) -> None:
        self.vehicle.stop()
