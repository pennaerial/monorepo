"""
PayloadColorStringApproachMode: drive the payload toward a coloured target
using HSV blob detection and a two-phase approach.

By default only **vertically elongated** blobs are tracked: the axis-aligned
bounding box must satisfy ``height >= vertical_blob_min_hw_ratio * width``.
The largest qualifying contour supplies centroid and area; other HSV
regions are ignored.  Set ``use_global_hsv_centroid`` to use the full mask
as before.

Set ``debug`` to true to publish a JPEG on
``{camera_namespace}/vision/color_string_approach/debug/compressed`` with
HSV mask contours drawn (green = tracked blob, orange = fails h/w, gray =
too small, yellow = other qualified candidates).

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

from uav.vehicles.Payload import Payload

from ..Mode import Mode


def _select_vertical_blob_from_contours(
    contours: list,
    min_hw_ratio: float,
    min_area: float,
) -> Optional[Tuple[float, int, int]]:
    """Pick largest qualifying contour. Returns ``(cx, area, contour_index)``."""
    best_idx = -1
    best_area = -1.0
    for i, cnt in enumerate(contours):
        a = cv2.contourArea(cnt)
        if a < min_area:
            continue
        _x, _y, bw, bh = cv2.boundingRect(cnt)
        if bw <= 0:
            continue
        if float(bh) < min_hw_ratio * float(bw):
            continue
        M = cv2.moments(cnt)
        if M["m00"] <= 0:
            continue
        if a > best_area:
            best_area = a
            best_idx = i

    if best_idx < 0:
        return None
    cnt = contours[best_idx]
    M = cv2.moments(cnt)
    cx = M["m10"] / M["m00"]
    return (cx, int(best_area), best_idx)


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
        use_global_hsv_centroid: bool = False,
        debug: bool = False,
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
        self.use_global_hsv_centroid = bool(use_global_hsv_centroid)
        self.debug = bool(debug)

        self._bridge = CvBridge()
        self._latest_image: Optional[Image | CompressedImage] = None
        self._debug_pub = None
        if self.debug:
            self._debug_pub = self.node.create_publisher(
                CompressedImage,
                f"{vehicle.camera_namespace}/vision/color_string_approach/debug/compressed",
                1,
            )

        if bool(compressed):
            self.node.create_subscription(
                CompressedImage,
                f"{vehicle.image_topic}/compressed",
                self._on_image,
                1,
            )
        else:
            self.node.create_subscription(
                Image, vehicle.image_topic, self._on_image, 1
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
        self.log(f"msg: {msg}")
        if msg is None:
            return None
        if isinstance(msg, CompressedImage):
            return self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _contour_class_vertical(
        self, cnt: np.ndarray
    ) -> Tuple[str, Tuple[int, int, int]]:
        """Return (reason, bgr_color) for drawing: small | ratio | ok."""
        a = cv2.contourArea(cnt)
        if a < self.min_detect_pixels:
            return "small", (80, 80, 80)
        _x, _y, bw, bh = cv2.boundingRect(cnt)
        if bw <= 0:
            return "small", (80, 80, 80)
        if float(bh) < self.vertical_blob_min_hw_ratio * float(bw):
            return "ratio", (0, 140, 255)
        M = cv2.moments(cnt)
        if M["m00"] <= 0:
            return "small", (80, 80, 80)
        return "ok", (0, 220, 220)

    def _publish_debug(
        self,
        bgr: np.ndarray,
        mask: np.ndarray,
        contours: list,
        sel_idx: Optional[int],
        cx: Optional[float],
        area: int,
        phase: str,
        lateral_error: Optional[float],
    ) -> None:
        if self._debug_pub is None:
            return
        h, w = bgr.shape[:2]
        icx = int(round(cx)) if cx is not None else None

        overlay = np.zeros_like(bgr)
        overlay[mask > 0] = (40, 180, 40)
        vis = cv2.addWeighted(bgr, 0.72, overlay, 0.28, 0)

        if self.use_global_hsv_centroid:
            for cnt in contours:
                cv2.drawContours(vis, [cnt], -1, (100, 100, 100), 1)
            if icx is not None:
                cv2.line(vis, (icx, 0), (icx, h - 1), (0, 255, 0), 2)
        else:
            for i, cnt in enumerate(contours):
                reason, col = self._contour_class_vertical(cnt)
                thick = 3 if i == sel_idx else 1
                if reason == "ok" and i != sel_idx:
                    col = (0, 255, 255)
                    thick = 2
                if i == sel_idx:
                    col = (0, 255, 0)
                    thick = 3
                cv2.drawContours(vis, [cnt], -1, col, thick)
                if i == sel_idx:
                    x, y, bw, bh = cv2.boundingRect(cnt)
                    cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 1)
            if icx is not None:
                cv2.line(vis, (icx, 0), (icx, h - 1), (255, 255, 255), 1)

        cx_frame = w // 2
        cv2.line(vis, (cx_frame, 0), (cx_frame, h - 1), (180, 180, 255), 1)

        err_s = f"err={lateral_error:+.0f}px" if lateral_error is not None else "err=—"
        cv2.putText(
            vis,
            f"{phase}  area={area}  {err_s}",
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            vis,
            "gray=small  orange=ratio  green=track  yellow=alt_ok",
            (8, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (200, 200, 200),
            1,
        )

        try:
            msg = self._bridge.cv2_to_compressed_imgmsg(vis, dst_format="jpeg")
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self._debug_pub.publish(msg)
        except Exception as exc:
            self.node.get_logger().warning(
                f"PayloadColorStringApproachMode: debug publish failed: {exc}"
            )

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
        if self.debug:
            self.log(
                "PayloadColorStringApproachMode: debug → "
                f"{self.vehicle.camera_namespace}/vision/color_string_approach/debug/compressed"
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
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        sel_idx: Optional[int] = None
        if self.use_global_hsv_centroid:
            M = cv2.moments(mask)
            area = int(M["m00"] / 255) if M["m00"] > 0 else 0
            cx = (M["m10"] / M["m00"]) if M["m00"] > 0 else None
        else:
            picked = _select_vertical_blob_from_contours(
                contours,
                self.vertical_blob_min_hw_ratio,
                float(self.min_detect_pixels),
            )
            if picked is None:
                cx = None
                area = 0
            else:
                cx, area, sel_idx = picked[0], picked[1], picked[2]

        phase = "LOST"
        lateral_error: Optional[float] = None
        if cx is not None and area >= self.min_detect_pixels:
            lateral_error = cx - self._image_width / 2.0
            phase = "ALIGN" if abs(lateral_error) > self.centering_threshold_px else "DRIVE"

        if self.debug:
            self._publish_debug(
                bgr,
                mask,
                contours,
                sel_idx,
                cx,
                area,
                phase,
                lateral_error,
            )

        if cx is None or area < self.min_detect_pixels:
            if self._last_seen_time is not None and (now - self._last_seen_time) > self.lost_timeout_s:
                self.vehicle.stop()
                self._last_seen_time = None
                if now - self._last_log_time >= 2.0:
                    self._last_log_time = now
                    self.log("PayloadColorStringApproachMode: target lost — stopped")
            return

        self._last_seen_time = now

        if area >= self.stop_area:
            self._stopping = True
            self.log(f"PayloadColorStringApproachMode: stop_area hit (area={area}), driving one more frame")

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
