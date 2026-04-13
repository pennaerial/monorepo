"""
PayloadColorStringApproachMode: drive the payload toward a coloured target
using HSV blob detection and a two-phase approach.

Phase 1 (ALIGN): rotate in place until the blob centroid is within
``centering_threshold_px`` of the image centre.

Phase 2 (DRIVE): move forward with a reduced angular correction to stay
on target.  If the centroid drifts outside the threshold the mode falls
back to Phase 1.

Terminates when the blob area exceeds ``stop_area`` pixels.
"""

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from uav.vehicles.Payload import Payload

from ..Mode import Mode


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
        lower_hsv: list[int] = (20, 100, 100),
        upper_hsv: list[int] = (35, 255, 255),
        forward_speed: float = 0.08,
        angular_gain: float = 0.003,
        centering_threshold_px: int = 15,
        drive_angular_scale: float = 0.3,
        min_detect_pixels: int = 50,
        stop_area: int = 5000,
        lost_timeout_s: float = 3.0,
        compressed: bool = False,
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
        if msg is None:
            return None
        if isinstance(msg, CompressedImage):
            return self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_enter(self) -> None:
        self._done = False
        self._stopping = False
        self._aligned = False
        self._image_width = 0.0
        self._last_seen_time = None
        self._last_log_time = 0.0
        self.log("PayloadColorStringApproachMode: started — looking for colour target")

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
        M = cv2.moments(mask)
        area = int(M["m00"] / 255) if M["m00"] > 0 else 0

        if area < self.min_detect_pixels:
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

        cx = M["m10"] / M["m00"]
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
