"""
PayloadWaitForDriveOutMode — waits until the payload is fully unreeled from the plane.

While reeled in, the camera sees mostly darkness (obstructed by the plane body).
Once unreeled, the obstruction clears and the frame brightens. When the non-dark
pixel ratio stays above `clear_ratio` for `wait_seconds` continuously, the mode
transitions to reverse.

CV runs inline (no separate vision node). The v4l2 + camera nodes still launch.

Transitions:
  "complete" -> whatever drive-out mode is configured in the mission YAML
"""

from __future__ import annotations

import math
from enum import Enum
from typing import Optional

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from uav.vehicles.Payload import Payload

from ..Mode import Mode


class DriveOutState(Enum):
    WAIT_UNREEL = 0
    REVERSING = 1
    TURNING = 2
    DONE = 3


class PayloadWaitForDriveOutMode(Mode):
    """
    Subscribe to the payload camera and measure frame brightness. Once the
    non-dark pixel ratio exceeds `clear_ratio` continuously for `wait_seconds`,
    reverse briefly, turn, and terminate.
    """

    mission_target = "payload"
    required_vision_nodes = ()
    transition_labels = ("complete",)

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        dark_threshold: int = 30,
        clear_ratio: float = 0.3,
        wait_seconds: float = 2.0,
        turn_angular: float = math.pi,
        turn_speed: float = 1.85,
        compressed_image: bool = False,
        debug: bool = False,
    ):
        super().__init__(node, vehicle)
        self.vehicle: Payload = vehicle
        self.dark_threshold = int(dark_threshold)
        self.clear_ratio = float(clear_ratio)
        self.wait_seconds = float(wait_seconds)
        self.turn_angular = float(turn_angular)
        self.turn_speed = float(turn_speed)
        self.debug = bool(debug)

        self._bridge = CvBridge()
        self._latest_image: Optional[Image | CompressedImage] = None
        self._compressed = bool(compressed_image)

        self.debug_pub = None
        if self.debug:
            self.debug_pub = self.node.create_publisher(
                CompressedImage,
                f"{vehicle.camera_namespace}/vision/payload_wait_drive_out/debug/compressed",
                1,
            )

        if self._compressed:
            self.node.create_subscription(
                CompressedImage,
                f"{vehicle.image_topic}/compressed",
                self._on_compressed_image,
                1,
            )
        else:
            self.node.create_subscription(Image, vehicle.image_topic, self._on_image, 1)

        self._done = False
        self._clear_since: Optional[float] = None
        self._dr_future = None
        self.state = DriveOutState.WAIT_UNREEL

    def _on_image(self, msg: Image) -> None:
        self._latest_image = msg

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        self._latest_image = msg

    def _get_bgr_frame(self) -> Optional[np.ndarray]:
        msg = self._latest_image
        if msg is None:
            return None
        if isinstance(msg, CompressedImage):
            frame = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        else:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return frame

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _measure_and_publish(self, bgr: np.ndarray) -> float:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        non_dark_mask = gray > self.dark_threshold
        ratio = float(np.count_nonzero(non_dark_mask)) / gray.size

        if self.debug_pub is not None:
            vis = bgr.copy()
            vis[non_dark_mask] = (0, 200, 0)
            elapsed = (self._now() - self._clear_since) if self._clear_since is not None else 0.0
            cv2.putText(vis, f"non-dark={ratio:.2f} thresh={self.dark_threshold}",
                        (8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(vis, f"timer={elapsed:.1f}/{self.wait_seconds:.1f}s",
                        (8, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            ok, buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if ok:
                msg = CompressedImage()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = buf.tobytes()
                self.debug_pub.publish(msg)

        return ratio

    def on_enter(self) -> None:
        self._done = False
        self._clear_since = None
        self._dr_future = None
        self.state = DriveOutState.WAIT_UNREEL
        self.log("waiting for unreel (watching for dark→bright transition)")
        self.vehicle.set_servo(0.0)

    def on_update(self, time_delta: float) -> None:
        if self._done:
            return

        if self.state == DriveOutState.WAIT_UNREEL:
            bgr = self._get_bgr_frame()
            if bgr is None:
                self.log("no camera image yet")
                return

            ratio = self._measure_and_publish(bgr)
            now = self._now()

            if ratio >= self.clear_ratio:
                if self._clear_since is None:
                    self._clear_since = now
                    self.log(f"frame bright (ratio={ratio:.2f}) — starting {self.wait_seconds}s timer")
                elapsed = now - self._clear_since
                self.log(f"clear for {elapsed:.1f}/{self.wait_seconds:.1f}s")
                if elapsed >= self.wait_seconds:
                    self.log("unreel confirmed — moving to reverse")
                    self.state = DriveOutState.REVERSING
            else:
                if self._clear_since is not None:
                    self.log(f"went dark again (ratio={ratio:.2f}) — resetting timer")
                self._clear_since = None
        elif self.state == DriveOutState.REVERSING:
            self.vehicle.set_servo(180.0)
            if self._dr_future is None:
                self._dr_future = self.vehicle.dead_reckon(
                    linear=-0.05, angular=0.0, speed=self.turn_speed
                )
            elif self._dr_future.done():
                result = self._dr_future.result()
                self.log(
                    f"PayloadWaitForDriveOutMode: reverse done success={result.success} — settling"
                )
                self._dr_future = None
                self.state = DriveOutState.TURNING
        elif self.state == DriveOutState.TURNING:
            if self._dr_future is None:
                self._dr_future = self.vehicle.dead_reckon(
                    linear=0.0, angular=self.turn_angular, speed=self.turn_speed
                )
            elif self._dr_future.done():
                result = self._dr_future.result()
                self.log(
                    f"PayloadWaitForDriveOutMode: turn done success={result.success} — terminating"
                )
                self._dr_future = None
                self._done = True

    def check_status(self) -> str:
        return "complete" if self._done else "continue"

    def on_exit(self) -> None:
        pass
