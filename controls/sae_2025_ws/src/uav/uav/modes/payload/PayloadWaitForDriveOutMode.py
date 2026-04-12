"""
PayloadWaitForDriveOutMode — idles until the camera detects a clear path ahead,
then signals termination so the mission transitions to the drive-out mode.

CV runs inline (no separate vision node service). The v4l2 camera node still
launches as part of the vehicle stack.

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
from uav.vision_nodes.payload_perception_common import detect_payload_unreeled

from ..Mode import Mode


class DriveOutState(Enum):
    WAIT_UNREEL = 0
    REVERSING = 1
    TURNING = 2
    DONE = 3


class PayloadWaitForDriveOutMode(Mode):
    """
    Subscribe to the payload camera topic and run detect_payload_unreeled each
    update tick.  Once the path registers as clear for `confirm_frames`
    consecutive frames, reverse briefly, turn, and terminate.
    """

    mission_target = "payload"
    required_vision_nodes = ()
    transition_labels = ("complete",)

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        confirm_frames: int = 70,
        clear_threshold: float = 0.3,
        lower_hsv: list[int] = [0, 0, 180],
        upper_hsv: list[int] = [180, 20, 255],
        turn_angular: float = math.pi,
        turn_speed: float = 1.85,
        compressed_image: bool = False,
        debug: bool = False,
    ):
        super().__init__(node, vehicle)
        self.vehicle: Payload = vehicle
        self.confirm_frames = int(confirm_frames)
        self.clear_threshold = float(clear_threshold)
        self._lower_hsv = tuple(int(v) for v in lower_hsv)
        self._upper_hsv = tuple(int(v) for v in upper_hsv)
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
            self.log(f"COMPRESSED, subscribing to {vehicle.image_topic}/compressed")
        else:
            self.node.create_subscription(Image, vehicle.image_topic, self._on_image, 1)

        self.log(f"{vehicle.image_topic}")
        self.log(f"{vehicle.image_topic}")
        self.log(f"{vehicle.image_topic}")
        self.log(f"{vehicle.image_topic}")

        self.done = False
        self.clear_count = 0
        self.ast_wait_log_time = 0.0
        self.dr_future = None
        self.reverse_done_time: Optional[float] = None

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

    def on_enter(self) -> None:
        self._done = False
        self._clear_count = 0
        self._first_frame_logged = False
        self._last_wait_log_time = 0.0
        self._dr_future = None
        self.state = DriveOutState.WAIT_UNREEL
        self.log("PayloadWaitForDriveOutMode: waiting for clear path")
        self.vehicle.set_servo(0.0)

    def on_update(self, time_delta: float) -> None:
        if self._done:
            return

        if self.state == DriveOutState.WAIT_UNREEL:

            bgr = self._get_bgr_frame()
            if bgr is None:
                self.log("DriveOutMode: No Image received")
                return

            _, clear_ratio, _, debug_frame = detect_payload_unreeled(
                bgr, self._lower_hsv, self._upper_hsv, debug=self.debug)

            if self.debug_pub is not None and debug_frame is not None:
                ok, buf = cv2.imencode(
                    ".jpg", debug_frame, [cv2.IMWRITE_JPEG_QUALITY, 60]
                )
                if ok:
                    # self.log("PUBLISHING DEBUG")
                    dbg_msg = CompressedImage()
                    dbg_msg.header.stamp = self.node.get_clock().now().to_msg()
                    dbg_msg.format = "jpeg"
                    dbg_msg.data = buf.tobytes()
                    self.debug_pub.publish(dbg_msg)
            can_drive_out = clear_ratio >= self.clear_threshold

            if can_drive_out:
                self._clear_count += 1
                self.log(
                    f"PayloadWaitForDriveOutMode: clear ({self._clear_count}/{self.confirm_frames}) "
                    f"ratio={clear_ratio:.2f}"
                )
                if self._clear_count >= self.confirm_frames:
                    self.state = DriveOutState.REVERSING
                    self.log("PayloadWaitForDriveOutMode: path clear — reversing 0.1 m")
            else:
                if self._clear_count > 0:
                    self.log(
                        f"PayloadWaitForDriveOutMode: path blocked (ratio={clear_ratio:.2f}) — resetting count"
                    )
                self._clear_count = 0
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
