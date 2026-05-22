"""
PayloadWaitForDriveOutMode — waits until the plane has landed on the DLZ.

Primary detection: HSV orange mask — does the camera see the orange DLZ?

When orange coverage >= orange_pixel_threshold continuously for wait_seconds,
the plane is confirmed landed → transition to reverse. Optical flow is NOT
checked while orange is visible (moving objects in the background cannot
disturb the timer).

Optical flow is only used when orange is absent: if the frame is still but
no orange is visible, the camera may be obstructed (plane belly, etc.) →
obstruction retreat. Optical flow state is reset whenever orange disappears
so readings are fresh after a landing.

CV runs inline (no separate vision node). The v4l2 + camera nodes still launch.

Transitions:
  "complete" -> whatever drive-out mode is configured in the mission YAML
"""

from __future__ import annotations

import math
from collections import deque
from enum import Enum
from typing import Optional, List

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from payload.payload import Payload

from vehicle_core.mode import Mode
from vehicle_core.runtime.plugin_loader import register_plugin


class DriveOutState(Enum):
    WAIT_UNREEL = 0
    OBSTRUCTION_RETREAT = 1
    REVERSING = 2
    TURNING = 3
    DONE = 4


@register_plugin(name="payload.PayloadWaitForDriveOutMode", base_cls=Mode)
class PayloadWaitForDriveOutMode(Mode):
    """
    Subscribe to the payload camera and detect when the plane has landed on the
    DLZ. Confirmation requires BOTH:
      1. Orange HSV coverage >= orange_pixel_threshold  (DLZ visible)
      2. Frame optical-flow magnitude < stillness_threshold  (payload not moving)

    Both conditions must hold continuously for wait_seconds before proceeding.
    """

    mission_target = "payload"
    required_vision_nodes = ()
    requires_camera = True
    transition_labels = ("complete",)

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        # DLZ color detection
        dlz_color_lower_hsv: List[int] = (5, 120, 120),
        dlz_color_upper_hsv: List[int] = (20, 255, 255),
        dlz_color_pixel_threshold: float = 0.04,
        # Stillness detection (optical flow)
        stillness_threshold: float = 2.0,
        stillness_window: int = 15,
        stillness_flow_crop_frac: float = 0.4,  # top fraction to discard (0.4 = use bottom 60%)
        # Obstruction retreat (still but no orange)
        obstruction_retreat_linear: float = -0.05,
        obstruction_retreat_speed: float = 1.5,
        obstruction_retreat_timeout_sec: float = 0.2,
        # Confirmation timers
        dlz_color_wait_seconds: float = 7.0,
        obstruction_frames: int = 20,
        # Drive-out motion params
        turn_angular: float = math.pi,
        turn_speed: float = 1.85,
        compressed_image: bool = False,
        debug: bool = False,
    ):
        super().__init__(node, vehicle)
        self.vehicle: Payload = vehicle

        self._dlz_lower = np.array(dlz_color_lower_hsv, dtype=np.uint8)
        self._dlz_upper = np.array(dlz_color_upper_hsv, dtype=np.uint8)
        self.dlz_color_pixel_threshold = float(dlz_color_pixel_threshold)

        self.stillness_threshold = float(stillness_threshold)
        self.stillness_window = int(stillness_window)
        self.stillness_flow_crop_frac = float(stillness_flow_crop_frac)

        self.obstruction_retreat_linear = float(obstruction_retreat_linear)
        self.obstruction_retreat_speed = float(obstruction_retreat_speed)
        self.obstruction_retreat_timeout_sec = float(obstruction_retreat_timeout_sec)

        self.dlz_color_wait_seconds = float(dlz_color_wait_seconds)
        self.obstruction_frames = int(obstruction_frames)
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
                vehicle.namespaced_path(
                    "vision/payload_wait_drive_out/debug/compressed"
                ),
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
        self._obstruction_count: int = 0
        self._dr_future = None
        self.state = DriveOutState.WAIT_UNREEL

        # Optical-flow state
        self._prev_gray: Optional[np.ndarray] = None
        self._flow_deltas: deque = deque(maxlen=self.stillness_window)

    def _on_image(self, msg: Image) -> None:
        self._latest_image = msg

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        self._latest_image = msg

    def _get_bgr_frame(self) -> Optional[np.ndarray]:
        msg = self._latest_image
        if msg is None:
            return None
        if isinstance(msg, CompressedImage):
            return self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _check_orange(self, bgr: np.ndarray) -> tuple[bool, float, np.ndarray]:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._dlz_lower, self._dlz_upper)
        coverage = float(np.count_nonzero(mask)) / mask.size
        return coverage >= self.dlz_color_pixel_threshold, coverage, mask

    def _check_stillness(self, bgr: np.ndarray) -> bool:
        gray_full = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        crop_row = int(gray_full.shape[0] * self.stillness_flow_crop_frac)
        gray = gray_full[crop_row:, :]

        if self._prev_gray is None:
            self._prev_gray = gray
            return False

        pts = cv2.goodFeaturesToTrack(
            self._prev_gray, maxCorners=80, qualityLevel=0.3, minDistance=7
        )
        if pts is None:
            # No trackable features — blank or obstructed, treat as still
            self._prev_gray = gray
            return True

        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(self._prev_gray, gray, pts, None)
        self._prev_gray = gray

        good = status.ravel() == 1
        if good.sum() == 0:
            return True

        motion = float(np.linalg.norm(next_pts[good] - pts[good], axis=1).mean())
        self._flow_deltas.append(motion)

        if len(self._flow_deltas) < self.stillness_window:
            return False
        return float(np.mean(self._flow_deltas)) < self.stillness_threshold

    def _publish_debug(
        self,
        bgr: np.ndarray,
        orange_found: bool,
        coverage: float,
        orange_mask: np.ndarray,
        is_still: Optional[bool],
    ) -> None:
        """Publish debug image. is_still=None when orange is found (optical flow not run)."""
        if self.debug_pub is None:
            return
        vis = bgr.copy()
        vis[orange_mask > 0] = (0, 140, 255)

        now = self._now()
        if orange_found:
            elapsed = (
                (now - self._clear_since) if self._clear_since is not None else 0.0
            )
            state_label = f"DLZ {elapsed:.1f}/{self.dlz_color_wait_seconds:.1f}s"
            label_color = (0, 255, 0)
        elif is_still:
            state_label = (
                f"OBSTRUCTED {self._obstruction_count}/{self.obstruction_frames}"
            )
            label_color = (0, 140, 255)
        else:
            state_label = "IN-FLIGHT"
            label_color = (128, 128, 128)

        cv2.putText(
            vis, state_label, (8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, label_color, 2
        )
        cv2.putText(
            vis,
            f"dlz={coverage:.2f} thresh={self.dlz_color_pixel_threshold:.2f}",
            (8, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )
        if is_still is not None:
            flow_mean = float(np.mean(self._flow_deltas)) if self._flow_deltas else 0.0
            cv2.putText(
                vis,
                f"flow={flow_mean:.2f} thresh={self.stillness_threshold:.2f}",
                (8, 82),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )

        ok, buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = buf.tobytes()
            self.debug_pub.publish(msg)

    def on_enter(self) -> None:
        self._done = False
        self._clear_since = None
        self._obstruction_count = 0
        self._dr_future = None
        self.state = DriveOutState.WAIT_UNREEL
        self._prev_gray = None
        self._flow_deltas.clear()
        self.log("waiting for DLZ (orange HSV)")
        self.vehicle.set_servo(0.0)

    def on_update(self, time_delta: float) -> None:
        if self._done:
            return

        if self.state == DriveOutState.WAIT_UNREEL:
            bgr = self._get_bgr_frame()
            if bgr is None:
                self.log("no camera image yet")
                return

            orange_found, coverage, orange_mask = self._check_orange(bgr)
            now = self._now()

            if orange_found:
                # Orange above threshold — timer runs, optical flow irrelevant
                self._obstruction_count = 0
                if self._clear_since is None:
                    self._clear_since = now
                    self.log(
                        f"DLZ in view — starting {self.dlz_color_wait_seconds}s timer"
                    )
                elapsed = now - self._clear_since
                self.log(
                    f"DLZ confirmed for {elapsed:.1f}/{self.dlz_color_wait_seconds:.1f}s"
                )
                self._publish_debug(bgr, True, coverage, orange_mask, None)
                if elapsed >= self.dlz_color_wait_seconds:
                    self.log("landing confirmed — moving to reverse")
                    self.state = DriveOutState.REVERSING
            else:
                # Orange absent — reset DLZ timer, reset optical flow so readings are fresh
                if self._clear_since is not None:
                    self.log(
                        "orange lost — resetting DLZ timer, resetting optical flow"
                    )
                    self._prev_gray = None
                    self._flow_deltas.clear()
                self._clear_since = None

                is_still = self._check_stillness(bgr)
                if is_still:
                    self._obstruction_count += 1
                    self.log(
                        f"still but no orange — obstruction {self._obstruction_count}/{self.obstruction_frames}"
                    )
                    if self._obstruction_count >= self.obstruction_frames:
                        self._obstruction_count = 0
                        self._prev_gray = None
                        self._flow_deltas.clear()
                        self.state = DriveOutState.OBSTRUCTION_RETREAT
                        self.log("obstruction confirmed — retreating to clear camera")
                else:
                    self._obstruction_count = 0
                self._publish_debug(bgr, False, coverage, orange_mask, is_still)

        elif self.state == DriveOutState.OBSTRUCTION_RETREAT:
            if self._dr_future is None:
                self._dr_future = self.vehicle.dead_reckon(
                    linear=self.obstruction_retreat_linear,
                    angular=0.0,
                    speed=self.obstruction_retreat_speed,
                    timeout_sec=self.obstruction_retreat_timeout_sec,
                )
            elif self._dr_future.done():
                result = self._dr_future.result()
                self.log(
                    f"obstruction retreat done success={result.success} — resuming DLZ watch"
                )
                self._dr_future = None
                # Reset optical-flow so motion from the retreat doesn't linger
                self._prev_gray = None
                self._flow_deltas.clear()
                self.state = DriveOutState.WAIT_UNREEL

        elif self.state == DriveOutState.REVERSING:
            self.vehicle.set_servo(180.0)
            if self._dr_future is None:
                self._dr_future = self.vehicle.dead_reckon(
                    linear=-0.05, angular=0.0, speed=self.turn_speed, timeout_sec=5.0
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
                    linear=0.0,
                    angular=self.turn_angular,
                    speed=self.turn_speed,
                    timeout_sec=5.0,
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
