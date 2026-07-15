"""
PayloadRetreatMode: drive to the edge of the DLZ and turn 180° to face the center.

Two-phase behaviour:
  1. drive_to_edge — drive forward until white DLZ coverage in the lower image drops
     below edge_threshold, confirming the payload has reached the zone boundary.
  2. turn_180 — spin in place until π radians have been accumulated, leaving the
     payload facing back toward the center of the DLZ.

Designed for a white DLZ on a green/grass background. No AprilTag library required.
"""

import math
from typing import Optional, Tuple

import cv2
import numpy as np
from pydantic import BaseModel
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from payload_interfaces.msg import DriveCommand
from cv_bridge import CvBridge

from vehicle_common.mode import Mode
from vehicle_common.mode_loader import register_mode
from payload.payload import Payload
from uav.vision_nodes import PayloadAprilTagNode

# HSV range for white (high value, low saturation)
_WHITE_LOWER = np.array([0, 0, 200], dtype=np.uint8)
_WHITE_UPPER = np.array([180, 30, 255], dtype=np.uint8)


@register_mode(
    id="payload.PayloadRetreatMode",
    targets=[Payload],
    required_vision_nodes=[PayloadAprilTagNode],
)
class PayloadRetreatMode(Mode):
    """
    Drive the payload to the edge of the DLZ, then turn 180° to face the center.

    Phase 1 (drive_to_edge): move forward at forward_speed_mps while watching
    the lower half of the camera image for white coverage.  The edge is confirmed
    once white_ratio drops below edge_threshold for edge_stable_frames consecutive
    frames — meaning the white DLZ floor has given way to the surrounding ground.

    Phase 2 (turn_180): spin in place at turn_angular_speed rad/s (direction set
    by turn_direction: +1 = CCW, -1 = CW) until π radians are accumulated.

    Publishes DriveCommand to the payload's own namespaced command topic.
    """

    class Params(BaseModel):
        pass

    required_vision_nodes = (PayloadAprilTagNode,)
    transition_labels = ()

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        forward_speed_mps: float = 0.12,
        edge_threshold: float = 0.30,
        edge_stable_frames: int = 5,
        edge_min_pixels: int = 10,
        edge_invalid_max_frames: int = 30,
        turn_angular_speed: float = 0.5,
        turn_direction: int = 1,
        turn_angular: float = math.pi,
        turn_speed: float = 1.85,
        compressed_image: bool = False,
        camera_debug: bool = False,
    ):
        super().__init__(node, vehicle)
        self.forward_speed_mps = float(forward_speed_mps)
        self.edge_threshold = float(edge_threshold)
        self.edge_stable_frames = int(edge_stable_frames)
        self.edge_min_pixels = int(edge_min_pixels)
        self.edge_invalid_max_frames = int(edge_invalid_max_frames)
        self.turn_angular_speed = float(turn_angular_speed)
        self.turn_direction = 1 if int(turn_direction) >= 0 else -1
        self.turn_angular = float(turn_angular)
        self.turn_speed = float(turn_speed)
        self.compressed_image = bool(compressed_image)
        self.camera_debug = bool(camera_debug)

        self._bridge = CvBridge()

        self._image_sub = None
        self._info_sub = None
        self._drive_pub = None
        self._image: Optional[object] = None

        self._phase: str = "drive_to_edge"
        self._edge_stable_count: int = 0
        self._edge_invalid_count: int = 0
        self._turned_rad: float = 0.0
        self._done: bool = False

    # ------------------------------------------------------------------
    # Mode lifecycle
    # ------------------------------------------------------------------

    def on_enter(self) -> None:
        self._phase = "drive_to_edge"
        self._edge_stable_count = 0
        self._edge_invalid_count = 0
        self._turned_rad = 0.0
        self._turn_future = None
        self._done = False
        self._image = None

        cam_topic = self.vehicle.namespaced_path("camera")
        drive_topic = self.vehicle.namespaced_path("cmd_drive")

        if self.compressed_image:
            self._image_sub = self.node.create_subscription(
                CompressedImage, cam_topic, self._image_cb, 1
            )
        else:
            self._image_sub = self.node.create_subscription(
                Image, cam_topic, self._image_cb, 1
            )
        self._drive_pub = self.node.create_publisher(DriveCommand, drive_topic, 10)
        self.log(
            f"PayloadRetreatMode: subscribed to {cam_topic}, publishing to {drive_topic}"
        )

    def on_exit(self) -> None:
        self._publish_drive(0.0, 0.0)
        if self.camera_debug:
            cv2.destroyWindow("PayloadRetreatMode")
        self.node.destroy_subscription(self._image_sub)
        self.node.destroy_publisher(self._drive_pub)

    def on_update(self, time_delta: float) -> None:
        if self._done or self._image is None:
            return

        try:
            if self.compressed_image:
                buf = np.frombuffer(self._image.data, dtype=np.uint8)
                bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            else:
                bgr = self._bridge.imgmsg_to_cv2(self._image, desired_encoding="bgr8")
        except Exception as exc:
            self.node.get_logger().warn(
                f"PayloadRetreatMode: image decode failed: {exc}"
            )
            return

        if bgr is None:
            return

        # ---- Phase 1: drive forward until we reach the DLZ edge ----
        if self._phase == "drive_to_edge":
            white_ratio, white_count = self._edge_metrics(bgr)
            self.node.get_logger().info(
                f"white={white_ratio * 100:.1f}%  count={white_count}"
            )

            if white_count < self.edge_min_pixels:
                self._edge_invalid_count += 1
                if self._edge_invalid_count >= self.edge_invalid_max_frames:
                    self.log("PayloadRetreatMode: too many invalid frames — stopping")
                    self._publish_drive(0.0, 0.0)
                    return
                self._publish_drive(self.forward_speed_mps, 0.0)
                return

            self._edge_invalid_count = 0

            at_edge = white_ratio < self.edge_threshold
            if at_edge:
                self._edge_stable_count += 1
                if self._edge_stable_count >= self.edge_stable_frames:
                    self._publish_drive(0.0, 0.0)
                    self._phase = "turn_180"
                    self.log(
                        f"PayloadRetreatMode: edge confirmed (white={white_ratio * 100:.1f}%) — starting 180° turn"
                    )
                    if self.camera_debug:
                        self._draw_debug(bgr, white_ratio, white_count, at_edge=True)
                    return
            else:
                self._edge_stable_count = 0

            if self.camera_debug:
                self._draw_debug(bgr, white_ratio, white_count, at_edge=False)
            self._publish_drive(self.forward_speed_mps, 0.0)
            return

        # ---- Phase 2: turn 180° in place, dead reckoning ----
        if self._phase == "turn_180":
            if self._turn_future is None:
                angular = self.turn_angular * self.turn_direction
                self._turn_future = self.vehicle.dead_reckon(
                    0.0, angular, self.turn_speed, timeout_sec=5.0
                )
                self.log(
                    f"PayloadRetreatMode: sent dead_reckon for {math.degrees(self.turn_angular):.0f}° turn at {self.turn_speed} rad/s"
                )
                return
            if self._turn_future.done():
                self._done = True
                self.log("PayloadRetreatMode: 180° dead_reckon complete — done")
            return

    def check_status(self) -> str:
        return "terminate" if self._done else "continue"

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _image_cb(self, msg) -> None:
        self._image = msg

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _edge_metrics(self, bgr: np.ndarray) -> Tuple[float, int]:
        """
        Lower half of the image: compute white_ratio and white_count.

        white_ratio = white_pixels / total_lower_pixels.
        A low ratio (< edge_threshold) means the white DLZ floor has given way
        to the surrounding ground — the payload has reached the zone edge.
        """
        h, w = bgr.shape[:2]
        lower = bgr[h // 2 :, :]
        hsv = cv2.cvtColor(lower, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, _WHITE_LOWER, _WHITE_UPPER)
        white_count = int(np.count_nonzero(white_mask))
        total_pixels = lower.shape[0] * lower.shape[1]
        white_ratio = white_count / total_pixels if total_pixels > 0 else 0.0
        return white_ratio, white_count

    def _draw_debug(
        self,
        bgr: np.ndarray,
        white_ratio: float,
        white_count: int,
        at_edge: bool,
    ) -> None:
        vis = bgr.copy()
        h, w = vis.shape[:2]

        lower = vis[h // 2 :, :]
        hsv = cv2.cvtColor(lower, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, _WHITE_LOWER, _WHITE_UPPER)
        lower[white_mask > 0] = (255, 100, 0)
        vis[h // 2 :, :] = lower

        cv2.line(vis, (0, h // 2), (w, h // 2), (255, 255, 0), 1)

        edge_color = (0, 0, 255) if at_edge else (0, 255, 255)
        label = "AT EDGE" if at_edge else "driving"
        cv2.putText(
            vis,
            f"{label}  white={white_ratio * 100:.1f}%  stable={self._edge_stable_count}/{self.edge_stable_frames}",
            (8, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            edge_color,
            2,
        )
        cv2.putText(
            vis,
            f"phase={self._phase}  white={white_count}",
            (8, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )
        if self._phase == "turn_180":
            cv2.putText(
                vis,
                f"turned={math.degrees(self._turned_rad):.1f}/180°",
                (8, 78),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 165, 255),
                1,
            )

        cv2.imshow("PayloadRetreatMode", vis)
        cv2.waitKey(1)

    def _publish_drive(self, linear: float, angular: float) -> None:
        if self._drive_pub is None:
            return
        self._drive_pub.publish(
            DriveCommand(linear=float(linear), angular=float(angular))
        )
