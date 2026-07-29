from __future__ import annotations

import math
from typing import Optional, override

import cv2
import numpy as np
from cv_bridge import CvBridge
from pydantic import BaseModel
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from payload.payload import Payload
from uav.vision_nodes import PayloadAprilTagNode
from uav.vision_nodes.payload_perception_common import DEFAULT_TAG_FAMILY
from uav_interfaces.srv import PayloadAprilTagState

from vehicle_common.mode import Mode
from vehicle_common.mode_loader import register_mode

_TWO_PI = 2.0 * math.pi


class PayloadScanForTagParams(BaseModel):
    tag_id: int = 0
    tag_size_m: float = 0.0508
    tag_family: str = DEFAULT_TAG_FAMILY
    spin_angular_speed: float = 0.4
    loop: bool = False
    compressed_image: bool = False


@register_mode(
    id="payload.PayloadScanForTagMode",
    params_cls=PayloadScanForTagParams,
    targets=[Payload],
    required_vision_nodes=[PayloadAprilTagNode],
    transition_labels=["found", "not_found"],
    requires_camera=True,
)
class PayloadScanForTagMode(Mode):
    """
    Spin in place looking for a specific AprilTag.

    Returns:
        The transition label:

        - "found": target tag is visible; stop and hand off to approach mode.
        - "not_found": completed a full 360° without seeing the tag; caller
          should reposition and try again.
    """

    required_vision_nodes = (PayloadAprilTagNode,)
    transition_labels = ("found", "not_found")
    requires_camera = True

    @override
    def initialize(
        self, node: Node, vehicle: Payload, params: PayloadScanForTagParams
    ) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params
        self.tag_family = (
            str(params.tag_family) if params.tag_family else DEFAULT_TAG_FAMILY
        )
        self._bridge = CvBridge()

    def on_enter(self) -> None:
        self._angle_swept = 0.0
        self._status = "continue"
        self._latest_image = None
        # Match spin direction to however PayloadDLZNavigateMode was travelling.
        # CCW travel → positive angular (spin left/CCW).
        # CW  travel → negative angular (spin right/CW).
        nav_direction = getattr(self.node, "dlz_navigate_direction", None)
        if nav_direction == "cw":
            self._signed_spin_speed = -self.p.spin_angular_speed
        else:
            self._signed_spin_speed = self.p.spin_angular_speed

        self._image_sub = None
        self._annotated_pub = None
        if getattr(self.node, "vision_debug", False):
            cam_topic = self.vehicle.namespaced_path("camera")
            if self.p.compressed_image:
                self._image_sub = self.node.create_subscription(
                    CompressedImage, f"{cam_topic}/compressed", self._image_cb, 1
                )
            else:
                self._image_sub = self.node.create_subscription(
                    Image, cam_topic, self._image_cb, 1
                )

            annotated_topic = self.vehicle.namespaced_path("annotated_image/compressed")
            self._annotated_pub = self.node.create_publisher(
                CompressedImage, annotated_topic, 1
            )

        self.log(
            f"PayloadScanForTagMode: scanning for tag {self.p.tag_id} "
            f"(spin={self._signed_spin_speed:.2f} rad/s, nav_direction={nav_direction or 'unset→ccw'})"
        )

    def _image_cb(self, msg) -> None:
        self._latest_image = msg

    def _get_bgr(self) -> Optional[np.ndarray]:
        msg = self._latest_image
        if msg is None:
            return None
        if self.p.compressed_image:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _publish_annotated(self, bgr: np.ndarray, response) -> None:
        if self._annotated_pub is None:
            return
        debug = bgr.copy()
        h, w = debug.shape[:2]

        if response is not None and response.has_image:
            for i, tid in enumerate(response.all_tag_ids):
                cx = int(response.all_center_x[i])
                cy = int(response.all_center_y[i])
                tz = response.all_tvec_z[i]
                color = (0, 255, 0) if int(tid) == self.p.tag_id else (0, 255, 255)
                r = 12
                cv2.circle(debug, (cx, cy), r, color, 2)
                cv2.drawMarker(debug, (cx, cy), color, cv2.MARKER_CROSS, r * 2, 1)
                cv2.putText(
                    debug,
                    f"id{int(tid)} {tz:.2f}m",
                    (cx + r + 4, cy - 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

        sweep_deg = math.degrees(self._angle_swept)
        cv2.putText(
            debug,
            f"SCAN  sweep={sweep_deg:.0f} deg  target=id{self.p.tag_id}",
            (5, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        try:
            msg = self._bridge.cv2_to_compressed_imgmsg(debug, dst_format="jpeg")
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self._annotated_pub.publish(msg)
        except Exception:
            pass

    def _request_state(self) -> Optional[PayloadAprilTagState.Response]:
        req = PayloadAprilTagState.Request()
        req.tag_size_m = self.p.tag_size_m
        req.tag_family = self.tag_family
        return self.send_request(PayloadAprilTagNode, req)

    def on_update(self, time_delta: float) -> None:
        if self._status != "continue":
            self.vehicle.stop()
            return

        response = self._request_state()

        bgr = self._get_bgr()
        if bgr is not None:
            self._publish_annotated(bgr, response)

        if response is not None and response.has_image:
            tag_ids = list(response.all_tag_ids)
            if self.p.tag_id in tag_ids:
                self.vehicle.stop()
                self.log(f"PayloadScanForTagMode: tag {self.p.tag_id} found → approach")
                if self.p.loop:
                    self._angle_swept = 0.0
                else:
                    self._status = "found"
                return

        # Check if full rotation complete before spinning more
        if self._angle_swept >= _TWO_PI:
            self.vehicle.stop()
            self.log(
                f"PayloadScanForTagMode: 360° complete, tag {self.p.tag_id} not found"
            )
            if self.p.loop:
                self._angle_swept = 0.0
            else:
                self._status = "not_found"
            return

        self._angle_swept += abs(self._signed_spin_speed) * time_delta
        self.vehicle.drive(0.0, self._signed_spin_speed)

    def check_status(self) -> str:
        return self._status

    def on_exit(self) -> None:
        self.vehicle.stop()
        if self._image_sub is not None:
            self.node.destroy_subscription(self._image_sub)
            self._image_sub = None
        if self._annotated_pub is not None:
            self.node.destroy_publisher(self._annotated_pub)
            self._annotated_pub = None
