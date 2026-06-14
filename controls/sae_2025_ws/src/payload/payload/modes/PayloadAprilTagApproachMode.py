from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from cv_bridge import CvBridge
import cv2

from payload.payload import Payload

from uav.vision_nodes.payload_perception_common import (
    DEFAULT_TAG_FAMILY,
    AprilTagDetectorCache,
    detect_payload_apriltags,
)

from vehicle_core.mode import Mode
from vehicle_core.runtime.plugin_loader import register_plugin


@dataclass(frozen=True)
class TagObservation:
    tag_id: int
    tvec_x: float
    tvec_y: float
    tvec_z: float
    center_x: float
    center_y: float
    yaw_error: float
    area: float


@register_plugin(name="payload.PayloadAprilTagApproachMode", base_cls=Mode)
class PayloadAprilTagApproachMode(Mode):
    """Drive the payload toward one AprilTag and terminate once close enough."""

    required_vision_nodes = ()
    requires_camera = True
    transition_labels = ("done", "tag_lost")

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        tag_id: Optional[int] = None,
        tag_size_m: float = 0.0508,
        tag_family: str = DEFAULT_TAG_FAMILY,
        max_forward_speed: float = 0.2,
        forward_gain: float = 0.5,
        angular_gain: float = 0.003,
        yaw_gain: float = 0.0,
        stop_distance_m: float = 0.2,
        tag_lost_coast_s: float = 0.5,
        compressed: bool = False,
        completion_state: str = "done",
    ):
        super().__init__(node, vehicle)
        self._completion_state = completion_state
        self.tag_id = None if tag_id is None else int(tag_id)
        self.tag_size_m = float(tag_size_m)
        self.tag_family = str(tag_family) if tag_family else DEFAULT_TAG_FAMILY
        self.max_forward_speed = float(max_forward_speed)
        self.forward_gain = float(forward_gain)
        self.angular_gain = float(angular_gain)
        self.yaw_gain = float(yaw_gain)
        self.stop_distance_m = float(stop_distance_m)
        self.tag_lost_coast_s = float(tag_lost_coast_s)

        self._bridge = CvBridge()
        self._latest_image: Optional[Image | CompressedImage] = None
        self._latest_camera_info: Optional[CameraInfo] = None
        self._detector_cache = AprilTagDetectorCache()
        self._detector = self._detector_cache.get(self.tag_family)

        if bool(compressed):
            self.node.create_subscription(
                CompressedImage,
                f"{vehicle.image_topic}/compressed",
                self._on_compressed_image,
                1,
            )
        else:
            self.node.create_subscription(Image, vehicle.image_topic, self._on_image, 1)

        self.node.create_subscription(
            CameraInfo, vehicle.camera_info_topic, self._on_camera_info, 1
        )

        self._done = False
        self._tag_lost = False
        self._image_width = 0.0
        self._last_tag_time: Optional[float] = None
        self._first_response_logged = False
        self._last_wait_log_time = 0.0
        self._last_no_tag_log_time = 0.0
        self._last_drive_log_time = 0.0

    def _on_image(self, msg: Image) -> None:
        self._latest_image = msg

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        self._latest_image = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._latest_camera_info is None:
            self._latest_camera_info = msg

    def _get_bgr_frame(self) -> Optional[object]:
        msg = self._latest_image
        if msg is None:
            return None
        if isinstance(msg, CompressedImage):
            return self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def on_enter(self) -> None:
        self._done = False
        self._tag_lost = False
        self._image_width = 0.0
        self._last_tag_time = None
        self._first_response_logged = False
        self._last_wait_log_time = 0.0
        self._last_no_tag_log_time = 0.0
        self._last_drive_log_time = 0.0
        self.vehicle.set_servo(180.0)
        self.log(
            "PayloadAprilTagApproachMode: servo set to 180, detecting apriltags inline"
        )

    def _detect_observations(self) -> Optional[dict[int, TagObservation]]:
        bgr = self._get_bgr_frame()
        if bgr is None:
            return None
        if self._latest_camera_info is None:
            return None
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        self._image_width = float(bgr.shape[1])
        raw_obs = detect_payload_apriltags(
            gray,
            self._latest_camera_info,
            self._detector,
            self._detector_cache.backend,
            self.tag_size_m,
        )
        return {
            obs.tag_id: TagObservation(
                tag_id=obs.tag_id,
                tvec_x=obs.tvec_x,
                tvec_y=obs.tvec_y,
                tvec_z=obs.tvec_z,
                center_x=obs.center_x,
                center_y=obs.center_y,
                yaw_error=obs.yaw_error,
                area=obs.area,
            )
            for obs in raw_obs
        }

    def _publish_drive(self, linear: float, angular: float) -> None:
        self.vehicle.drive(float(linear), float(angular))

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _handle_tag_timeout(self, now: float) -> bool:
        if self._last_tag_time is None:
            return False
        if now - self._last_tag_time <= self.tag_lost_coast_s:
            return False
        self._publish_drive(0.0, 0.0)
        self._tag_lost = True
        self._last_tag_time = None
        self.log("PayloadAprilTagApproachMode: tag lost; transitioning")
        return True

    def _select_target(
        self, observations: dict[int, TagObservation]
    ) -> Optional[TagObservation]:
        if self.tag_id is not None:
            return observations.get(int(self.tag_id))
        if not observations:
            return None
        return max(observations.values(), key=lambda obs: obs.area)

    def on_update(self, time_delta: float) -> None:
        if self._done:
            return

        now = self._now()

        if self._detector is None:
            self.log(
                "PayloadAprilTagApproachMode: apriltag not installed; cannot detect tags."
            )
            return

        if self._latest_image is None or self._latest_camera_info is None:
            if now - self._last_wait_log_time >= 2.0:
                self._last_wait_log_time = now
                self.log(
                    "PayloadAprilTagApproachMode: waiting for payload camera "
                    f"(image={self._latest_image is not None}, "
                    f"camera_info={self._latest_camera_info is not None})"
                )
            self._handle_tag_timeout(now)
            return

        if not self._first_response_logged:
            self._first_response_logged = True
            self.log("PayloadAprilTagApproachMode: first camera frame received")

        observations = self._detect_observations()
        if observations is None:
            self._handle_tag_timeout(now)
            return

        target = self._select_target(observations)

        if target is None:
            if self._handle_tag_timeout(now):
                return

            if now - self._last_no_tag_log_time >= 2.0:
                self._last_no_tag_log_time = now
                if observations:
                    self.log(
                        "PayloadAprilTagApproachMode: tags in view "
                        f"{list(observations)} but no match"
                    )
                else:
                    self.log("PayloadAprilTagApproachMode: no tag in view")
            return

        self._last_tag_time = now

        distance = float(target.tvec_z)
        if distance <= self.stop_distance_m:
            self.vehicle.set_servo(0.0)
            self.vehicle.stop()
            self._done = True
            self.log(
                f"PayloadAprilTagApproachMode: servo set to 0, stopped at {distance:.3f}m"
            )
            return

        image_width = self._image_width if self._image_width > 0.0 else 640.0
        lateral_error_px = float(target.center_x - (image_width / 2.0))
        linear = min(self.forward_gain * distance, self.max_forward_speed)
        angular = (-self.angular_gain * lateral_error_px) - (
            self.yaw_gain * target.yaw_error
        )

        if now - self._last_drive_log_time >= 1.0:
            self._last_drive_log_time = now
            self.log(
                f"PayloadAprilTagApproachMode: tag={target.tag_id} dist={distance:.3f}m "
                f"linear={linear:.2f} angular={angular:.2f}"
            )

        self._publish_drive(linear, angular)

    def check_status(self) -> str:
        if self._done:
            return self._completion_state
        if self._tag_lost:
            return "tag_lost"
        return "continue"

    def on_exit(self) -> None:
        self.vehicle.stop()
