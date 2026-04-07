from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from rclpy.node import Node

from uav.vehicles.Payload import Payload
from uav.vision_nodes import PayloadAprilTagNode
from uav.vision_nodes.payload_perception_common import DEFAULT_TAG_FAMILY
from uav_interfaces.srv import PayloadAprilTagState

from ..Mode import Mode


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


class PayloadAprilTagApproachMode(Mode):
    """Drive the payload toward one AprilTag and terminate once close enough."""

    mission_target = "payload"
    required_vision_nodes = (PayloadAprilTagNode,)

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
    ):
        super().__init__(node, vehicle)
        self.vehicle: Payload = vehicle
        self.tag_id = None if tag_id is None else int(tag_id)
        self.tag_size_m = float(tag_size_m)
        self.tag_family = str(tag_family) if tag_family else DEFAULT_TAG_FAMILY
        self.max_forward_speed = float(max_forward_speed)
        self.forward_gain = float(forward_gain)
        self.angular_gain = float(angular_gain)
        self.yaw_gain = float(yaw_gain)
        self.stop_distance_m = float(stop_distance_m)
        self.tag_lost_coast_s = float(tag_lost_coast_s)

        self._done = False
        self._image_width = 0.0
        self._last_tag_time: Optional[float] = None
        self._first_response_logged = False
        self._last_wait_log_time = 0.0
        self._last_no_tag_log_time = 0.0
        self._last_drive_log_time = 0.0

    def on_enter(self) -> None:
        self._done = False
        self._image_width = 0.0
        self._last_tag_time = None
        self._first_response_logged = False
        self._last_wait_log_time = 0.0
        self._last_no_tag_log_time = 0.0
        self._last_drive_log_time = 0.0
        self.vehicle.set_servo(180.0)
        self.log(
            "PayloadAprilTagApproachMode: servo set to 180, using PayloadAprilTagNode service"
        )

    def _request_state(self) -> Optional[PayloadAprilTagState.Response]:
        request = PayloadAprilTagState.Request()
        request.tag_size_m = float(self.tag_size_m)
        request.tag_family = self.tag_family
        return self.send_request(PayloadAprilTagNode, request)

    def _decode_observations(
        self, response: PayloadAprilTagState.Response
    ) -> dict[int, TagObservation]:
        self._image_width = float(response.image_width)
        counts = [
            len(response.all_tag_ids),
            len(response.all_tvec_x),
            len(response.all_tvec_y),
            len(response.all_tvec_z),
            len(response.all_center_x),
            len(response.all_center_y),
            len(response.all_yaw_error),
            len(response.all_tag_area),
        ]
        observation_count = min(counts) if counts else 0

        observations: dict[int, TagObservation] = {}
        for index in range(observation_count):
            tag_id = int(response.all_tag_ids[index])
            observations[tag_id] = TagObservation(
                tag_id=tag_id,
                tvec_x=float(response.all_tvec_x[index]),
                tvec_y=float(response.all_tvec_y[index]),
                tvec_z=float(response.all_tvec_z[index]),
                center_x=float(response.all_center_x[index]),
                center_y=float(response.all_center_y[index]),
                yaw_error=float(response.all_yaw_error[index]),
                area=float(response.all_tag_area[index]),
            )
        return observations

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
        self._last_tag_time = None
        self.log("PayloadAprilTagApproachMode: tag lost; stopping")
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
        response = self._request_state()
        if response is None:
            self._handle_tag_timeout(now)
            return

        if not response.detector_available:
            self.log(
                "PayloadAprilTagApproachMode: apriltag not installed; cannot detect tags."
            )
            return

        if not response.has_image or not response.has_camera_info:
            if now - self._last_wait_log_time >= 2.0:
                self._last_wait_log_time = now
                self.log(
                    "PayloadAprilTagApproachMode: waiting for payload camera "
                    f"(image={response.has_image}, camera_info={response.has_camera_info})"
                )
            self._handle_tag_timeout(now)
            return

        if not self._first_response_logged:
            self._first_response_logged = True
            self.log("PayloadAprilTagApproachMode: first perception response received")

        observations = self._decode_observations(response)
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
        return "terminate" if self._done else "continue"

    def on_exit(self) -> None:
        self.vehicle.stop()
