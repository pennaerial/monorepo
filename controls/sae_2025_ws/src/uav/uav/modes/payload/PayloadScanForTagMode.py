from __future__ import annotations

import math
from typing import Optional

from rclpy.node import Node

from uav.vehicles.Payload import Payload
from uav.vision_nodes import PayloadAprilTagNode
from uav.vision_nodes.payload_perception_common import DEFAULT_TAG_FAMILY
from uav_interfaces.srv import PayloadAprilTagState

from ..Mode import Mode

_TWO_PI = 2.0 * math.pi


class PayloadScanForTagMode(Mode):
    """
    Spin in place looking for a specific AprilTag.

    Returns:
        "found"     — target tag is visible; stop and hand off to approach mode
        "not_found" — completed a full 360° without seeing the tag; caller
                      should reposition and try again
    """

    mission_target = "payload"
    required_vision_nodes = (PayloadAprilTagNode,)
    transition_labels = ("found", "not_found")

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        tag_id: int = 0,
        tag_size_m: float = 0.0508,
        tag_family: str = DEFAULT_TAG_FAMILY,
        spin_angular_speed: float = 0.4,
    ):
        super().__init__(node, vehicle)
        self.tag_id = int(tag_id)
        self.tag_size_m = float(tag_size_m)
        self.tag_family = str(tag_family) if tag_family else DEFAULT_TAG_FAMILY
        self.spin_angular_speed = float(spin_angular_speed)

    def on_enter(self) -> None:
        self._angle_swept = 0.0
        self._status = "continue"
        # Match spin direction to however PayloadDLZNavigateMode was travelling.
        # CCW travel → positive angular (spin left/CCW).
        # CW  travel → negative angular (spin right/CW).
        nav_direction = getattr(self.node, "dlz_navigate_direction", None)
        if nav_direction == "cw":
            self._signed_spin_speed = -self.spin_angular_speed
        else:
            self._signed_spin_speed = self.spin_angular_speed
        self.log(
            f"PayloadScanForTagMode: scanning for tag {self.tag_id} "
            f"(spin={self._signed_spin_speed:.2f} rad/s, nav_direction={nav_direction or 'unset→ccw'})"
        )

    def _request_state(self) -> Optional[PayloadAprilTagState.Response]:
        req = PayloadAprilTagState.Request()
        req.tag_size_m = self.tag_size_m
        req.tag_family = self.tag_family
        return self.send_request(PayloadAprilTagNode, req)

    def on_update(self, time_delta: float) -> None:
        if self._status != "continue":
            self.vehicle.stop()
            return

        response = self._request_state()
        if response is not None and response.has_image:
            tag_ids = list(response.all_tag_ids)
            if self.tag_id in tag_ids:
                self.vehicle.stop()
                self._status = "found"
                self.log(f"PayloadScanForTagMode: tag {self.tag_id} found → approach")
                return

        # Check if full rotation complete before spinning more
        if self._angle_swept >= _TWO_PI:
            self.vehicle.stop()
            self._status = "not_found"
            self.log(
                f"PayloadScanForTagMode: 360° complete, tag {self.tag_id} not found"
            )
            return

        self._angle_swept += abs(self._signed_spin_speed) * time_delta
        self.vehicle.drive(0.0, self._signed_spin_speed)

    def check_status(self) -> str:
        return self._status

    def on_exit(self) -> None:
        self.vehicle.stop()
