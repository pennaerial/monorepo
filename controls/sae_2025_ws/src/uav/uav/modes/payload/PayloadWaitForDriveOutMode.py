"""
PayloadWaitForDriveOutMode — idles until the vision node detects a clear path ahead,
then signals termination so the mission transitions to the drive-out mode.

Transitions:
  "terminate" -> whatever drive-out mode is configured in the mission YAML
  "continue"  -> stay in this mode (path not yet clear)
"""

from __future__ import annotations

import math
from typing import Optional

from rclpy.node import Node

from uav.vehicles.Payload import Payload
from uav.vision_nodes.PayloadDriveOutNode import PayloadDriveOutNode
from uav_interfaces.srv import PayloadDriveOutState
from enum import Enum

from ..Mode import Mode


class DriveOutState(Enum):
    WAIT_UNREEL = 0
    REVERSING = 1
    SETTLING = 2  # brief pause after reverse before turning
    TURNING = 3
    DONE = 4


class PayloadWaitForDriveOutMode(Mode):
    """
    Poll PayloadDriveOutNode each update tick. Once the path ahead registers as
    clear for `confirm_frames` consecutive frames, set done and terminate.
    """

    mission_target = "payload"
    required_vision_nodes = (PayloadDriveOutNode,)

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        confirm_frames: int = 5,
        lower_hsv: list = (0, 0, 180),
        upper_hsv: list = (180, 20, 255),
    ):
        super().__init__(node, vehicle)
        self.vehicle: Payload = vehicle
        self.confirm_frames = int(confirm_frames)
        self._lower_hsv = [int(v) for v in lower_hsv]
        self._upper_hsv = [int(v) for v in upper_hsv]

        self._done = False
        self._clear_count = 0
        self._first_response_logged = False
        self._last_wait_log_time = 0.0
        self._dr_future = None
        self._reverse_done_time: Optional[float] = None

    def on_enter(self) -> None:
        self._done = False
        self._clear_count = 0
        self._first_response_logged = False
        self._last_wait_log_time = 0.0
        self._dr_future = None
        self._reverse_done_time = None
        self.state = DriveOutState.WAIT_UNREEL
        self.log("PayloadWaitForDriveOutMode: waiting for clear path")

    def _request_state(self) -> Optional[PayloadDriveOutState.Response]:
        req = PayloadDriveOutState.Request()
        req.lower_hsv = self._lower_hsv
        req.upper_hsv = self._upper_hsv
        return self.send_request(PayloadDriveOutNode, req)

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_update(self, time_delta: float) -> None:
        if self._done:
            self.log("done")
            return

        if self.state == DriveOutState.WAIT_UNREEL:
            self.vehicle.set_servo(0.0)
            response = self._request_state()
            if response is None:
                return

            if not self._first_response_logged:
                self._first_response_logged = True
                self.log("PayloadWaitForDriveOutMode: first vision response received")

            now = self._now()

            if not response.has_image:
                if now - self._last_wait_log_time >= 2.0:
                    self._last_wait_log_time = now
                    self.log("PayloadWaitForDriveOutMode: waiting for camera image")
                self._clear_count = 0
                return

            self.log(f"{str(response.can_drive_out)}")
            if response.can_drive_out:
                self._clear_count += 1
                self.log(
                    f"PayloadWaitForDriveOutMode: clear ({self._clear_count}/{self.confirm_frames}) "
                    f"ratio={response.clear_ratio:.2f}"
                )
                if self._clear_count >= self.confirm_frames:
                    self.state = DriveOutState.REVERSING
                    self._dr_future = self.vehicle.dead_reckon(
                        linear=-0.1, angular=0.0, speed=0.3
                    )
                    self.log("PayloadWaitForDriveOutMode: path clear — reversing 0.1 m")
            else:
                if self._clear_count > 0:
                    self.log(
                        f"PayloadWaitForDriveOutMode: path blocked (ratio={response.clear_ratio:.2f}) — resetting count"
                    )
                self._clear_count = 0
        elif self.state == DriveOutState.REVERSING:
            self.vehicle.set_servo(180.0)
            if self._dr_future is not None and self._dr_future.done():
                result = self._dr_future.result()
                self.log(
                    f"PayloadWaitForDriveOutMode: reverse done success={result.success} — settling"
                )
                self._dr_future = None
                self._reverse_done_time = self._now()
                self.state = DriveOutState.SETTLING
        elif self.state == DriveOutState.SETTLING:
            if self._now() - self._reverse_done_time >= 0.2:
                self.log(
                    "PayloadWaitForDriveOutMode: settle complete — starting 180 turn"
                )
                self._dr_future = self.vehicle.dead_reckon(
                    linear=0.0, angular=math.pi, speed=1.85
                )
                self.state = DriveOutState.TURNING
        elif self.state == DriveOutState.TURNING:
            if self._dr_future is not None and self._dr_future.done():
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
