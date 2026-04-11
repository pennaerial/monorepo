from __future__ import annotations

import math
import time as _time
from dataclasses import dataclass
from typing import Optional

import numpy as np
from rclpy.node import Node

from uav.vehicles.Payload import Payload
from uav.vision_nodes import PayloadAprilTagNode
from uav.vision_nodes.payload_perception_common import DEFAULT_TAG_FAMILY
from uav_interfaces.srv import PayloadAprilTagState

from ..Mode import Mode


def _wrap_angle(rad: float) -> float:
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class TagObservation:
    tag_id: int
    pose_x: float
    pose_y: float
    pose_yaw: float
    tvec_z: float


class PayloadNavigateAroundPlaneMode(Mode):
    """
    Navigate payload from the wrong side of the VTOL to the approach side.

    Scan: spin slowly to acquire an initial VTOL-frame pose from any visible tag.
    Early exit: if the approach tag is already visible during scan, hand off immediately.
    Navigate: drive through two hardcoded VTOL-frame waypoints (side → front),
    continuously correcting using AprilTag pose from whichever tag is visible.
    Done: once the front waypoint is reached and the approach tag is visible,
    signal the "approach" transition to hand off to PayloadAprilTagApproachMode.
    """

    mission_target = "payload"
    required_vision_nodes = (PayloadAprilTagNode,)
    transition_labels = ("approach",)

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        orbit_radius_m: float = 1.5,
        approach_tag_id: int = 0,
        nav_speed_mps: float = 0.15,
        nav_angular_gain: float = 2.0,
        max_angular_rps: float = 0.6,
        waypoint_reach_dist_m: float = 0.4,
        scan_spin_rps: float = 0.3,
        scan_timeout_s: float = 8.0,
        tag_lost_coast_s: float = 1.0,
        tag_size_m: float = 0.1,
        tag_family: str = DEFAULT_TAG_FAMILY,
    ):
        super().__init__(node, vehicle)
        self.orbit_radius_m = float(orbit_radius_m)
        self.approach_tag_id = int(approach_tag_id)
        self.nav_speed_mps = float(nav_speed_mps)
        self.nav_angular_gain = float(nav_angular_gain)
        self.max_angular_rps = float(max_angular_rps)
        self.waypoint_reach_dist_m = float(waypoint_reach_dist_m)
        self.scan_spin_rps = float(scan_spin_rps)
        self.scan_timeout_s = float(scan_timeout_s)
        self.tag_lost_coast_s = float(tag_lost_coast_s)
        self.tag_size_m = float(tag_size_m)
        self.tag_family = str(tag_family) or DEFAULT_TAG_FAMILY

    def on_enter(self) -> None:
        self._phase = "scan"
        self._scan_start = _time.time()
        self._waypoints: list[tuple[float, float]] = []
        self._wp_index = 0
        self._done = False
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_tag_time: Optional[float] = None
        self._first_response_logged = False
        self._last_log_time = 0.0
        self.log("PayloadNavigateAroundPlaneMode: starting scan phase")

    def _request_state(self) -> Optional[PayloadAprilTagState.Response]:
        request = PayloadAprilTagState.Request()
        request.tag_size_m = self.tag_size_m
        request.tag_family = self.tag_family
        return self.send_request(PayloadAprilTagNode, request)

    def _decode_observations(
        self, response: PayloadAprilTagState.Response
    ) -> dict[int, TagObservation]:
        counts = [
            len(response.tag_ids),
            len(response.pose_x),
            len(response.pose_y),
            len(response.pose_yaw),
            len(response.tvec_z),
        ]
        n = min(counts) if counts else 0
        observations: dict[int, TagObservation] = {}
        for i in range(n):
            tag_id = int(response.tag_ids[i])
            observations[tag_id] = TagObservation(
                tag_id=tag_id,
                pose_x=float(response.pose_x[i]),
                pose_y=float(response.pose_y[i]),
                pose_yaw=float(response.pose_yaw[i]),
                tvec_z=float(response.tvec_z[i]),
            )
        return observations

    def _best_observation(
        self, observations: dict[int, TagObservation]
    ) -> Optional[TagObservation]:
        if not observations:
            return None
        return min(observations.values(), key=lambda o: o.tvec_z)

    def _generate_waypoints(
        self, pose_x: float, pose_y: float
    ) -> list[tuple[float, float]]:
        r = self.orbit_radius_m
        side_y = r if pose_y >= 0 else -r
        return [
            (0.0, side_y),  # cross mid-fuselage at wing clearance
            (-r, 0.0),  # arrive in front of plane
        ]

    def _publish_drive(self, linear: float, angular: float) -> None:
        self._last_linear = linear
        self._last_angular = angular
        self.vehicle.drive(linear, angular)

    def on_update(self, time_delta: float) -> None:
        if self._done:
            return

        response = self._request_state()
        if response is None:
            return

        if not response.detector_available:
            self.log("PayloadNavigateAroundPlaneMode: apriltag not installed.")
            return

        if not response.has_image or not response.has_camera_info:
            return

        if not self._first_response_logged:
            self._first_response_logged = True
            self.log(
                "PayloadNavigateAroundPlaneMode: first perception response received"
            )

        observations = self._decode_observations(response)
        now = _time.time()

        if self._phase == "scan":
            self._run_scan(observations, now)
        elif self._phase == "navigate":
            self._run_navigate(observations, now)

    def _run_scan(self, observations: dict[int, TagObservation], now: float) -> None:
        # Early exit: approach tag already visible, skip navigation entirely
        if self.approach_tag_id in observations:
            obs = observations[self.approach_tag_id]
            self.log(
                f"PayloadNavigateAroundPlaneMode: SCAN -> DONE "
                f"(approach tag {self.approach_tag_id} already visible at "
                f"pose=({obs.pose_x:.2f}, {obs.pose_y:.2f}))"
            )
            self._publish_drive(0.0, 0.0)
            self._done = True
            return

        # Timeout: no pose at all, give up and signal handoff anyway
        elapsed = now - self._scan_start
        if elapsed >= self.scan_timeout_s:
            self.log(
                "PayloadNavigateAroundPlaneMode: SCAN timeout, no pose acquired. "
                "Signalling approach handoff."
            )
            self._publish_drive(0.0, 0.0)
            self._done = True
            return

        # Got a valid pose from some other tag → generate waypoints and navigate
        best = self._best_observation(observations)
        if best is not None:
            self._waypoints = self._generate_waypoints(best.pose_x, best.pose_y)
            self._wp_index = 0
            self.log(
                f"PayloadNavigateAroundPlaneMode: SCAN -> NAVIGATE "
                f"via tag {best.tag_id} pose=({best.pose_x:.2f}, {best.pose_y:.2f}) "
                f"waypoints={[(round(x, 2), round(y, 2)) for x, y in self._waypoints]}"
            )
            self._phase = "navigate"
            self._last_tag_time = now
            self._run_navigate(observations, now)
            return

        # Still scanning — spin in place
        spin_w = self.scan_spin_rps * 2.0 * math.pi
        if now - self._last_log_time >= 2.0:
            self._last_log_time = now
            self.log(
                f"PayloadNavigateAroundPlaneMode: SCAN spinning "
                f"({elapsed:.1f}/{self.scan_timeout_s:.1f}s)"
            )
        self._publish_drive(0.0, spin_w)

    def _run_navigate(
        self, observations: dict[int, TagObservation], now: float
    ) -> None:
        best = self._best_observation(observations)

        if best is None:
            # Tag lost — coast briefly, then stop
            if (
                self._last_tag_time is not None
                and (now - self._last_tag_time) < self.tag_lost_coast_s
            ):
                self._publish_drive(self._last_linear, self._last_angular)
            else:
                if now - self._last_log_time >= 2.0:
                    self._last_log_time = now
                    self.log(
                        "PayloadNavigateAroundPlaneMode: NAVIGATE tag lost, stopping"
                    )
                self._publish_drive(0.0, 0.0)
            return

        self._last_tag_time = now
        pose_x, pose_y, pose_yaw = best.pose_x, best.pose_y, best.pose_yaw

        # All waypoints done — wait at final position for approach tag
        if self._wp_index >= len(self._waypoints):
            if self.approach_tag_id in observations:
                self.log(
                    f"PayloadNavigateAroundPlaneMode: NAVIGATE -> DONE "
                    f"(front reached, approach tag {self.approach_tag_id} visible)"
                )
                self._publish_drive(0.0, 0.0)
                self._done = True
            else:
                if now - self._last_log_time >= 2.0:
                    self._last_log_time = now
                    self.log(
                        f"PayloadNavigateAroundPlaneMode: NAVIGATE at final wp, "
                        f"waiting for tag {self.approach_tag_id}"
                    )
                self._publish_drive(0.0, 0.0)
            return

        wp_x, wp_y = self._waypoints[self._wp_index]
        dx = wp_x - pose_x
        dy = wp_y - pose_y
        dist = math.hypot(dx, dy)

        if dist <= self.waypoint_reach_dist_m:
            self._wp_index += 1
            self.log(
                f"PayloadNavigateAroundPlaneMode: NAVIGATE reached wp {self._wp_index - 1}, "
                f"advancing to wp {self._wp_index}"
            )
            self._run_navigate(observations, now)
            return

        desired_heading = math.atan2(dy, dx)
        heading_error = _wrap_angle(desired_heading - pose_yaw)
        linear = self.nav_speed_mps
        angular = float(
            np.clip(
                self.nav_angular_gain * heading_error,
                -self.max_angular_rps,
                self.max_angular_rps,
            )
        )

        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self.log(
                f"PayloadNavigateAroundPlaneMode: NAVIGATE "
                f"wp={self._wp_index}/{len(self._waypoints)} "
                f"dist={dist:.2f}m pose=({pose_x:.2f},{pose_y:.2f}) "
                f"heading_err={math.degrees(heading_error):.1f}°"
            )

        self._publish_drive(linear, angular)

    def check_status(self) -> str:
        return "approach" if self._done else "continue"

    def on_exit(self) -> None:
        self.vehicle.stop()
