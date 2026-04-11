from __future__ import annotations

from dataclasses import dataclass
import math
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
    tvec_x: float
    tvec_y: float
    tvec_z: float
    center_x: float
    center_y: float


class PayloadDriveToAprilTagMode(Mode):
    """
    Drives the payload toward a VTOL using processed AprilTag observations.

    When dock_rear_orthogonal is True, uses a two-layer architecture:
      Decision layer (rare): search -> orbit -> dock
      Control layer (continuous): small steering corrections only

    One initial 360° scan; orbit direction is computed once (shortest path to rear)
    and locked. No full spins after scan; brief tag loss = coast. Orbit->dock uses
    distance-dependent alignment: stricter when close (e.g. <=0.8m require 10°)
    to avoid false docking.
    """

    mission_target = "payload"
    required_vision_nodes = (PayloadAprilTagNode,)
    transition_labels = ()

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        tag_id: Optional[int] = None,
        stop_distance_m: float = 0.2,
        tag_size_m: float = 0.1,
        tag_family: str = DEFAULT_TAG_FAMILY,
        linear_gain: float = 0.3,
        angular_gain: float = 0.002,
        dock_rear_orthogonal: bool = False,
        dock_target_tag_id: int = 1,
        dock_orbit_radius_m: float = 2.0,
        dock_orbit_speed_mps: float = 0.15,
        dock_cam_to_rear_m: float = 0.103,
        dock_heading_rate_limit: float = 0.15,
        dock_search_spin_rps: float = 0.4,
        dock_align_angle_deg: float = 30.0,
        dock_align_hold_s: float = 0.4,
        dock_approach_front_dist_m: float = 2.0,
        dock_approach_front_min_m: float = 1.8,
        dock_approach_front_max_m: float = 2.2,
        dock_front_safe_standoff_m: float = 1.0,
        dock_approach_backup_margin_m: float = 0.2,
        dock_approach_bearing_gain: float = 0.3,
        dock_approach_pixel_scale: float = 0.5,
        dock_orbit_straight_duration_s: float = 7.0,
        dock_orbit_peek_duration_s: float = 1.5,
        dock_orbit_peek_max_when_lost_s: float = 15.0,
        dock_orbit_radial_k: float = 0.8,
        dock_orbit_yaw_k: float = 2.0,
        dock_orbit_peek_blend: float = 0.4,
        start_phase: Optional[str] = None,
    ):
        super().__init__(node, vehicle)
        self.start_phase = start_phase
        self.tag_id = tag_id
        self.stop_distance_m = float(stop_distance_m)
        self.tag_size_m = float(tag_size_m)
        self.tag_family = str(tag_family) if tag_family else DEFAULT_TAG_FAMILY
        self.linear_gain = float(linear_gain)
        self.angular_gain = float(angular_gain)
        self.dock_rear_orthogonal = bool(dock_rear_orthogonal)
        self.dock_target_tag_id = int(dock_target_tag_id)
        self.dock_orbit_radius_m = float(dock_orbit_radius_m)
        self.dock_orbit_speed_mps = float(dock_orbit_speed_mps)
        self.dock_cam_to_rear_m = float(dock_cam_to_rear_m)
        self.dock_heading_rate_limit = float(dock_heading_rate_limit)
        self.dock_search_spin_rps = float(dock_search_spin_rps)
        self.dock_align_angle_deg = float(dock_align_angle_deg)
        self.dock_align_hold_s = float(dock_align_hold_s)
        self.dock_approach_front_dist_m = float(dock_approach_front_dist_m)
        self.dock_approach_front_min_m = float(dock_approach_front_min_m)
        self.dock_approach_front_max_m = float(dock_approach_front_max_m)
        self.dock_front_safe_standoff_m = float(dock_front_safe_standoff_m)
        self.dock_approach_backup_margin_m = float(dock_approach_backup_margin_m)
        self.dock_approach_bearing_gain = float(dock_approach_bearing_gain)
        self.dock_approach_pixel_scale = float(dock_approach_pixel_scale)
        self.dock_orbit_straight_duration_s = float(dock_orbit_straight_duration_s)
        self.dock_orbit_peek_duration_s = float(dock_orbit_peek_duration_s)
        self.dock_orbit_peek_max_when_lost_s = float(dock_orbit_peek_max_when_lost_s)
        self.dock_orbit_radial_k = float(dock_orbit_radial_k)
        self.dock_orbit_yaw_k = float(dock_orbit_yaw_k)
        self.dock_orbit_peek_blend = float(dock_orbit_peek_blend)

        self.done = False
        self._image_width = 0.0
        self.node.get_logger().info(
            f"PayloadDriveToAprilTagMode: mission params check "
            f"dock_orbit_straight_duration_s={self.dock_orbit_straight_duration_s} "
            f"dock_orbit_radius_m={self.dock_orbit_radius_m} dock_orbit_yaw_k={self.dock_orbit_yaw_k} "
            f"dock_approach_front_dist_m={self.dock_approach_front_dist_m}"
        )

    def on_enter(self) -> None:
        self.done = False
        self._image_width = 0.0

        if getattr(self, "start_phase", None) == "dock":
            self._phase = "dock"
            self._orbit_dir = 1
            self._pose_vtol = None
            self._vtol_center = None
            self.log(
                "PayloadDriveToAprilTagMode: started in dock phase (handoff from color orbit)"
            )
        else:
            self._phase = "search"
            self._orbit_dir = None
            self._pose_vtol = None
            self._vtol_center = None

        self._last_cmd = (0.0, 0.0)
        self._last_tag_time = None
        self._last_log_time = 0.0
        self._last_closest_id = None
        self._search_start_yaw = None
        self._search_total_rot = 0.0
        self._search_poses = []
        self._search_started = False
        self._orbit_segment = None
        self._orbit_straight_start_time = None
        self._orbit_straight_heading = None
        self._orbit_peek_start_time = None
        self._orbit_turn_back_start_time = None
        self._dock_align_start = None
        self._first_response_logged = False
        self._last_wait_log_time = 0.0
        self._last_no_tag_log_time = 0.0
        self._last_drive_log_time = 0.0
        self.log("PayloadDriveToAprilTagMode: using PayloadAprilTagNode service")

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
            len(response.tag_ids),
            len(response.pose_x),
            len(response.pose_y),
            len(response.pose_yaw),
            len(response.tvec_x),
            len(response.tvec_y),
            len(response.tvec_z),
            len(response.center_x),
            len(response.center_y),
        ]
        observation_count = min(counts) if counts else 0

        observations: dict[int, TagObservation] = {}
        for index in range(observation_count):
            tag_id = int(response.tag_ids[index])
            observations[tag_id] = TagObservation(
                tag_id=tag_id,
                pose_x=float(response.pose_x[index]),
                pose_y=float(response.pose_y[index]),
                pose_yaw=float(response.pose_yaw[index]),
                tvec_x=float(response.tvec_x[index]),
                tvec_y=float(response.tvec_y[index]),
                tvec_z=float(response.tvec_z[index]),
                center_x=float(response.center_x[index]),
                center_y=float(response.center_y[index]),
            )
        return observations

    def _back_tag_view_angle_deg(
        self, observations: dict[int, TagObservation]
    ) -> Optional[float]:
        observation = observations.get(self.dock_target_tag_id)
        if observation is None:
            return None
        norm = math.sqrt(
            (observation.tvec_x * observation.tvec_x)
            + (observation.tvec_y * observation.tvec_y)
            + (observation.tvec_z * observation.tvec_z)
        )
        if norm < 1e-6:
            return None
        cos_angle = max(-1.0, min(1.0, observation.tvec_z / norm))
        return math.degrees(math.acos(cos_angle))

    def _publish_drive(self, linear: float, angular: float) -> None:
        self._last_cmd = (float(linear), float(angular))
        self.vehicle.drive(linear, angular)

    def _compute_straight_heading(self, x_vtol: float, y_vtol: float) -> float:
        bearing_to_vtol = math.atan2(-y_vtol, -x_vtol)
        heading_tangent = _wrap_angle(
            bearing_to_vtol + self._orbit_dir * (math.pi / 2.0)
        )
        radial_err = math.hypot(x_vtol, y_vtol) - self.dock_orbit_radius_m
        radial_k_eff = self.dock_orbit_radial_k * (2.0 if radial_err < 0 else 1.0)
        correction = math.atan(radial_k_eff * radial_err)
        return _wrap_angle(heading_tangent + self._orbit_dir * correction)

    def _choose_orbit_dir(self, x: float, y: float, context: str = "") -> int:
        theta = math.atan2(y, x)
        delta = _wrap_angle(-theta)
        direction = 1 if delta > 0 else -1
        self.log(
            f"DOCK | orbit_dir x={x:.3f} y={y:.3f} theta_deg={math.degrees(theta):.1f} "
            f"delta_deg={math.degrees(delta):.1f} dir={'CCW' if direction == 1 else 'CW'} [{context}]"
        )
        return direction

    def _dock_update(
        self, now: float, time_delta: float, observations: dict[int, TagObservation]
    ) -> None:
        tag_results = observations
        seen_ids = list(tag_results)
        back_visible = self.dock_target_tag_id in tag_results
        any_visible = bool(tag_results)

        if now - getattr(self, "_last_det_log_time", 0.0) >= 0.5:
            self._last_det_log_time = now
            tag_info = {tid: f"d={tag_results[tid].tvec_z:.3f}m" for tid in tag_results}
            self.log(
                f"DOCK | detections solved={list(tag_results.keys())} info={tag_info} phase={self._phase}"
            )

        if any_visible:
            if self._phase == "orbit":
                side_ids = [tag_id for tag_id in [2, 3] if tag_id in tag_results]
                back_ids = [1] if 1 in tag_results else []
                front_ids = [0] if 0 in tag_results else []
                candidates = (
                    side_ids
                    if side_ids
                    else (
                        back_ids if back_ids else (front_ids if front_ids else seen_ids)
                    )
                )
                closest_id = min(
                    candidates, key=lambda tag_id: tag_results[tag_id].tvec_z
                )
            else:
                closest_id = min(
                    seen_ids, key=lambda tag_id: tag_results[tag_id].tvec_z
                )

            closest = tag_results[closest_id]
            measurement = (closest.pose_x, closest.pose_y, closest.pose_yaw)
            if self._pose_vtol is None or closest_id != self._last_closest_id:
                self._pose_vtol = measurement
                if self._last_closest_id is None:
                    self.log(
                        f"DOCK | first pose tag={closest_id} "
                        f"x={measurement[0]:.3f} y={measurement[1]:.3f} yaw={math.degrees(measurement[2]):.1f}°"
                    )
            else:
                alpha = 0.35
                pose_x, pose_y, pose_yaw = self._pose_vtol
                self._pose_vtol = (
                    ((1 - alpha) * pose_x) + (alpha * measurement[0]),
                    ((1 - alpha) * pose_y) + (alpha * measurement[1]),
                    _wrap_angle(
                        pose_yaw + alpha * _wrap_angle(measurement[2] - pose_yaw)
                    ),
                )

            self._last_closest_id = closest_id
            self._last_tag_time = now

            measured_center = (-measurement[0], -measurement[1])
            if self._vtol_center is None:
                self._vtol_center = measured_center
            else:
                alpha = 0.02
                center_x, center_y = self._vtol_center
                self._vtol_center = (
                    center_x + alpha * (measured_center[0] - center_x),
                    center_y + alpha * (measured_center[1] - center_y),
                )

        if self._phase == "search":
            front = tag_results.get(0)
            only_front = bool(seen_ids) and set(seen_ids) == {0}
            front_dist = front.tvec_z if front is not None else None

            if (
                not self._search_started
                and self._pose_vtol is not None
                and only_front
                and front_dist is not None
            ):
                if front_dist > self.dock_approach_front_max_m:
                    self._phase = "approach_front"
                    self.log(
                        f"DOCK | search -> approach_front (only front tag, d={front_dist:.2f}m > {self.dock_approach_front_max_m}m)"
                    )
                    self._control_approach_front(now, tag_results, seen_ids)
                    return
                if front_dist < self.dock_approach_front_min_m:
                    self._phase = "approach_front"
                    self.log(
                        f"DOCK | search -> approach_front (only front tag, d={front_dist:.2f}m < {self.dock_approach_front_min_m}m, will back up)"
                    )
                    self._control_approach_front(now, tag_results, seen_ids)
                    return

            spin_w = self.dock_search_spin_rps * 2.0 * math.pi
            if not self._search_started:
                if self._pose_vtol is not None:
                    self._search_started = True
                    self._search_start_yaw = self._pose_vtol[2]
                    self._search_total_rot = 0.0
                    self._search_poses = []
                    self.log("DOCK | search: first tag seen, starting 360° scan")
                else:
                    self._publish_drive(0.0, spin_w)
                    return

            self._search_total_rot += abs(spin_w) * time_delta
            for tag_id, observation in tag_results.items():
                self._search_poses.append(
                    (
                        tag_id,
                        (observation.pose_x, observation.pose_y, observation.pose_yaw),
                    )
                )

            if self._search_total_rot < 2.0 * math.pi:
                view_angle = self._back_tag_view_angle_deg(tag_results)
                aligned_now = (
                    view_angle is not None and view_angle <= self.dock_align_angle_deg
                )
                if back_visible and aligned_now:
                    if self._dock_align_start is None:
                        self._dock_align_start = now
                    if (now - self._dock_align_start) >= self.dock_align_hold_s:
                        self._phase = "dock"
                        self._dock_sub = "align"
                        self._dock_align_start = None
                        self.log(
                            f"DOCK | search (360° scan) -> dock (back tag view_angle={view_angle:.1f}° <= {self.dock_align_angle_deg}° held)"
                        )
                        self._publish_drive(0.0, 0.0)
                        return
                else:
                    self._dock_align_start = None

                if now - self._last_log_time >= 2.0:
                    self._last_log_time = now
                    self.log(
                        f"DOCK | search: scanning {math.degrees(self._search_total_rot):.0f}°/360° "
                        f"tags_collected={len(self._search_poses)}"
                    )
                self._publish_drive(0.0, spin_w)
                return

            x_vtol, y_vtol, _ = self._pose_vtol
            self._orbit_dir = self._choose_orbit_dir(x_vtol, y_vtol, "search")
            self._vtol_center = (-x_vtol, -y_vtol)
            self._desired_yaw = _wrap_angle(math.atan2(-y_vtol, -x_vtol))
            self._phase = "orbit"
            self._orbit_segment = "straight"
            self._orbit_straight_start_time = now
            self._orbit_straight_heading = self._compute_straight_heading(
                x_vtol, y_vtol
            )
            self._orbit_peek_start_time = None
            self._orbit_turn_back_start_time = None
            self.log(
                f"DOCK | search -> orbit (360° done, {len(self._search_poses)} sightings) "
                f"pos=({x_vtol:.3f},{y_vtol:.3f})"
            )

        if self._phase == "approach_front":
            front = tag_results.get(0)
            if not any_visible or front is None:
                self._control_approach_front(now, tag_results, seen_ids)
                return
            if set(seen_ids) != {0}:
                self._transition_approach_front_to_orbit(now)
                return
            if front.tvec_z < self.dock_front_safe_standoff_m:
                self._transition_approach_front_to_orbit(now)
                return
            if (
                self.dock_approach_front_min_m
                <= front.tvec_z
                <= self.dock_approach_front_max_m
            ):
                self._transition_approach_front_to_orbit(now)
                return
            self._control_approach_front(now, tag_results, seen_ids)
            return

        if (
            self._phase == "dock"
            and getattr(self, "_dock_sub", None) not in ("spin_180", "back_up_5s")
            and not back_visible
            and any_visible
        ):
            self._phase = "orbit"
            self._dock_sub = None
            self._dock_align_start = None
            if self._vtol_center is not None and self._pose_vtol is not None:
                pose_x, pose_y, _ = self._pose_vtol
                self._orbit_segment = "straight"
                self._orbit_straight_start_time = now
                self._orbit_straight_heading = self._compute_straight_heading(
                    pose_x, pose_y
                )
                self._orbit_peek_start_time = None
                self._orbit_turn_back_start_time = None
            self.log(
                f"DOCK | dock -> orbit (back tag lost, others visible: {seen_ids})"
            )

        if self._phase == "orbit":
            self._control_orbit(now, time_delta, seen_ids, any_visible, tag_results)
        elif self._phase == "dock":
            self._control_dock(now, time_delta, tag_results, seen_ids)

    def _transition_approach_front_to_orbit(self, now: float) -> None:
        if self._pose_vtol is None:
            return

        pose_x, pose_y, _ = self._pose_vtol
        self._orbit_dir = self._choose_orbit_dir(pose_x, pose_y, "approach_front")
        self._vtol_center = (-pose_x, -pose_y)
        self._phase = "orbit"
        self._orbit_segment = "turn_back"
        self._orbit_turn_back_start_time = now
        self._orbit_turn_back_heading = self._compute_straight_heading(pose_x, pose_y)
        self._orbit_straight_start_time = None
        self._orbit_straight_heading = None
        self._orbit_peek_start_time = None
        self.log(
            f"DOCK | approach_front -> orbit (dir={'CCW' if self._orbit_dir == 1 else 'CW'}, align first)"
        )

    def _control_approach_front(
        self,
        now: float,
        tag_results: dict[int, TagObservation],
        seen_ids: list[int],
    ) -> None:
        front = tag_results.get(0)
        if front is None:
            self._publish_drive(*self._last_cmd)
            return

        distance = front.tvec_z
        bearing = math.atan2(front.tvec_x, front.tvec_z)
        img_w = self._image_width if self._image_width > 0.0 else 640.0
        err_x = front.center_x - (img_w / 2.0)
        angular = float(
            np.clip(
                -self.dock_approach_bearing_gain * bearing
                - self.angular_gain * self.dock_approach_pixel_scale * err_x,
                -0.4,
                0.4,
            )
        )

        if distance < self.dock_approach_front_min_m:
            dist_to_band = self.dock_approach_front_min_m - distance
            linear = float(np.clip(-self.linear_gain * dist_to_band, -0.15, -0.05))
            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self.log(
                    f"DOCK | approach_front BACKUP d={distance:.2f}m -> "
                    f"[{self.dock_approach_front_min_m:.1f},{self.dock_approach_front_max_m:.1f}]m "
                    f"v={linear:.3f} w={angular:.3f} tags={seen_ids}"
                )
            self._publish_drive(linear, angular)
            return

        if distance < self.dock_front_safe_standoff_m:
            self._publish_drive(0.0, angular)
            return

        dist_err = distance - self.dock_approach_front_max_m
        linear = (
            float(np.clip(self.linear_gain * max(0.0, dist_err), 0.05, 0.2))
            if dist_err > 0.0
            else 0.0
        )
        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self.log(
                f"DOCK | approach_front d={distance:.2f}m err={dist_err:.2f}m "
                f"v={linear:.3f} w={angular:.3f} tags={seen_ids}"
            )
        self._publish_drive(linear, angular)

    def _control_orbit(
        self,
        now: float,
        time_delta: float,
        seen_ids: list[int],
        any_visible: bool,
        tag_results: dict[int, TagObservation],
    ) -> None:
        if self._pose_vtol is None:
            self._publish_drive(0.0, 0.0)
            return

        pose_x, pose_y, pose_yaw = self._pose_vtol
        if any_visible:
            self._orbit_yaw_estimate = pose_yaw

        radius = math.hypot(pose_x, pose_y)
        theta_vtol = math.atan2(pose_y, pose_x)
        back_visible = self.dock_target_tag_id in tag_results

        if back_visible and any_visible:
            view_angle = self._back_tag_view_angle_deg(tag_results)
            aligned_now = (
                view_angle is not None and view_angle <= self.dock_align_angle_deg
            )
            if aligned_now:
                if self._dock_align_start is None:
                    self._dock_align_start = now
                if (now - self._dock_align_start) >= self.dock_align_hold_s:
                    self._phase = "dock"
                    self._dock_sub = "align"
                    self._dock_align_start = None
                    self.log(
                        f"DOCK | orbit -> dock (back tag view_angle={view_angle:.1f}° <= {self.dock_align_angle_deg}° held)"
                    )
                    self._publish_drive(0.0, 0.0)
                    return
            else:
                self._dock_align_start = None

        heading_tangent = _wrap_angle(theta_vtol + self._orbit_dir * (math.pi / 2.0))
        radial_err = radius - self.dock_orbit_radius_m
        radial_k_eff = self.dock_orbit_radial_k * (2.0 if radial_err < 0 else 1.0)
        _heading_cmd = _wrap_angle(
            heading_tangent - self._orbit_dir * math.atan(radial_k_eff * radial_err)
        )

        if self._orbit_segment is None:
            self._orbit_segment = "straight"
            self._orbit_straight_start_time = now
            self._orbit_straight_heading = self._compute_straight_heading(
                pose_x, pose_y
            )
            self._orbit_peek_start_time = None
            self._orbit_turn_back_start_time = None

        if not any_visible:
            segment = self._orbit_segment
            elapsed = now - (self._orbit_straight_start_time or now)
            if segment == "straight" and elapsed >= self.dock_orbit_straight_duration_s:
                self._orbit_segment = "peek_turn"
                self._orbit_peek_start_time = now
                self._orbit_peek_start_yaw = (
                    self._orbit_yaw_estimate
                    if self._orbit_yaw_estimate is not None
                    else pose_yaw
                )

            if segment == "peek_turn":
                peek_elapsed = now - (self._orbit_peek_start_time or now)
                if peek_elapsed >= self.dock_orbit_peek_max_when_lost_s:
                    self._orbit_segment = "straight"
                    self._orbit_straight_start_time = now
                    if now - self._last_log_time >= 0.5:
                        self._last_log_time = now
                        self.log("DOCK | orbit peek_turn -> straight (timeout, no tag)")
                else:
                    bearing_to_center = math.atan2(-pose_y, -pose_x)
                    yaw_eff = (
                        self._orbit_yaw_estimate
                        if self._orbit_yaw_estimate is not None
                        else pose_yaw
                    )
                    yaw_err_peek = _wrap_angle(bearing_to_center - yaw_eff)
                    w_peek = float(
                        np.clip(self.dock_orbit_yaw_k * yaw_err_peek, -0.6, 0.6)
                    )
                    self._publish_drive(0.0, w_peek)
                    self._orbit_yaw_estimate = _wrap_angle(
                        yaw_eff + (w_peek * time_delta)
                    )
                    return

            yaw_eff = (
                self._orbit_yaw_estimate
                if self._orbit_yaw_estimate is not None
                else pose_yaw
            )
            hold_heading = (
                self._orbit_straight_heading
                if self._orbit_straight_heading is not None
                else heading_tangent
            )
            yaw_err_hold = _wrap_angle(hold_heading - yaw_eff)
            linear = float(np.clip(self.dock_orbit_speed_mps, 0.05, 0.2))
            angular = float(np.clip(self.dock_orbit_yaw_k * yaw_err_hold, -0.4, 0.4))
            self._orbit_yaw_estimate = _wrap_angle(yaw_eff + (angular * time_delta))
            self._publish_drive(linear, angular)
            return

        if self._orbit_segment == "straight":
            elapsed = now - (self._orbit_straight_start_time or now)
            if elapsed >= self.dock_orbit_straight_duration_s:
                self._orbit_segment = "peek_turn"
                self._orbit_peek_start_time = now
                self._orbit_peek_start_yaw = pose_yaw
                if now - self._last_log_time >= 0.5:
                    self._last_log_time = now
                    self.log(
                        f"DOCK | orbit straight -> peek_turn (elapsed={elapsed:.1f}s)"
                    )

            if self._orbit_straight_heading is None:
                self._orbit_straight_heading = self._compute_straight_heading(
                    pose_x, pose_y
                )
            yaw_err = _wrap_angle(self._orbit_straight_heading - pose_yaw)
            linear = float(np.clip(self.dock_orbit_speed_mps, 0.05, 0.2))
            angular = float(np.clip(self.dock_orbit_yaw_k * yaw_err, -0.4, 0.4))
            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self.log(
                    f"DOCK | orbit straight r={radius:.3f} r_err={radial_err:.3f} "
                    f"h_lock={math.degrees(self._orbit_straight_heading):.1f}° v={linear:.3f} w={angular:.3f} tags={seen_ids}"
                )
            self._publish_drive(linear, angular)
            return

        if self._orbit_segment == "peek_turn":
            bearing_to_center = math.atan2(-pose_y, -pose_x)
            yaw_err_peek = _wrap_angle(bearing_to_center - pose_yaw)
            angular = float(np.clip(self.dock_orbit_yaw_k * yaw_err_peek, -0.6, 0.6))
            self._orbit_segment = "turn_back"
            self._orbit_turn_back_start_time = now
            if now - self._last_log_time >= 0.5:
                self._last_log_time = now
                self.log("DOCK | orbit peek_turn -> turn_back (tag visible)")
            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self.log(
                    f"DOCK | orbit peek r={radius:.3f} r_err={radial_err:.3f} w={angular:.3f} tags={seen_ids}"
                )
            self._publish_drive(0.0, angular)
            return

        if self._orbit_segment == "turn_back":
            if getattr(self, "_orbit_turn_back_heading", None) is None:
                self._orbit_turn_back_heading = self._compute_straight_heading(
                    pose_x, pose_y
                )
            heading = self._orbit_turn_back_heading
            yaw_err = _wrap_angle(heading - pose_yaw)
            angular = float(np.clip(self.dock_orbit_yaw_k * yaw_err, -0.6, 0.6))
            if abs(yaw_err) < 0.25:
                self._orbit_segment = "straight"
                self._orbit_straight_start_time = now
                self._orbit_straight_heading = self._orbit_turn_back_heading
                self._orbit_turn_back_heading = None
                if now - self._last_log_time >= 0.5:
                    self._last_log_time = now
                    self.log("DOCK | orbit turn_back -> straight (aligned)")
            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self.log(
                    f"DOCK | orbit turn_back r={radius:.3f} err={math.degrees(yaw_err):.1f}° w={angular:.3f} tags={seen_ids}"
                )
            self._publish_drive(0.05, angular)

    def _control_dock(
        self,
        now: float,
        time_delta: float,
        tag_results: dict[int, TagObservation],
        seen_ids: list[int],
    ) -> None:
        back = tag_results.get(self.dock_target_tag_id)
        if getattr(self, "_dock_sub", None) is None:
            self._dock_sub = "approach"

        sub = self._dock_sub
        if sub == "align":
            if back is None:
                self._publish_drive(*self._last_cmd)
                return

            bearing = math.atan2(back.tvec_x, back.tvec_z)
            img_w = self._image_width if self._image_width > 0.0 else 640.0
            err_x = back.center_x - (img_w / 2.0)
            aligned = abs(bearing) <= math.radians(5.0) and abs(err_x) <= 6.0
            if aligned:
                if self._dock_align_start is None:
                    self._dock_align_start = now
                if (now - self._dock_align_start) >= self.dock_align_hold_s:
                    self._dock_sub = "approach"
                    self._dock_align_start = None
                    self.log("DOCK | align -> approach (straight-on confirmed)")
            else:
                self._dock_align_start = None

            angular = float(
                np.clip((-1.2 * bearing) - (self.angular_gain * err_x), -0.6, 0.6)
            )
            self._publish_drive(0.0, angular)
            return

        if sub == "spin_180":
            spin_rate = 0.5
            total_rot = getattr(self, "_dock_spin_total_rot", 0.0) + (
                spin_rate * time_delta
            )
            self._dock_spin_total_rot = total_rot
            if total_rot >= (1.05 * math.pi):
                self._dock_sub = "back_up_5s"
                self._dock_back_up_5s_start = now
                self._publish_drive(0.0, 0.0)
                self.log("DOCK | dock spin_180 -> back_up_5s")
                return
            self._publish_drive(0.0, spin_rate)
            return

        if sub == "back_up_5s":
            start_time = getattr(self, "_dock_back_up_5s_start", now)
            if (now - start_time) >= 5.0:
                self.done = True
                self._publish_drive(0.0, 0.0)
                self.log(f"DOCK | complete (back_up_5s done, tags={seen_ids})")
                return
            self._publish_drive(-0.1, 0.0)
            return

        if back is None:
            self._publish_drive(*self._last_cmd)
            return

        distance = back.tvec_z
        if distance <= self.stop_distance_m:
            self._publish_drive(0.0, 0.0)
            self._dock_sub = "spin_180"
            self._dock_spin_total_rot = 0.0
            self.log(
                f"DOCK | dock approach -> spin_180 (at {distance:.3f}m, then back 5s)"
            )
            return

        img_w = self._image_width if self._image_width > 0.0 else 640.0
        err_x = back.center_x - (img_w / 2.0)
        bearing = math.atan2(back.tvec_x, back.tvec_z)
        view_angle_deg = self._back_tag_view_angle_deg(tag_results)
        approach_align_release_deg = self.dock_align_angle_deg + 2.0
        if view_angle_deg is not None and view_angle_deg > approach_align_release_deg:
            self._dock_sub = "align"
            self.log(
                f"DOCK | approach -> align (view_angle={view_angle_deg:.1f}° > {approach_align_release_deg:.1f}°)"
            )
            self._publish_drive(0.0, 0.0)
            return
        if abs(bearing) > math.radians(self.dock_align_angle_deg) or abs(err_x) > 25.0:
            self._dock_sub = "align"
            self.log(
                f"DOCK | approach -> align (bearing={math.degrees(bearing):.1f}° or off-center)"
            )
            self._publish_drive(0.0, 0.0)
            return

        angular = float(
            np.clip((-self.angular_gain * err_x) - (0.5 * bearing), -0.4, 0.4)
        )
        linear = float(
            np.clip(
                self.linear_gain * (distance - self.stop_distance_m),
                0.05,
                0.2,
            )
        )
        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self.log(
                f"DOCK | dock approach dist={distance:.3f}m tx={back.tvec_x:.3f}m "
                f"bearing={math.degrees(bearing):.1f}° v={linear:.3f} w={angular:.3f} tags={seen_ids}"
            )
        self._publish_drive(linear, angular)

    def on_exit(self) -> None:
        self.vehicle.stop()

    def on_update(self, time_delta: float) -> None:
        import time as _time

        now = _time.time()
        if self.done:
            return

        response = self._request_state()
        if response is None:
            return

        if not response.detector_available:
            self.log(
                "PayloadDriveToAprilTagMode: apriltag not installed; cannot detect tags."
            )
            return

        if not response.has_image or not response.has_camera_info:
            if now - getattr(self, "_last_wait_log_time", 0.0) >= 2.0:
                self._last_wait_log_time = now
                self.log(
                    f"PayloadDriveToAprilTagMode: waiting for payload camera "
                    f"(image={response.has_image}, camera_info={response.has_camera_info})"
                )
            return

        if not self._first_response_logged:
            self._first_response_logged = True
            self.log("PayloadDriveToAprilTagMode: first perception response received")

        observations = self._decode_observations(response)
        if self.dock_rear_orthogonal:
            self._dock_update(now, time_delta, observations)
            return

        target = None
        if observations:
            if self.tag_id is not None:
                target = observations.get(int(self.tag_id))
            else:
                target = next(iter(observations.values()))

        if target is None:
            if now - getattr(self, "_last_no_tag_log_time", 0.0) >= 2.0:
                self._last_no_tag_log_time = now
                if observations:
                    self.log(
                        f"PayloadDriveToAprilTagMode: tags in view {list(observations)} but no match, angular=0.2"
                    )
                else:
                    self.log("PayloadDriveToAprilTagMode: no tag in view, angular=0.2")
            self._publish_drive(0.0, 0.2)
            return

        distance = target.tvec_z
        if distance <= self.stop_distance_m:
            self._publish_drive(0.0, 0.0)
            self.done = True
            self.log(f"PayloadDriveToAprilTagMode: done, stopped at {distance:.3f}m")
            return

        image_width = self._image_width if self._image_width > 0.0 else 640.0
        err_x = target.center_x - (image_width / 2.0)
        linear = np.clip(
            self.linear_gain * (distance - self.stop_distance_m),
            0.0,
            0.5,
        )
        angular = np.clip(-self.angular_gain * err_x, -0.5, 0.5)
        if now - getattr(self, "_last_drive_log_time", 0.0) >= 1.0:
            self._last_drive_log_time = now
            self.log(
                f"PayloadDriveToAprilTagMode: tag={target.tag_id} dist={distance:.3f}m "
                f"linear={linear:.2f} angular={angular:.2f}"
            )
        self._publish_drive(float(linear), float(angular))

    def check_status(self) -> str:
        if self.done:
            return "terminate"
        return "continue"
