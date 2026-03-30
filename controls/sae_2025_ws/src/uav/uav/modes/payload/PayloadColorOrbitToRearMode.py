from __future__ import annotations

from dataclasses import dataclass
import math
import time as _time
from typing import Optional

import numpy as np
from rclpy.node import Node

from uav.vehicles.Payload import Payload
from uav.vision_nodes import PayloadColorOrbitNode
from uav.vision_nodes.payload_perception_common import DEFAULT_TAG_FAMILY
from uav_interfaces.srv import PayloadColorOrbitState

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


class PayloadColorOrbitToRearMode(Mode):
    """
    Navigate payload to rear of VTOL when starting on opposite side.

    Startup: full 360° rotation in place (no translation), collect AprilTag poses.
    After scan: if rear tag visible -> dock immediately; else lock orbit direction and
    outward heading, drive to the DLZ edge, turn 90°, follow the boundary by processed
    color metrics, and hand off to docking once the rear tag is aligned.
    """

    mission_target = "payload"
    required_vision_nodes = (PayloadColorOrbitNode,)

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        tag_size_m: float = 0.1,
        tag_family: str = DEFAULT_TAG_FAMILY,
        dock_target_tag_id: int = 1,
        dock_align_angle_deg: float = 30.0,
        dock_align_hold_s: float = 0.4,
        search_spin_rps: float = 0.3,
        go_to_edge_speed_mps: float = 0.12,
        edge_threshold: float = 0.35,
        edge_stable_frames: int = 5,
        edge_min_pixels: int = 500,
        edge_invalid_max_frames: int = 30,
        edge_orbit_speed_mps: float = 0.12,
        edge_k_lat: float = 0.008,
        edge_k_ang: float = 0.6,
        handoff_dock_mode_name: str = "dock",
    ):
        super().__init__(node, vehicle)
        self.tag_size_m = float(tag_size_m)
        self.tag_family = str(tag_family) or DEFAULT_TAG_FAMILY
        self.dock_target_tag_id = int(dock_target_tag_id)
        self.dock_align_angle_deg = float(dock_align_angle_deg)
        self.dock_align_hold_s = float(dock_align_hold_s)
        self.search_spin_rps = float(search_spin_rps)
        self.go_to_edge_speed_mps = float(go_to_edge_speed_mps)
        self.edge_threshold = float(edge_threshold)
        self.edge_stable_frames = int(edge_stable_frames)
        self.edge_min_pixels = int(edge_min_pixels)
        self.edge_invalid_max_frames = int(edge_invalid_max_frames)
        self.edge_orbit_speed_mps = float(edge_orbit_speed_mps)
        self.edge_k_lat = float(edge_k_lat)
        self.edge_k_ang = float(edge_k_ang)
        self.handoff_dock_mode_name = str(handoff_dock_mode_name)

    def on_enter(self) -> None:
        self._phase = "search"
        self._orbit_dir = None
        self._orbit_locked = False
        self._pose_vtol = None
        self._search_total_rot = 0.0
        self._search_poses = []
        self._search_rear_seen = False
        self._outward_heading = None
        self._edge_align_start = None
        self._edge_align_yaw_turned = 0.0
        self._dock_align_start = None
        self._edge_stable_count = 0
        self._edge_invalid_count = 0
        self._last_log_time = 0.0
        self._handoff_requested = False
        self._first_response_logged = False
        self.log("PayloadColorOrbitToRearMode: using PayloadColorOrbitNode service")

    def _request_state(self) -> Optional[PayloadColorOrbitState.Response]:
        request = PayloadColorOrbitState.Request()
        request.tag_size_m = float(self.tag_size_m)
        request.tag_family = self.tag_family
        request.dock_target_tag_id = int(self.dock_target_tag_id)
        return self.send_request(PayloadColorOrbitNode, request)

    def _decode_observations(
        self, response: PayloadColorOrbitState.Response
    ) -> dict[int, TagObservation]:
        counts = [
            len(response.tag_ids),
            len(response.pose_x),
            len(response.pose_y),
            len(response.pose_yaw),
            len(response.tvec_x),
            len(response.tvec_y),
            len(response.tvec_z),
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
            )
        return observations

    def _publish_drive(self, linear: float, angular: float) -> None:
        self.vehicle.drive(linear, angular)

    def on_update(self, time_delta: float) -> None:
        if self._handoff_requested:
            return

        response = self._request_state()
        if response is None:
            return

        if not response.detector_available:
            self.log("PayloadColorOrbitToRearMode: apriltag not installed.")
            return

        if not response.has_image or not response.has_camera_info:
            return

        if not self._first_response_logged:
            self._first_response_logged = True
            self.log("PayloadColorOrbitToRearMode: first perception response received")

        observations = self._decode_observations(response)
        any_visible = bool(observations)
        back_visible = bool(response.dock_target_visible)

        if any_visible:
            closest_id = min(
                observations,
                key=lambda tag_id: observations[tag_id].tvec_z,
            )
            closest = observations[closest_id]
            self._pose_vtol = (closest.pose_x, closest.pose_y, closest.pose_yaw)
        if back_visible and self._phase == "search":
            self._search_rear_seen = True

        now = _time.time()

        if self._phase == "search":
            spin_w = self.search_spin_rps * 2.0 * math.pi
            self._search_total_rot += abs(spin_w) * time_delta
            if any_visible:
                for tag_id, observation in observations.items():
                    self._search_poses.append(
                        (
                            tag_id,
                            (
                                observation.pose_x,
                                observation.pose_y,
                                observation.pose_yaw,
                            ),
                        )
                    )
            if self._search_total_rot < 2.0 * math.pi:
                if now - self._last_log_time >= 2.0:
                    self._last_log_time = now
                    self.log(
                        f"PayloadColorOrbitToRearMode: SEARCH scanning "
                        f"{math.degrees(self._search_total_rot):.0f}/360° "
                        f"tags={len(set(item[0] for item in self._search_poses))} rear_seen={self._search_rear_seen}"
                    )
                self._publish_drive(0.0, spin_w)
                return

            self.log(
                f"PayloadColorOrbitToRearMode: SEARCH complete 360° "
                f"sightings={len(self._search_poses)} rear_seen={self._search_rear_seen}"
            )
            if self._search_rear_seen:
                self._phase = "handoff_to_dock"
                self._handoff_requested = True
                self._publish_drive(0.0, 0.0)
                self.log(
                    "PayloadColorOrbitToRearMode: rear tag seen during scan -> HANDOFF_TO_DOCK (dock immediately)"
                )
                return
            if self._pose_vtol is None:
                self.log(
                    "PayloadColorOrbitToRearMode: no pose after scan, cannot proceed"
                )
                self._publish_drive(0.0, 0.0)
                return
            self._phase = "decide_direction"

        if self._phase == "decide_direction":
            pose_x, pose_y, _ = self._pose_vtol
            theta = math.atan2(pose_y, pose_x)
            angle_to_rear = _wrap_angle(math.pi - theta)
            self._orbit_dir = 1 if angle_to_rear >= 0 else -1
            self._outward_heading = theta
            self._orbit_locked = True
            self._phase = "go_to_edge"
            self.log(
                f"PayloadColorOrbitToRearMode: DECIDE_DIRECTION -> GO_TO_EDGE "
                f"dir={'CCW' if self._orbit_dir == 1 else 'CW'} outward={math.degrees(self._outward_heading):.1f}° (locked)"
            )

        if self._phase == "go_to_edge":
            pink_ratio = float(response.pink_ratio)
            pink_count = int(response.pink_count)
            green_count = int(response.green_count)
            total = pink_count + green_count
            at_edge = total >= self.edge_min_pixels and pink_ratio < self.edge_threshold
            valid = total >= self.edge_min_pixels

            if not valid:
                self._edge_invalid_count += 1
                if self._edge_invalid_count >= self.edge_invalid_max_frames:
                    self.log(
                        "PayloadColorOrbitToRearMode: GO_TO_EDGE too many invalid frames, stopping"
                    )
                    self._publish_drive(0.0, 0.0)
                    return
                self._publish_drive(self.go_to_edge_speed_mps, 0.0)
                return
            self._edge_invalid_count = 0

            if at_edge:
                self._edge_stable_count += 1
                if self._edge_stable_count >= self.edge_stable_frames:
                    self._publish_drive(0.0, 0.0)
                    self._phase = "edge_align"
                    self._edge_stable_count = 0
                    self._edge_align_yaw_turned = 0.0
                    self._edge_align_start = now
                    self.log(
                        "PayloadColorOrbitToRearMode: GO_TO_EDGE -> EDGE_ALIGN (edge detected)"
                    )
                    return
            else:
                self._edge_stable_count = 0

            yaw_err = (
                _wrap_angle(self._outward_heading - self._pose_vtol[2])
                if self._pose_vtol is not None
                else 0.0
            )
            if abs(yaw_err) <= 0.12:
                angular = 0.0
            else:
                angular = float(np.clip(1.0 * yaw_err, -0.5, 0.5))
            self._publish_drive(self.go_to_edge_speed_mps, angular)
            return

        if self._phase == "edge_align":
            turn_target = math.pi / 2.0
            if self._edge_align_yaw_turned >= turn_target - 0.1:
                self._phase = "edge_orbit"
                self.log("PayloadColorOrbitToRearMode: EDGE_ALIGN -> EDGE_ORBIT")
                return
            angular = 0.5 if self._orbit_dir == 1 else -0.5
            self._edge_align_yaw_turned += abs(angular) * time_delta
            self._publish_drive(0.0, angular)
            return

        if self._phase == "edge_orbit":
            lateral_px = (
                float(response.lateral_error_px) if response.boundary_detected else 0.0
            )
            if self._orbit_dir == -1:
                lateral_px = -lateral_px
            boundary_angle = (
                float(response.boundary_angle) if response.boundary_detected else 0.0
            )
            angular = float(
                np.clip(
                    (self.edge_k_lat * lateral_px) + (self.edge_k_ang * boundary_angle),
                    -0.5,
                    0.5,
                )
            )
            linear = self.edge_orbit_speed_mps

            view_angle = (
                float(response.dock_target_view_angle_deg)
                if response.dock_target_visible
                else None
            )
            aligned = view_angle is not None and view_angle <= self.dock_align_angle_deg
            if aligned and back_visible:
                if self._dock_align_start is None:
                    self._dock_align_start = now
                elif (now - self._dock_align_start) >= self.dock_align_hold_s:
                    self._phase = "handoff_to_dock"
                    self._handoff_requested = True
                    self._publish_drive(0.0, 0.0)
                    self.log(
                        f"PayloadColorOrbitToRearMode: EDGE_ORBIT -> HANDOFF_TO_DOCK "
                        f"(back tag aligned {view_angle:.1f}° for {self.dock_align_hold_s}s)"
                    )
                    return
            else:
                self._dock_align_start = None

            self._publish_drive(linear, angular)
            return

        if self._phase == "handoff_to_dock":
            self._publish_drive(0.0, 0.0)

    def check_status(self) -> str:
        if self._handoff_requested and self._phase == "handoff_to_dock":
            return "handoff_dock"
        return "continue"

    def on_exit(self) -> None:
        self.vehicle.stop()
