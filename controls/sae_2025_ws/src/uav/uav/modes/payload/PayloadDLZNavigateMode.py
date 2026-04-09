from __future__ import annotations

import math
from typing import Optional

import numpy as np
from rclpy.node import Node

from uav.vehicles.Payload import Payload
from uav.vision_nodes import PayloadAprilTagNode, PayloadColorSquareNode
from uav.vision_nodes.payload_perception_common import DEFAULT_TAG_FAMILY
from uav_interfaces.srv import PayloadAprilTagState, PayloadColorSquareState

from ..Mode import Mode

# Angle thresholds
_QUARTER_TURN = math.pi / 2.0   # 90 degrees
_EIGHTH_TURN  = math.pi / 4.0   # 45 degrees
_ANGLE_TOL    = 0.05             # radians, stop slightly early to avoid overshoot

_VALID_START_PHASES = ("wait_for_plane", "scan_tags", "line_follow")


class PayloadDLZNavigateMode(Mode):
    """
    Navigate the payload along the alternating-colour square border of the DLZ.

    Phase sequence (controlled by start_phase):
        WAIT_FOR_PLANE → SCAN_TAGS → TURN_ONTO_TAPE → LINE_FOLLOW

    WAIT_FOR_PLANE
        Hold still facing inward. When any AprilTag is visible for detect_frames
        consecutive frames the plane is confirmed landed → advance to SCAN_TAGS.

    SCAN_TAGS
        Hold still for scan_duration_s and accumulate all visible tag IDs.
        Look up the tag combo in tag_transition_table to determine direction and
        target_transitions. If no match, fall back to the constructor defaults.
        If target_transitions == 0 → done immediately (already at the dock side).

    TURN_ONTO_TAPE
        Fixed 45° turn to align with the tape:
          direction="cw"  → turn left  (positive angular)
          direction="ccw" → turn right (negative angular)

    LINE_FOLLOW
        Follow the combined red/blue tape strip. Detect colour transitions (A↔B)
        to count segments and execute corner turns:
          direction="cw"  : B→A transition = corner → turn right 90°
          direction="ccw" : A→B transition = corner → turn left  90°
        Stop after target_transitions total colour changes.

    tag_transition_table key format
        Keys use ',' to separate AND-groups and '|' for OR within a group.
        "0|4|5,2" = (tag 0 OR 4 OR 5 visible) AND (tag 2 visible).
        Test keys in YAML order — put more-specific rules first.
    """

    mission_target = "payload"
    required_vision_nodes = (PayloadColorSquareNode, PayloadAprilTagNode)

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        direction: str = "ccw",
        target_transitions: int = 1,
        turn_angular_speed: float = 0.5,
        line_follow_speed_mps: float = 0.10,
        k_lat: float = 0.003,
        k_ang: float = 0.4,
        max_angular: float = 0.5,
        # Scan / lookup params
        tag_transition_table: dict = {},
        detect_frames: int = 5,
        scan_duration_s: float = 1.0,
        start_phase: str = "wait_for_plane",
        tag_size_m: float = 0.0508,
        tag_family: str = DEFAULT_TAG_FAMILY,
    ):
        super().__init__(node, vehicle)
        direction = str(direction).lower().strip()
        if direction not in ("cw", "ccw"):
            raise ValueError(f"direction must be 'cw' or 'ccw', got {direction!r}")
        start_phase = str(start_phase).lower().strip()
        if start_phase not in _VALID_START_PHASES:
            raise ValueError(
                f"start_phase must be one of {_VALID_START_PHASES}, got {start_phase!r}"
            )

        # Fallback defaults (overwritten by table lookup at runtime)
        self._default_direction = direction
        self._default_target_transitions = int(target_transitions)

        self.turn_angular_speed = float(turn_angular_speed)
        self.line_follow_speed_mps = float(line_follow_speed_mps)
        self.k_lat = float(k_lat)
        self.k_ang = float(k_ang)
        self.max_angular = float(max_angular)

        self.tag_transition_table = dict(tag_transition_table)
        self.detect_frames = int(detect_frames)
        self.scan_duration_s = float(scan_duration_s)
        self.start_phase = start_phase
        self.tag_size_m = float(tag_size_m)
        self.tag_family = str(tag_family) if tag_family else DEFAULT_TAG_FAMILY

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _turn_angular(self) -> float:
        """Signed angular speed for the initial tape-alignment turn.
        CW direction → turn left (positive). CCW direction → turn right (negative).
        """
        return self.turn_angular_speed if self.direction == "cw" else -self.turn_angular_speed

    def _corner_transition(self, prev: str, curr: str) -> bool:
        """True when the A↔B transition is a corner for the current travel direction."""
        if self.direction == "ccw":
            return prev == "A" and curr == "B"
        else:  # cw
            return prev == "B" and curr == "A"

    def _request_color_state(self) -> Optional[PayloadColorSquareState.Response]:
        return self.send_request(PayloadColorSquareNode, PayloadColorSquareState.Request())

    def _request_apriltag_state(self) -> Optional[PayloadAprilTagState.Response]:
        req = PayloadAprilTagState.Request()
        req.tag_size_m = self.tag_size_m
        req.tag_family = self.tag_family
        return self.send_request(PayloadAprilTagNode, req)

    def _match_table(self, seen_ids: set[int]) -> Optional[dict]:
        """
        Match seen tag IDs against tag_transition_table.
        Key format: comma-separated AND-groups, each group pipe-separated ORs.
        Returns the first matching entry dict, or None.
        """
        for key, entry in self.tag_transition_table.items():
            groups = str(key).split(",")
            if all(
                any(int(t.strip()) in seen_ids for t in group.split("|"))
                for group in groups
            ):
                return entry
        return None

    # ------------------------------------------------------------------
    # Mode lifecycle
    # ------------------------------------------------------------------

    def on_enter(self) -> None:
        # Runtime direction/transitions start as defaults; SCAN_TAGS may overwrite them
        self.direction = self._default_direction
        self.target_transitions = self._default_target_transitions

        self._phase = self.start_phase
        self._consecutive_tag_frames = 0

        # SCAN_TAGS state
        self._scan_elapsed = 0.0
        self._seen_tag_ids: set[int] = set()

        # TURN_ONTO_TAPE state
        self._angle_turned = 0.0

        # LINE_FOLLOW state
        self._prev_color = "A" if self.direction == "cw" else "B"
        self._transitions = 0
        self._lf_phase = "following"   # "following" | "corner_turn"
        self._corner_turned = 0.0
        self._done = False

        self.log(
            f"PayloadDLZNavigateMode: enter  start_phase={self.start_phase}  "
            f"direction={self.direction}  target_transitions={self.target_transitions}"
        )

    def on_update(self, time_delta: float) -> None:
        if self._done:
            self.vehicle.stop()
            return

        if self._phase == "wait_for_plane":
            self._update_wait_for_plane()
        elif self._phase == "scan_tags":
            self._update_scan_tags(time_delta)
        elif self._phase == "turn_onto_tape":
            self._update_turn_onto_tape(time_delta)
        elif self._phase == "line_follow":
            self._update_line_follow(time_delta)

    def check_status(self) -> str:
        if self._done:
            return "done"
        return "continue"

    def on_exit(self) -> None:
        self.vehicle.stop()

    # ------------------------------------------------------------------
    # Phase: WAIT_FOR_PLANE
    # ------------------------------------------------------------------

    def _update_wait_for_plane(self) -> None:
        self.vehicle.stop()
        response = self._request_apriltag_state()
        if response is None or not response.has_image:
            # Service busy or camera not ready — skip without resetting counter
            return
        tag_ids = list(response.all_tag_ids)
        if tag_ids:
            self._consecutive_tag_frames += 1
            self.log(
                f"PayloadDLZNavigateMode: WAIT_FOR_PLANE — tags={tag_ids}  "
                f"consecutive={self._consecutive_tag_frames}/{self.detect_frames}"
            )
            if self._consecutive_tag_frames >= self.detect_frames:
                self._phase = "scan_tags"
                self._scan_elapsed = 0.0
                self._seen_tag_ids = set()
                self.log(
                    f"PayloadDLZNavigateMode: plane detected ({self._consecutive_tag_frames} frames) "
                    f"→ SCAN_TAGS"
                )
        else:
            self.log("PayloadDLZNavigateMode: WAIT_FOR_PLANE — no tags visible, resetting counter")
            self._consecutive_tag_frames = 0

    # ------------------------------------------------------------------
    # Phase: SCAN_TAGS
    # ------------------------------------------------------------------

    def _update_scan_tags(self, time_delta: float) -> None:
        self.vehicle.stop()
        self._scan_elapsed += time_delta

        response = self._request_apriltag_state()
        if response is not None and response.has_image:
            for tag_id in response.all_tag_ids:
                self._seen_tag_ids.add(int(tag_id))

        if self._scan_elapsed < self.scan_duration_s:
            return

        # Scan complete — look up table
        self.log(
            f"PayloadDLZNavigateMode: SCAN_TAGS complete  seen={sorted(self._seen_tag_ids)}"
        )
        entry = self._match_table(self._seen_tag_ids)
        if entry is not None:
            self.direction = str(entry["direction"]).lower().strip()
            self.target_transitions = int(entry["transitions"])
            self.log(
                f"PayloadDLZNavigateMode: table match → direction={self.direction}  "
                f"target_transitions={self.target_transitions}"
            )
        else:
            self.log(
                f"PayloadDLZNavigateMode: no table match for {sorted(self._seen_tag_ids)}  "
                f"using fallback direction={self.direction}  "
                f"target_transitions={self.target_transitions}"
            )

        # Update _prev_color now that direction is finalised
        self._prev_color = "A" if self.direction == "cw" else "B"

        if self.target_transitions == 0:
            self._done = True
            self.log("PayloadDLZNavigateMode: target_transitions=0 → already at dock → done")
            return

        # start_phase=="scan_tags" means we're already on the tape → skip the alignment turn
        next_phase = "line_follow" if self.start_phase == "scan_tags" else "turn_onto_tape"
        self._phase = next_phase
        self.log(f"PayloadDLZNavigateMode: → {next_phase.upper()}")

    # ------------------------------------------------------------------
    # Phase: TURN_ONTO_TAPE
    # ------------------------------------------------------------------

    def _update_turn_onto_tape(self, time_delta: float) -> None:
        angular = self._turn_angular()
        self._angle_turned += abs(angular) * time_delta

        if self._angle_turned >= _EIGHTH_TURN - _ANGLE_TOL:
            self.vehicle.stop()
            self._phase = "line_follow"
            self.log(
                f"PayloadDLZNavigateMode: TURN_ONTO_TAPE complete "
                f"({math.degrees(self._angle_turned):.1f}°) → LINE_FOLLOW"
            )
            return

        self.vehicle.drive(0.0, angular)

    # ------------------------------------------------------------------
    # Phase: LINE_FOLLOW
    # ------------------------------------------------------------------

    def _update_line_follow(self, time_delta: float) -> None:
        if self._lf_phase == "corner_turn":
            self._do_corner_turn(time_delta)
            return

        # Query vision
        response = self._request_color_state()
        if response is None or not response.has_image:
            self.vehicle.drive(0.0, 0.0)
            return

        # Colour transition detection
        curr = response.current_color
        if curr in ("A", "B") and curr != self._prev_color:
            self._transitions += 1
            is_corner = self._corner_transition(self._prev_color, curr)
            self.log(
                f"PayloadDLZNavigateMode: colour {self._prev_color}→{curr}  "
                f"{'CORNER' if is_corner else 'mid-side'}  "
                f"transitions={self._transitions}/{self.target_transitions}"
            )
            self._prev_color = curr

            if self._transitions >= self.target_transitions:
                self.vehicle.stop()
                self._done = True
                self.log("PayloadDLZNavigateMode: target_transitions reached → done")
                return

            if is_corner:
                self._lf_phase = "corner_turn"
                self._corner_turned = 0.0
                self.vehicle.drive(0.0, 0.0)
                return

        # Boundary following
        if response.boundary_detected:
            # lateral_error_px > 0 → tape is right of centre → steer right (negative angular)
            angular = float(np.clip(
                -self.k_lat * response.lateral_error_px + self.k_ang * response.boundary_angle,
                -self.max_angular,
                self.max_angular,
            ))
        else:
            angular = 0.0

        self.vehicle.drive(self.line_follow_speed_mps, angular)

    def _do_corner_turn(self, time_delta: float) -> None:
        # Corner turns are opposite to the initial alignment turn:
        #   CCW travel → turn left (+), CW travel → turn right (-)
        angular = -self._turn_angular()
        self._corner_turned += abs(angular) * time_delta

        if self._corner_turned >= _EIGHTH_TURN - _ANGLE_TOL:
            self._lf_phase = "following"
            self.vehicle.drive(0.0, 0.0)
            self.log("PayloadDLZNavigateMode: corner turn complete → FOLLOWING")
            return

        self.vehicle.drive(0.0, angular)
