from __future__ import annotations

import math
from typing import Optional

import numpy as np
from rclpy.node import Node

from uav.vehicles.Payload import Payload
from uav.vision_nodes import PayloadColorSquareNode
from uav_interfaces.srv import PayloadColorSquareState

from ..Mode import Mode

# Angle thresholds
_QUARTER_TURN = math.pi / 2.0   # 90 degrees
_EIGHTH_TURN  = math.pi / 4.0   # 45 degrees
_ANGLE_TOL    = 0.05             # radians, stop slightly early to avoid overshoot


class PayloadDLZNavigateMode(Mode):
    """
    Navigate the payload along the alternating-colour square border of the DLZ.

    Phase 1 – TURN_ONTO_TAPE
        The payload starts at a corner of the square, facing inward at 45°.
        A fixed 45° turn aligns it with the tape:
          direction="cw"  → turn left  (positive angular, CCW robot spin)
          direction="ccw" → turn right (negative angular, CW robot spin)

    Phase 2 – LINE_FOLLOW
        Follow the combined orange/blue tape strip using lateral-error steering.
        Detect colour transitions (A↔B) to count segments and handle corners:
          direction="cw"  : B→A transition = corner → turn right 90°
          direction="ccw" : A→B transition = corner → turn left  90°
        Stop after target_transitions total colour changes.
    """

    mission_target = "payload"
    required_vision_nodes = (PayloadColorSquareNode,)

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
    ):
        super().__init__(node, vehicle)
        direction = str(direction).lower().strip()
        if direction not in ("cw", "ccw"):
            raise ValueError(f"direction must be 'cw' or 'ccw', got {direction!r}")
        self.direction = direction
        self.target_transitions = int(target_transitions)
        self.turn_angular_speed = float(turn_angular_speed)
        self.line_follow_speed_mps = float(line_follow_speed_mps)
        self.k_lat = float(k_lat)
        self.k_ang = float(k_ang)
        self.max_angular = float(max_angular)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _turn_angular(self) -> float:
        """Signed angular speed for initial tape-alignment turn and corner turns.
        CW direction → turn left (positive). CCW direction → turn right (negative).
        """
        return self.turn_angular_speed if self.direction == "cw" else -self.turn_angular_speed

    def _corner_transition(self, prev: str, curr: str) -> bool:
        """True when the A↔B transition is a corner for the current travel direction."""
        if self.direction == "ccw":
            return prev == "A" and curr == "B"
        else:  # cw
            return prev == "B" and curr == "A"

    def _request_state(self) -> Optional[PayloadColorSquareState.Response]:
        return self.send_request(PayloadColorSquareNode, PayloadColorSquareState.Request())

    # ------------------------------------------------------------------
    # Mode lifecycle
    # ------------------------------------------------------------------

    def on_enter(self) -> None:
        self._phase = "turn_onto_tape"
        self._angle_turned = 0.0

        # LINE_FOLLOW state
        # Expected initial colour: CW starts on A (orange), CCW starts on B (blue)
        self._prev_color = "A" if self.direction == "cw" else "B"
        self._transitions = 0
        self._lf_phase = "following"   # "following" | "corner_turn"
        self._corner_turned = 0.0
        self._done = False

        self.log(
            f"PayloadDLZNavigateMode: enter  direction={self.direction}  "
            f"target_transitions={self.target_transitions}"
        )

    def on_update(self, time_delta: float) -> None:
        if self._done:
            self.vehicle.stop()
            return

        if self._phase == "turn_onto_tape":
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
    # Phase: TURN_ONTO_TAPE
    # ------------------------------------------------------------------

    def _update_turn_onto_tape(self, time_delta: float) -> None:
        angular = self._turn_angular()
        self._angle_turned += abs(angular) * time_delta

        if self._angle_turned >= _QUARTER_TURN - _ANGLE_TOL:
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
        response = self._request_state()
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
            # lateral_error_px > 0  → tape is right of centre → steer right (negative angular)
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
