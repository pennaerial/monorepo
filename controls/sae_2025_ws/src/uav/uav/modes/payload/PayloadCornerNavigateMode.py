"""
PayloadCornerNavigateMode: starting from inside the alternating-border DLZ, drive
out to a tape edge, align with it, then follow the border in the configured
travel direction (cw / ccw) until reaching the first corner.

Phase sequence:
    DRIVE_OUT → TURN_ONTO_TAPE → LINE_FOLLOW → DONE

DRIVE_OUT
    Drive forward at drive_out_speed_mps. Subscribes to the payload camera
    directly (mirroring PayloadRetreatMode) and inspects only the bottom
    drive_out_strip_frac of the frame, so the line is recognised when it is
    physically under the payload, not when it first appears far ahead.

    Two sub-states with hysteresis:
        seeking_tape  – no colour ever seen yet. Drive forward until red OR
                        blue pixel count crosses drive_out_min_pixels for
                        detect_frames consecutive frames; record the dominant
                        colour and advance to crossing_tape.
        crossing_tape – currently driving over the tape. Keep driving forward
                        until red AND blue both fall below drive_out_min_pixels
                        for detect_frames consecutive frames — that means we
                        have crossed the line and are now on the far side.
                        Stop and advance to TURN_ONTO_TAPE.

TURN_ONTO_TAPE
    The payload may have hit the edge at any angle, so use vision feedback to
    rotate in place until the tape is roughly directly ahead. Rotation
    direction is fixed by the desired travel direction:
        direction="ccw" → rotate left (positive angular)
        direction="cw"  → rotate right (negative angular)
    Rationale: traveling ccw around the square keeps the DLZ interior on the
    payload's left, so a left turn from an outward-facing pose lines forward
    up with the ccw travel direction. Mirror for cw.
    Stop when |lateral_error_px| < align_lat_tol_px for align_stable_frames
    consecutive readings (boundary must be detected). Cap rotation at
    max_align_rad as a safety bail-out.

LINE_FOLLOW
    Reuses the steering control law from PayloadDLZNavigateMode (same k_lat,
    k_ang, max_angular). Detects A↔B transitions and stops on the first
    transition that matches the corner signature for the current direction:
        direction="ccw" : A→B = corner
        direction="cw"  : B→A = corner
    Unlike PayloadDLZNavigateMode, _prev_color is seeded from the colour
    actually observed in DRIVE_OUT, so a payload that lands on a "corner-side"
    colour correctly waits for the next full cycle.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from uav.utils import blue, red
from uav.vehicles.Payload import Payload
from uav.vision_nodes import PayloadColorSquareNode
from uav_interfaces.srv import PayloadColorSquareState

from ..Mode import Mode

# HSV ranges shared with PayloadColorSquareNode (uav.utils): A=red (two hue
# ranges because red wraps around the HSV hue wheel), B=blue.
_LOWER_A1 = np.array(red[0][0], dtype=np.uint8)
_UPPER_A1 = np.array(red[0][1], dtype=np.uint8)
_LOWER_A2 = np.array(red[1][0], dtype=np.uint8)
_UPPER_A2 = np.array(red[1][1], dtype=np.uint8)
_LOWER_B = np.array(blue[0], dtype=np.uint8)
_UPPER_B = np.array(blue[1], dtype=np.uint8)


class PayloadCornerNavigateMode(Mode):
    mission_target = "payload"
    required_vision_nodes = (PayloadColorSquareNode,)

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        direction: str = "ccw",
        # DRIVE_OUT
        drive_out_speed_mps: float = 0.12,
        detect_frames: int = 3,
        max_drive_out_s: float = 20.0,
        drive_out_strip_frac: float = 0.20,
        drive_out_min_pixels: int = 150,
        compressed_image: bool = False,
        # TURN_TO_CENTER (rotate CCW until tape is centered in view)
        align_angular_speed: float = 0.4,
        center_tol_px: float = 30.0,
        center_min_pixels: int = 150,
        center_stable_frames: int = 3,
        max_turn_to_center_rad: float = 2.0 * math.pi,
        # (legacy TURN_ONTO_TAPE params, unused while TURN_TO_CENTER is the
        # active alignment phase — kept so the eventual line-follow phase can
        # be re-enabled without changing the mission YAML.)
        align_lat_tol_px: float = 25.0,
        align_stable_frames: int = 3,
        max_align_rad: float = math.pi * 1.25,
        # LINE_FOLLOW
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

        self.drive_out_speed_mps = float(drive_out_speed_mps)
        self.detect_frames = int(detect_frames)
        self.max_drive_out_s = float(max_drive_out_s)
        self.drive_out_strip_frac = float(drive_out_strip_frac)
        if not (0.0 < self.drive_out_strip_frac <= 1.0):
            raise ValueError(
                f"drive_out_strip_frac must be in (0, 1], got {drive_out_strip_frac!r}"
            )
        self.drive_out_min_pixels = int(drive_out_min_pixels)
        self.compressed_image = bool(compressed_image)

        self.align_angular_speed = float(align_angular_speed)
        self.center_tol_px = float(center_tol_px)
        self.center_min_pixels = int(center_min_pixels)
        self.center_stable_frames = int(center_stable_frames)
        self.max_turn_to_center_rad = float(max_turn_to_center_rad)
        self.align_lat_tol_px = float(align_lat_tol_px)
        self.align_stable_frames = int(align_stable_frames)
        self.max_align_rad = float(max_align_rad)

        self.line_follow_speed_mps = float(line_follow_speed_mps)
        self.k_lat = float(k_lat)
        self.k_ang = float(k_ang)
        self.max_angular = float(max_angular)

        self._bridge = CvBridge()
        self._image_sub = None
        self._image: Optional[object] = None

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _request_color_state(self) -> Optional[PayloadColorSquareState.Response]:
        return self.send_request(
            PayloadColorSquareNode, PayloadColorSquareState.Request()
        )

    def _align_angular(self) -> float:
        """Signed angular speed for the alignment turn.
        ccw → turn left (positive). cw → turn right (negative)."""
        return (
            self.align_angular_speed
            if self.direction == "ccw"
            else -self.align_angular_speed
        )

    def _corner_transition(self, prev: str, curr: str) -> bool:
        """True when the A↔B transition is a corner for the current travel direction."""
        if self.direction == "ccw":
            return prev == "A" and curr == "B"
        return prev == "B" and curr == "A"

    # ------------------------------------------------------------------
    # Mode lifecycle
    # ------------------------------------------------------------------

    def on_enter(self) -> None:
        self._phase = "drive_out"
        self._done = False
        self._terminate = False

        # DRIVE_OUT state
        self._drive_out_elapsed = 0.0
        self._do_substate = "seeking_tape"  # "seeking_tape" | "crossing_tape"
        self._enter_streak = 0
        self._exit_streak = 0
        self._first_color: Optional[str] = None
        self._image = None

        # TURN_TO_CENTER state
        self._turn_to_center_rad = 0.0
        self._center_stable = 0

        # (legacy TURN_ONTO_TAPE state)
        self._align_turned = 0.0
        self._align_stable = 0

        # LINE_FOLLOW state
        self._prev_color: Optional[str] = None

        cam_topic = self.vehicle.namespaced_path("camera")
        if self.compressed_image:
            self._image_sub = self.node.create_subscription(
                CompressedImage, cam_topic, self._image_cb, 10
            )
        else:
            self._image_sub = self.node.create_subscription(
                Image, cam_topic, self._image_cb, 10
            )

        self.log(
            f"PayloadCornerNavigateMode: enter direction={self.direction} "
            f"drive_out_speed={self.drive_out_speed_mps:.2f}m/s "
            f"strip_frac={self.drive_out_strip_frac:.2f} "
            f"camera={cam_topic}"
        )

    def on_update(self, time_delta: float) -> None:
        if self._done or self._terminate:
            self.vehicle.stop()
            return

        if self._phase == "drive_out":
            self._update_drive_out(time_delta)
        elif self._phase == "turn_to_center":
            self._update_turn_to_center(time_delta)
        elif self._phase == "turn_onto_tape":
            self._update_turn_onto_tape(time_delta)
        elif self._phase == "line_follow":
            self._update_line_follow(time_delta)

    def check_status(self) -> str:
        if self._done:
            return "done"
        if self._terminate:
            return "terminate"
        return "continue"

    def on_exit(self) -> None:
        self.vehicle.stop()
        if self._image_sub is not None:
            self.node.destroy_subscription(self._image_sub)
            self._image_sub = None
        # Expose the resolved travel direction for downstream modes, matching
        # the convention used by PayloadDLZNavigateMode.on_exit.
        self.node.dlz_navigate_direction = self.direction

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _image_cb(self, msg) -> None:
        self._image = msg

    def _decode_image(self) -> Optional[np.ndarray]:
        if self._image is None:
            return None
        try:
            if self.compressed_image:
                buf = np.frombuffer(self._image.data, dtype=np.uint8)
                return cv2.imdecode(buf, cv2.IMREAD_COLOR)
            return self._bridge.imgmsg_to_cv2(self._image, desired_encoding="bgr8")
        except Exception as exc:
            self.node.get_logger().warn(
                f"PayloadCornerNavigateMode: image decode failed: {exc}"
            )
            return None

    def _lower_strip_color_counts(self, bgr: np.ndarray) -> Tuple[int, int]:
        """Return (red_pixels, blue_pixels) in the bottom drive_out_strip_frac
        of the frame, cropped horizontally to the middle third so tape that
        only clips the corner of the FOV is ignored."""
        h, w = bgr.shape[:2]
        strip_start = int(h * (1.0 - self.drive_out_strip_frac))
        col_start = w // 3
        col_end = w - (w // 3)
        strip = bgr[strip_start:, col_start:col_end]
        hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, _LOWER_A1, _UPPER_A1),
            cv2.inRange(hsv, _LOWER_A2, _UPPER_A2),
        )
        blue_mask = cv2.inRange(hsv, _LOWER_B, _UPPER_B)
        return int(np.count_nonzero(red_mask)), int(np.count_nonzero(blue_mask))

    def _middle_third_color_metrics(
        self, bgr: np.ndarray
    ) -> Tuple[int, float]:
        """Look at the middle horizontal third of the frame (full height) and
        return ``(total_color_pixels, lateral_error_px)`` where
        ``lateral_error_px`` is the centroid x of red+blue pixels minus the
        crop's center x. Positive = colour is right of center.

        Returns ``(0, 0.0)`` if no colour is found."""
        h, w = bgr.shape[:2]
        col_start = w // 3
        col_end = w - (w // 3)
        crop = bgr[:, col_start:col_end]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, _LOWER_A1, _UPPER_A1),
            cv2.inRange(hsv, _LOWER_A2, _UPPER_A2),
        )
        blue_mask = cv2.inRange(hsv, _LOWER_B, _UPPER_B)
        combined = cv2.bitwise_or(red_mask, blue_mask)
        total = int(np.count_nonzero(combined))
        if total == 0:
            return 0, 0.0
        # Mean x-coordinate of nonzero pixels in the crop frame
        ys, xs = np.nonzero(combined)
        centroid_x = float(xs.mean())
        crop_center_x = combined.shape[1] / 2.0
        return total, centroid_x - crop_center_x

    # ------------------------------------------------------------------
    # Phase: DRIVE_OUT
    # ------------------------------------------------------------------

    def _update_drive_out(self, time_delta: float) -> None:
        self._drive_out_elapsed += time_delta
        if self._drive_out_elapsed >= self.max_drive_out_s:
            self.vehicle.stop()
            self._terminate = True
            self.log(
                f"PayloadCornerNavigateMode: DRIVE_OUT timed out after "
                f"{self._drive_out_elapsed:.1f}s without seeing tape — terminating"
            )
            return

        bgr = self._decode_image()
        if bgr is None:
            # No camera frame yet — keep driving forward.
            self.vehicle.drive(self.drive_out_speed_mps, 0.0)
            return

        red_count, blue_count = self._lower_strip_color_counts(bgr)
        color_seen = (
            red_count >= self.drive_out_min_pixels
            or blue_count >= self.drive_out_min_pixels
        )

        if self._do_substate == "seeking_tape":
            if color_seen:
                self._enter_streak += 1
                # Track the dominant colour each frame.
                if red_count >= blue_count:
                    self._first_color = "A"
                else:
                    self._first_color = "B"
                if self._enter_streak >= self.detect_frames:
                    self._do_substate = "crossing_tape"
                    self._exit_streak = 0
                    self.log(
                        f"PayloadCornerNavigateMode: DRIVE_OUT line detected "
                        f"colour={self._first_color} (red={red_count}px blue={blue_count}px) "
                        f"→ crossing_tape"
                    )
            else:
                self._enter_streak = 0
                self._first_color = None
            self.vehicle.drive(self.drive_out_speed_mps, 0.0)
            return

        # crossing_tape: drive until colour disappears, then stop on the far side.
        if not color_seen:
            self._exit_streak += 1
            if self._exit_streak >= self.detect_frames:
                self.vehicle.stop()
                self._phase = "turn_to_center"
                self._turn_to_center_rad = 0.0
                self._center_stable = 0
                self.log(
                    f"PayloadCornerNavigateMode: DRIVE_OUT crossed line "
                    f"(seed colour={self._first_color}) → TURN_TO_CENTER"
                )
                return
        else:
            self._exit_streak = 0
            # Refresh dominant colour while still crossing — covers grazing
            # initial detections that misread the side.
            if red_count > blue_count:
                self._first_color = "A"
            elif blue_count > red_count:
                self._first_color = "B"

        self.vehicle.drive(self.drive_out_speed_mps, 0.0)

    # ------------------------------------------------------------------
    # Phase: TURN_TO_CENTER
    # ------------------------------------------------------------------

    def _update_turn_to_center(self, time_delta: float) -> None:
        # Always turn CCW (positive angular = left turn) for this phase.
        angular = self.align_angular_speed
        self._turn_to_center_rad += abs(angular) * time_delta

        bgr = self._decode_image()
        if bgr is not None:
            total, lateral_error_px = self._middle_third_color_metrics(bgr)
            if (
                total >= self.center_min_pixels
                and abs(lateral_error_px) < self.center_tol_px
            ):
                self._center_stable += 1
                if self._center_stable >= self.center_stable_frames:
                    self.vehicle.stop()
                    # NOTE: temporarily terminate here so TURN_TO_CENTER can be
                    # validated in sim in isolation. Re-enable a transition to
                    # the next phase once this is working.
                    self._done = True
                    self.log(
                        f"PayloadCornerNavigateMode: TURN_TO_CENTER centered "
                        f"(lat={lateral_error_px:.1f}px, total={total}px, "
                        f"turned={math.degrees(self._turn_to_center_rad):.1f}°) → done"
                    )
                    return
            else:
                self._center_stable = 0

        if self._turn_to_center_rad >= self.max_turn_to_center_rad:
            self.vehicle.stop()
            self._terminate = True
            self.log(
                f"PayloadCornerNavigateMode: TURN_TO_CENTER spun "
                f"{math.degrees(self._turn_to_center_rad):.1f}° without centering — "
                f"terminating"
            )
            return

        self.vehicle.drive(0.0, angular)

    # ------------------------------------------------------------------
    # Phase: TURN_ONTO_TAPE
    # ------------------------------------------------------------------

    def _update_turn_onto_tape(self, time_delta: float) -> None:
        angular = self._align_angular()
        self._align_turned += abs(angular) * time_delta

        response = self._request_color_state()
        if response is not None and response.has_image and response.boundary_detected:
            if abs(response.lateral_error_px) < self.align_lat_tol_px:
                self._align_stable += 1
                if self._align_stable >= self.align_stable_frames:
                    self.vehicle.stop()
                    self._phase = "line_follow"
                    # Seed prev_color with whatever the camera sees right now,
                    # falling back to the colour we caught during DRIVE_OUT.
                    seed = response.current_color
                    if seed not in ("A", "B"):
                        seed = self._first_color or "A"
                    self._prev_color = seed
                    self.log(
                        f"PayloadCornerNavigateMode: alignment locked "
                        f"(lat={response.lateral_error_px:.1f}px, "
                        f"turned={math.degrees(self._align_turned):.1f}°) "
                        f"prev_color={self._prev_color} → LINE_FOLLOW"
                    )
                    return
            else:
                self._align_stable = 0
        else:
            self._align_stable = 0

        if self._align_turned >= self.max_align_rad:
            self.vehicle.stop()
            self._phase = "line_follow"
            self._prev_color = self._first_color or "A"
            self.log(
                f"PayloadCornerNavigateMode: alignment timeout at "
                f"{math.degrees(self._align_turned):.1f}° — falling through to LINE_FOLLOW "
                f"with prev_color={self._prev_color}"
            )
            return

        self.vehicle.drive(0.0, angular)

    # ------------------------------------------------------------------
    # Phase: LINE_FOLLOW
    # ------------------------------------------------------------------

    def _update_line_follow(self, time_delta: float) -> None:
        response = self._request_color_state()
        if response is None or not response.has_image:
            self.vehicle.drive(0.0, 0.0)
            return

        curr = response.current_color
        if curr in ("A", "B") and curr != self._prev_color:
            is_corner = self._corner_transition(self._prev_color, curr)
            self.log(
                f"PayloadCornerNavigateMode: colour {self._prev_color}→{curr} "
                f"{'CORNER' if is_corner else 'mid-side'}"
            )
            self._prev_color = curr

            if is_corner:
                self.vehicle.stop()
                self._done = True
                self.log("PayloadCornerNavigateMode: corner reached → done")
                return

        if response.boundary_detected:
            angular = float(
                np.clip(
                    -self.k_lat * response.lateral_error_px
                    + self.k_ang * response.boundary_angle,
                    -self.max_angular,
                    self.max_angular,
                )
            )
        else:
            angular = 0.0

        self.vehicle.drive(self.line_follow_speed_mps, angular)
