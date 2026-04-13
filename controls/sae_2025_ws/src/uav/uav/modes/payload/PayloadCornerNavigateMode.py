"""
PayloadCornerNavigateMode: starting from inside the alternating-border DLZ, drive
out to a tape edge, align with it, then follow the border in the configured
travel direction (cw / ccw) until reaching the first corner.

Phase sequence:
    DRIVE_OUT → TURN_TO_CENTER → LINE_FOLLOW → DONE

DRIVE_OUT
    Drive forward at drive_out_speed_mps. Subscribes to the payload camera
    directly (mirroring PayloadRetreatMode) and inspects only the bottom
    drive_out_strip_frac of the frame, so the line is recognised when it is
    physically under the payload, not when it first appears far ahead.

    Two sub-states with hysteresis:
        seeking_tape  – no colour ever seen yet. Drive forward until colour A
                        OR colour B pixel count crosses drive_out_min_pixels
                        for detect_frames consecutive frames; record the
                        dominant colour and advance to crossing_tape.
        crossing_tape – currently driving over the tape. Keep driving forward
                        until A AND B both fall below drive_out_min_pixels
                        for detect_frames consecutive frames — that means we
                        have crossed the line and are now on the far side.
                        Stop and advance to TURN_TO_CENTER.

TURN_TO_CENTER
    Rotate in place (always CCW) until the border tape is roughly centered in
    the middle-third of the frame. Locks when the combined-mask centroid is
    within center_tol_px of crop center for center_stable_frames consecutive
    frames; the dominant colour observed during the lock seeds LINE_FOLLOW.
    Caps rotation at max_turn_to_center_rad as a safety bail-out.

LINE_FOLLOW
    Proportional-steering line follower on the currently-tracked colour's
    centroid within a wide bottom strip. Detects A↔B transitions (using the
    same 1.5× dominance rule as PayloadColorSquareNode) and stops on the
    first transition that matches the corner signature for the current
    direction:
        direction="ccw" : B→A = corner  (red→blue with default YAML)
        direction="cw"  : A→B = corner  (blue→red with default YAML)
    _prev_color is seeded from TURN_TO_CENTER's last dominant observation,
    falling back to the colour caught during DRIVE_OUT.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from uav.vehicles.Payload import Payload

from ..Mode import Mode

# Mirrors PayloadColorSquareNode._COLOR_RATIO: when one tape colour has at
# least this many times more pixels than the other in the line-follow strip,
# we treat it as the new dominant colour and trigger a transition.
_COLOR_DOMINANCE_RATIO = 1.5


class PayloadCornerNavigateMode(Mode):
    mission_target = "payload"

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        direction: str = "ccw",
        # HSV color ranges — ccw=color A, cw=color B
        ccw_lower_hsv: list[int] = (0, 80, 80),
        ccw_upper_hsv: list[int] = (10, 255, 255),
        cw_lower_hsv: list[int] = (85, 120, 60),
        cw_upper_hsv: list[int] = (140, 255, 255),
        # DRIVE_OUT
        drive_out_speed_mps: float = 0.12,
        detect_frames: int = 3,
        max_drive_out_s: float = 30.0,
        drive_out_strip_frac: float = 0.30,
        drive_out_min_pixels: int = 150,
        compressed_image: bool = False,
        # TURN_TO_CENTER
        align_angular_speed: float = 0.4,
        center_tol_px: float = 30.0,
        center_min_pixels: int = 150,
        center_stable_frames: int = 3,
        max_turn_to_center_rad: float = 2.0 * math.pi,
        # LINE_FOLLOW
        line_follow_speed_mps: float = 0.10,
        line_follow_strip_frac: float = 0.40,
        line_follow_min_pixels: int = 50,
        k_lat: float = 0.003,
        max_angular: float = 0.5,
        # TAPE_ALIGN — rotate until diagonal black-tape centroid is centered
        tape_align_lower_hsv: list[int] = (0, 0, 0),
        tape_align_upper_hsv: list[int] = (180, 255, 60),
        tape_align_angular_speed: float = 0.6,
        tape_align_center_tol_px: float = 20.0,
        tape_align_min_pixels: int = 100,
        tape_align_stable_frames: int = 3,
    ):
        super().__init__(node, vehicle)
        direction = str(direction).lower().strip()
        if direction not in ("cw", "ccw"):
            raise ValueError(f"direction must be 'cw' or 'ccw', got {direction!r}")
        self.direction = direction

        self._lower_a = np.array(ccw_lower_hsv, dtype=np.uint8)
        self._upper_a = np.array(ccw_upper_hsv, dtype=np.uint8)
        self._lower_b = np.array(cw_lower_hsv, dtype=np.uint8)
        self._upper_b = np.array(cw_upper_hsv, dtype=np.uint8)

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

        self.line_follow_speed_mps = float(line_follow_speed_mps)
        self.line_follow_strip_frac = float(line_follow_strip_frac)
        if not (0.0 < self.line_follow_strip_frac <= 1.0):
            raise ValueError(
                f"line_follow_strip_frac must be in (0, 1], got {line_follow_strip_frac!r}"
            )
        self.line_follow_min_pixels = int(line_follow_min_pixels)
        self.k_lat = float(k_lat)
        self.max_angular = float(max_angular)

        self._lower_black = np.array(tape_align_lower_hsv, dtype=np.uint8)
        self._upper_black = np.array(tape_align_upper_hsv, dtype=np.uint8)
        self.tape_align_angular_speed = float(tape_align_angular_speed)
        self.tape_align_center_tol_px = float(tape_align_center_tol_px)
        self.tape_align_min_pixels = int(tape_align_min_pixels)
        self.tape_align_stable_frames = int(tape_align_stable_frames)

        self._bridge = CvBridge()
        self._image_sub = None
        self._image: Optional[object] = None
        self._annotated_pub = None

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _corner_transition(self, prev: str, curr: str) -> bool:
        """True when the A↔B transition is a corner for the current travel direction."""
        if self.direction == "ccw":
            return prev == "B" and curr == "A"
        return prev == "A" and curr == "B"

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
        self._latest_dominant: Optional[str] = None

        # LINE_FOLLOW state
        self._prev_color: Optional[str] = None
        self._corner_color_seen: bool = False

        # TAPE_ALIGN state
        self._tape_align_stable: int = 0
        # Starts True — transitions are armed immediately on entry. Set to
        # False after each mid-side swap to suppress false corners from
        # residual pixels at the boundary we just crossed; re-arms once the
        # residual drops below line_follow_min_pixels.
        self._boundary_cleared: bool = True

        cam_topic = self.vehicle.namespaced_path("camera")
        if self.compressed_image:
            self._image_sub = self.node.create_subscription(
                CompressedImage, f"{cam_topic}/compressed", self._image_cb, 1
            )
        else:
            self._image_sub = self.node.create_subscription(
                Image, cam_topic, self._image_cb, 1
            )

        annotated_topic = self.vehicle.namespaced_path(
            "annotated_image/compressed"
        )
        self._annotated_pub = self.node.create_publisher(
            CompressedImage, annotated_topic, 1
        )

        self.log(
            f"PayloadCornerNavigateMode: enter direction={self.direction} "
            f"drive_out_speed={self.drive_out_speed_mps:.2f}m/s "
            f"strip_frac={self.drive_out_strip_frac:.2f} "
            f"camera={cam_topic} annotated={annotated_topic}"
        )

    def on_update(self, time_delta: float) -> None:
        if self._done or self._terminate:
            self.vehicle.stop()
            return

        if self._phase == "drive_out":
            self._update_drive_out(time_delta)
        elif self._phase == "turn_to_center":
            self._update_turn_to_center(time_delta)
        elif self._phase == "line_follow":
            self._update_line_follow(time_delta)
        elif self._phase == "tape_align":
            self._update_tape_align()

    def check_status(self) -> str:
        # Both the success path (_done) and the safety bail-outs (_terminate)
        # end the mission. ModeManager.handle_mode_state treats "terminate" as
        # the built-in mission-end state; any other value requires a YAML
        # transition entry, so we report "terminate" for both.
        if self._done or self._terminate:
            return "terminate"
        return "continue"

    def on_exit(self) -> None:
        self.vehicle.stop()
        if self._image_sub is not None:
            self.node.destroy_subscription(self._image_sub)
            self._image_sub = None
        if self._annotated_pub is not None:
            self.node.destroy_publisher(self._annotated_pub)
            self._annotated_pub = None
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
        """Return (color_a_pixels, color_b_pixels) in the bottom drive_out_strip_frac
        of the frame, cropped horizontally to the middle third so tape that
        only clips the corner of the FOV is ignored."""
        h, w = bgr.shape[:2]
        strip_start = int(h * (1.0 - self.drive_out_strip_frac))
        col_start = w // 3
        col_end = w - (w // 3)
        strip = bgr[strip_start:, col_start:col_end]
        hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
        mask_a = cv2.inRange(hsv, self._lower_a, self._upper_a)
        mask_b = cv2.inRange(hsv, self._lower_b, self._upper_b)
        return int(np.count_nonzero(mask_a)), int(np.count_nonzero(mask_b))

    def _middle_third_color_metrics(
        self, bgr: np.ndarray
    ) -> Tuple[int, float, Optional[str]]:
        """Look at the middle horizontal third of the frame (full height) and
        return ``(total_color_pixels, lateral_error_px, dominant_color)``:

        - ``total_color_pixels`` — combined color_a+color_b pixel count in the crop
        - ``lateral_error_px`` — centroid x of those pixels minus the crop's
          center x; positive means colour is right of center
        - ``dominant_color`` — ``"A"``, ``"B"``, or ``None`` if
          neither dominates / no colour is found

        Returns ``(0, 0.0, None)`` if no colour is found."""
        h, w = bgr.shape[:2]
        row_start = int(h * 0.6)
        col_start = w // 3
        col_end = w - (w // 3)
        crop = bgr[row_start:, col_start:col_end]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask_a = cv2.inRange(hsv, self._lower_a, self._upper_a)
        mask_b = cv2.inRange(hsv, self._lower_b, self._upper_b)
        count_a = int(np.count_nonzero(mask_a))
        count_b = int(np.count_nonzero(mask_b))
        total = count_a + count_b
        if total == 0:
            return 0, 0.0, None
        combined = cv2.bitwise_or(mask_a, mask_b)
        ys, xs = np.nonzero(combined)
        centroid_x = float(xs.mean())
        crop_center_x = combined.shape[1] / 2.0
        dominant: Optional[str]
        if count_a > count_b:
            dominant = "A"
        elif count_b > count_a:
            dominant = "B"
        else:
            dominant = None
        return total, centroid_x - crop_center_x, dominant

    def _single_color_strip_metrics(
        self, bgr: np.ndarray, color: str
    ) -> Tuple[int, float]:
        """Return ``(pixel_count, lateral_error_px)`` for a single colour
        (``"A"`` or ``"B"``) within the bottom
        ``line_follow_strip_frac`` of the frame, full width.
        ``lateral_error_px`` = centroid x of that colour minus the strip's
        center x. Positive = colour is right of center.

        Returns ``(0, 0.0)`` if the colour is not found in the strip."""
        h, w = bgr.shape[:2]
        strip_start = int(h * (1.0 - self.line_follow_strip_frac))
        strip = bgr[strip_start:, :]
        hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
        if color == "A":
            mask = cv2.inRange(hsv, self._lower_a, self._upper_a)
        elif color == "B":
            mask = cv2.inRange(hsv, self._lower_b, self._upper_b)
        else:
            return 0, 0.0
        count = int(np.count_nonzero(mask))
        if count == 0:
            return 0, 0.0
        ys, xs = np.nonzero(mask)
        centroid_x = float(xs.mean())
        strip_center_x = strip.shape[1] / 2.0
        return count, centroid_x - strip_center_x

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

        count_a, count_b = self._lower_strip_color_counts(bgr)
        color_seen = (
            count_a >= self.drive_out_min_pixels
            or count_b >= self.drive_out_min_pixels
        )

        if self._do_substate == "seeking_tape":
            if color_seen:
                self._enter_streak += 1
                # Track the dominant colour each frame.
                if count_a >= count_b:
                    self._first_color = "A"
                else:
                    self._first_color = "B"
                if self._enter_streak >= self.detect_frames:
                    self._do_substate = "crossing_tape"
                    self._exit_streak = 0
                    self.log(
                        f"PayloadCornerNavigateMode: DRIVE_OUT line detected "
                        f"colour={self._first_color} (A={count_a}px B={count_b}px) "
                        f"→ crossing_tape"
                    )
            else:
                self._enter_streak = 0
                self._first_color = None
            self._annotate_drive_out(bgr, count_a, count_b)
            self.vehicle.drive(self.drive_out_speed_mps, 0.0)
            return

        # crossing_tape: drive until colour disappears, then stop on the far side.
        if not color_seen:
            self._exit_streak += 1
            if self._exit_streak >= self.detect_frames:
                self.vehicle.stop()
                self._annotate_drive_out(bgr, count_a, count_b)
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
            if count_a > count_b:
                self._first_color = "A"
            elif count_b > count_a:
                self._first_color = "B"

        self._annotate_drive_out(bgr, count_a, count_b)
        self.vehicle.drive(self.drive_out_speed_mps, 0.0)

    # ------------------------------------------------------------------
    # Phase: TURN_TO_CENTER
    # ------------------------------------------------------------------

    def _update_turn_to_center(self, time_delta: float) -> None:
        # Always turn CCW (positive angular = left turn) for this phase.
        angular = self.align_angular_speed
        self._turn_to_center_rad += abs(angular) * time_delta

        bgr = self._decode_image()
        total = 0
        lateral_error_px = 0.0
        dominant: Optional[str] = None
        if bgr is not None:
            total, lateral_error_px, dominant = self._middle_third_color_metrics(bgr)
            if (
                total >= self.center_min_pixels
                and abs(lateral_error_px) < self.center_tol_px
            ):
                # Track the latest dominant colour while we hold the centered
                # condition so the line-follow seed reflects whatever the
                # camera was actually looking at when we locked.
                if dominant is not None:
                    self._latest_dominant = dominant
                self._center_stable += 1
                if self._center_stable >= self.center_stable_frames:
                    self.vehicle.stop()
                    self._prev_color = (
                        self._latest_dominant or dominant or self._first_color or "A"
                    )
                    self._annotate_turn_to_center(
                        bgr, total, lateral_error_px, dominant
                    )
                    self._phase = "line_follow"
                    self.log(
                        f"PayloadCornerNavigateMode: TURN_TO_CENTER centered "
                        f"(lat={lateral_error_px:.1f}px, total={total}px, "
                        f"turned={math.degrees(self._turn_to_center_rad):.1f}°) "
                        f"prev_color={self._prev_color} → LINE_FOLLOW"
                    )
                    return
            else:
                self._center_stable = 0
                if dominant is not None:
                    self._latest_dominant = dominant

        if self._turn_to_center_rad >= self.max_turn_to_center_rad:
            self.vehicle.stop()
            self._terminate = True
            self.log(
                f"PayloadCornerNavigateMode: TURN_TO_CENTER spun "
                f"{math.degrees(self._turn_to_center_rad):.1f}° without centering — "
                f"terminating"
            )
            return

        if bgr is not None:
            self._annotate_turn_to_center(bgr, total, lateral_error_px, dominant)
        self.vehicle.drive(0.0, angular)

    # ------------------------------------------------------------------
    # Phase: LINE_FOLLOW
    # ------------------------------------------------------------------

    def _update_line_follow(self, time_delta: float) -> None:
        bgr = self._decode_image()
        if bgr is None:
            self.vehicle.drive(0.0, 0.0)
            return

        current = self._prev_color or "A"
        other = "B" if current == "A" else "A"

        cur_count, cur_lateral_px = self._single_color_strip_metrics(bgr, current)
        other_count, other_lateral_px = self._single_color_strip_metrics(bgr, other)
        transitioned_this_tick = False

        # After a mid-side swap, the "other" colour (residual from the
        # boundary we just crossed) must drop below line_follow_min_pixels at
        # least once before we check for new transitions. This proves the
        # payload has physically moved away from the boundary, so the next
        # time the other colour appears it's a real new boundary, not residual
        # pixels from the old one.
        if not self._boundary_cleared:
            if other_count < self.line_follow_min_pixels:
                self._boundary_cleared = True
        elif not self._corner_color_seen:
            transitioned = (
                other_count >= self.line_follow_min_pixels
                and other_count > cur_count * _COLOR_DOMINANCE_RATIO
            )

            if transitioned:
                transitioned_this_tick = True
                if self._corner_transition(current, other):
                    # CORNER. Latch the flag but keep tracking the current
                    # colour (which is now disappearing) so we drive straight
                    # on through the end of our side's segment into the corner.
                    self._corner_color_seen = True
                    self.log(
                        f"PayloadCornerNavigateMode: corner {current}→{other} "
                        f"({other_count}px) — driving into corner until "
                        f"{current} disappears"
                    )
                else:
                    # Mid-side transition. Switch the tracked colour and
                    # require the old colour's residual pixels to clear
                    # before checking for further transitions.
                    self._boundary_cleared = False
                    self.log(
                        f"PayloadCornerNavigateMode: mid-side {current}→{other} "
                        f"({other_count}px) — switching tracked colour to "
                        f"{other}"
                    )
                    self._prev_color = other
                    current, other = other, current
                    cur_count, cur_lateral_px = other_count, other_lateral_px
                    other_count, other_lateral_px = (
                        self._single_color_strip_metrics(bgr, other)
                    )

        # Stop condition: corner has been detected and the colour we are
        # following has dropped below the visibility threshold (we've driven
        # past the end of our side's segment into the corner).
        if self._corner_color_seen and cur_count < self.line_follow_min_pixels:
            self.vehicle.stop()
            self._annotate_line_follow(
                bgr,
                current,
                cur_count,
                cur_lateral_px,
                other,
                other_count,
                other_lateral_px,
                transitioned_this_tick,
                0.0,
            )
            self._phase = "tape_align"
            self._tape_align_stable = 0
            self.log(
                f"PayloadCornerNavigateMode: current colour {current} lost "
                f"after corner detected → TAPE_ALIGN"
            )
            return

        # Steer purely on the current colour's centroid so the other colour
        # cannot pull the centroid off the side we're following.
        if cur_count >= self.line_follow_min_pixels:
            angular = float(
                np.clip(
                    -self.k_lat * cur_lateral_px,
                    -self.max_angular,
                    self.max_angular,
                )
            )
        else:
            angular = 0.0

        self._annotate_line_follow(
            bgr,
            current,
            cur_count,
            cur_lateral_px,
            other,
            other_count,
            other_lateral_px,
            transitioned_this_tick,
            angular,
        )
        self.vehicle.drive(self.line_follow_speed_mps, angular)

    # ------------------------------------------------------------------
    # Phase: TAPE_ALIGN
    # ------------------------------------------------------------------

    def _black_centroid_metrics(self, bgr: np.ndarray) -> Tuple[int, float]:
        """Return ``(pixel_count, lateral_error_px)`` for black tape in the
        bottom ``line_follow_strip_frac`` of the frame, full width.
        ``lateral_error_px`` = centroid x minus strip center x; positive means
        tape is right of center."""
        h, w = bgr.shape[:2]
        strip_start = int(h * (1.0 - self.line_follow_strip_frac))
        strip = bgr[strip_start:, :]
        hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower_black, self._upper_black)
        count = int(np.count_nonzero(mask))
        if count == 0:
            return 0, 0.0
        ys, xs = np.nonzero(mask)
        centroid_x = float(xs.mean())
        strip_center_x = strip.shape[1] / 2.0
        return count, centroid_x - strip_center_x

    def _update_tape_align(self) -> None:
        bgr = self._decode_image()
        if bgr is None:
            self.vehicle.stop()
            return

        count, lateral_px = self._black_centroid_metrics(bgr)

        if count < self.tape_align_min_pixels:
            # Tape not visible yet — rotate slowly to search
            self.vehicle.drive(0.0, self.tape_align_angular_speed)
            self._tape_align_stable = 0
            self._annotate_tape_align(bgr, count, lateral_px, "searching")
            self.log(
                f"PayloadCornerNavigateMode: TAPE_ALIGN searching "
                f"(black_px={count})"
            )
            return

        # Proportional correction toward center
        if abs(lateral_px) <= self.tape_align_center_tol_px:
            self._tape_align_stable += 1
            self.vehicle.stop()
            if self._tape_align_stable >= self.tape_align_stable_frames:
                self._annotate_tape_align(bgr, count, lateral_px, "LOCKED")
                self.log(
                    f"PayloadCornerNavigateMode: TAPE_ALIGN centered "
                    f"(lat={lateral_px:.1f}px, px={count}) → done"
                )
                self._done = True
            else:
                self._annotate_tape_align(bgr, count, lateral_px, "centering")
        else:
            self._tape_align_stable = 0
            direction = -1.0 if lateral_px > 0 else 1.0
            self.vehicle.drive(0.0, direction * self.tape_align_angular_speed)
            self._annotate_tape_align(bgr, count, lateral_px, "turning")
            self.log(
                f"PayloadCornerNavigateMode: TAPE_ALIGN turning "
                f"lat={lateral_px:.1f}px black_px={count}"
            )

    # ------------------------------------------------------------------
    # Annotated debug image
    # ------------------------------------------------------------------

    # BGR colours used by the annotators. Yellow for colour A, orange for B —
    # chosen so they never collide with the underlying red/blue tape pixels.
    _DBG_COLOR_A = (0, 255, 255)
    _DBG_COLOR_B = (0, 165, 255)
    _DBG_CROP = (80, 80, 80)
    _DBG_CENTER = (255, 255, 255)
    _DBG_OK = (0, 255, 0)
    _DBG_WARN = (0, 128, 255)
    _DBG_TOL = (80, 160, 80)

    def _draw_shifted_contours(
        self,
        debug: np.ndarray,
        mask: np.ndarray,
        x_offset: int,
        y_offset: int,
        color: Tuple[int, int, int],
        thickness: int,
    ) -> None:
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return
        shift = np.array([[[x_offset, y_offset]]])
        shifted = [c + shift for c in contours]
        cv2.drawContours(debug, shifted, -1, color, thickness)

    def _put_label(
        self,
        debug: np.ndarray,
        text: str,
        y: int,
        color: Tuple[int, int, int] = (200, 200, 200),
        scale: float = 0.5,
        thickness: int = 1,
    ) -> None:
        cv2.putText(
            debug,
            text,
            (8, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
        )

    def _publish_annotated(self, debug: np.ndarray) -> None:
        if self._annotated_pub is None:
            return
        try:
            msg = self._bridge.cv2_to_compressed_imgmsg(debug, dst_format="jpeg")
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self._annotated_pub.publish(msg)
        except Exception as exc:
            self.node.get_logger().warn(
                f"PayloadCornerNavigateMode: annotated publish failed: {exc}"
            )

    def _annotate_drive_out(
        self, bgr: np.ndarray, count_a: int, count_b: int
    ) -> None:
        if self._annotated_pub is None or bgr is None:
            return
        debug = bgr.copy()
        h, w = bgr.shape[:2]
        y0 = int(h * (1.0 - self.drive_out_strip_frac))
        x0 = w // 3
        x1 = w - (w // 3)
        # Recompute the same masks the logic used so the contours exactly
        # match what DRIVE_OUT was thresholding on.
        strip = bgr[y0:, x0:x1]
        hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
        mask_a = cv2.inRange(hsv, self._lower_a, self._upper_a)
        mask_b = cv2.inRange(hsv, self._lower_b, self._upper_b)
        cv2.rectangle(debug, (x0, y0), (x1 - 1, h - 1), self._DBG_CROP, 1)
        self._draw_shifted_contours(debug, mask_a, x0, y0, self._DBG_COLOR_A, 2)
        self._draw_shifted_contours(debug, mask_b, x0, y0, self._DBG_COLOR_B, 2)
        self._put_label(
            debug,
            f"DRIVE_OUT [{self._do_substate}] dir={self.direction}",
            22,
            (0, 255, 255),
            0.6,
            2,
        )
        self._put_label(
            debug,
            f"A={count_a}px B={count_b}px  min={self.drive_out_min_pixels}",
            44,
        )
        self._put_label(
            debug,
            f"enter={self._enter_streak}/{self.detect_frames} "
            f"exit={self._exit_streak}/{self.detect_frames} "
            f"first={self._first_color} t={self._drive_out_elapsed:.1f}s",
            62,
        )
        self._publish_annotated(debug)

    def _annotate_turn_to_center(
        self,
        bgr: np.ndarray,
        total: int,
        lateral_error_px: float,
        dominant: Optional[str],
    ) -> None:
        if self._annotated_pub is None or bgr is None:
            return
        debug = bgr.copy()
        h, w = bgr.shape[:2]
        y0 = int(h * 0.6)
        x0 = w // 3
        x1 = w - (w // 3)
        crop = bgr[y0:, x0:x1]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask_a = cv2.inRange(hsv, self._lower_a, self._upper_a)
        mask_b = cv2.inRange(hsv, self._lower_b, self._upper_b)
        cv2.rectangle(debug, (x0, y0), (x1 - 1, h - 1), self._DBG_CROP, 1)
        self._draw_shifted_contours(debug, mask_a, x0, y0, self._DBG_COLOR_A, 2)
        self._draw_shifted_contours(debug, mask_b, x0, y0, self._DBG_COLOR_B, 2)
        crop_center_x = x0 + (x1 - x0) // 2
        # Tolerance band (green vertical lines).
        tol_left = int(crop_center_x - self.center_tol_px)
        tol_right = int(crop_center_x + self.center_tol_px)
        cv2.line(debug, (tol_left, y0), (tol_left, h - 1), self._DBG_TOL, 1)
        cv2.line(debug, (tol_right, y0), (tol_right, h - 1), self._DBG_TOL, 1)
        # Crop-center reference (white).
        cv2.line(
            debug,
            (crop_center_x, y0),
            (crop_center_x, h - 1),
            self._DBG_CENTER,
            1,
        )
        # Combined centroid (green if in tolerance, orange otherwise).
        if total >= self.center_min_pixels:
            centroid_x = int(crop_center_x + lateral_error_px)
            in_tol = abs(lateral_error_px) < self.center_tol_px
            color = self._DBG_OK if in_tol else self._DBG_WARN
            cv2.line(debug, (centroid_x, y0), (centroid_x, h - 1), color, 2)
        self._put_label(
            debug,
            f"TURN_TO_CENTER dir={self.direction}",
            22,
            (0, 255, 255),
            0.6,
            2,
        )
        self._put_label(
            debug,
            f"total={total}px lat={lateral_error_px:+.1f}px "
            f"tol=+/-{self.center_tol_px:.0f} dom={dominant}",
            44,
        )
        self._put_label(
            debug,
            f"stable={self._center_stable}/{self.center_stable_frames} "
            f"turned={math.degrees(self._turn_to_center_rad):.1f}deg",
            62,
        )
        self._publish_annotated(debug)

    def _annotate_line_follow(
        self,
        bgr: np.ndarray,
        current: str,
        cur_count: int,
        cur_lateral_px: float,
        other: str,
        other_count: int,
        other_lateral_px: float,
        transitioned: bool,
        angular: float,
    ) -> None:
        if self._annotated_pub is None or bgr is None:
            return
        debug = bgr.copy()
        h, w = bgr.shape[:2]
        y0 = int(h * (1.0 - self.line_follow_strip_frac))
        strip = bgr[y0:, :]
        hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
        mask_a = cv2.inRange(hsv, self._lower_a, self._upper_a)
        mask_b = cv2.inRange(hsv, self._lower_b, self._upper_b)
        mask_current = mask_a if current == "A" else mask_b
        mask_other = mask_b if current == "A" else mask_a
        cv2.rectangle(debug, (0, y0), (w - 1, h - 1), self._DBG_CROP, 1)
        # Current colour is drawn thick (what we're steering on); the other
        # colour thin (what we're watching for a transition).
        self._draw_shifted_contours(
            debug, mask_current, 0, y0, self._DBG_OK, 3
        )
        self._draw_shifted_contours(
            debug, mask_other, 0, y0, self._DBG_WARN, 1
        )
        strip_center_x = w // 2
        cv2.line(
            debug,
            (strip_center_x, y0),
            (strip_center_x, h - 1),
            self._DBG_CENTER,
            1,
        )
        if cur_count >= self.line_follow_min_pixels:
            cur_centroid_x = int(strip_center_x + cur_lateral_px)
            cv2.line(
                debug,
                (cur_centroid_x, y0),
                (cur_centroid_x, h - 1),
                self._DBG_OK,
                2,
            )
        if other_count >= self.line_follow_min_pixels:
            other_centroid_x = int(strip_center_x + other_lateral_px)
            cv2.line(
                debug,
                (other_centroid_x, y0),
                (other_centroid_x, h - 1),
                self._DBG_WARN,
                1,
            )
        corner_tag = "CORNER" if self._corner_color_seen else "follow"
        trans_tag = " TRANS" if transitioned else ""
        self._put_label(
            debug,
            f"LINE_FOLLOW dir={self.direction} cur={current} [{corner_tag}]{trans_tag}",
            22,
            (0, 255, 255),
            0.6,
            2,
        )
        self._put_label(
            debug,
            f"{current}={cur_count}px(lat={cur_lateral_px:+.1f}) "
            f"{other}={other_count}px(lat={other_lateral_px:+.1f})",
            44,
        )
        self._put_label(
            debug,
            f"min={self.line_follow_min_pixels} "
            f"ang={angular:+.2f}rad/s v={self.line_follow_speed_mps:.2f}m/s",
            62,
        )
        self._publish_annotated(debug)

    def _annotate_tape_align(
        self,
        bgr: np.ndarray,
        count: int,
        lateral_px: float,
        status: str,
    ) -> None:
        if self._annotated_pub is None or bgr is None:
            return
        debug = bgr.copy()
        h, w = bgr.shape[:2]
        # Exactly match _black_centroid_metrics: bottom line_follow_strip_frac,
        # full width, same HSV thresholds, same center computation.
        y0 = int(h * (1.0 - self.line_follow_strip_frac))
        strip = bgr[y0:, :]
        hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower_black, self._upper_black)
        cv2.rectangle(debug, (0, y0), (w - 1, h - 1), self._DBG_CROP, 1)
        self._draw_shifted_contours(debug, mask, 0, y0, (180, 180, 180), 2)
        # Use float / 2.0 to match _black_centroid_metrics exactly.
        strip_center_x = strip.shape[1] / 2.0
        # Tolerance band (green vertical lines).
        tol_left = int(strip_center_x - self.tape_align_center_tol_px)
        tol_right = int(strip_center_x + self.tape_align_center_tol_px)
        cv2.line(debug, (tol_left, y0), (tol_left, h - 1), self._DBG_TOL, 1)
        cv2.line(debug, (tol_right, y0), (tol_right, h - 1), self._DBG_TOL, 1)
        # Strip-center reference (white).
        cv2.line(
            debug,
            (int(strip_center_x), y0),
            (int(strip_center_x), h - 1),
            self._DBG_CENTER,
            1,
        )
        # Black-tape centroid (green if in tolerance, orange otherwise).
        # Reconstruct the centroid position the same way the algorithm does:
        # centroid_x_in_strip = xs.mean(), lateral_px = centroid_x_in_strip - strip_center_x
        # so centroid_x_in_strip = strip_center_x + lateral_px, drawn at x offset 0.
        if count >= self.tape_align_min_pixels:
            centroid_x = int(strip_center_x + lateral_px)
            in_tol = abs(lateral_px) <= self.tape_align_center_tol_px
            color = self._DBG_OK if in_tol else self._DBG_WARN
            cv2.line(debug, (centroid_x, y0), (centroid_x, h - 1), color, 2)
        self._put_label(
            debug,
            f"TAPE_ALIGN [{status}] dir={self.direction}",
            22,
            (0, 255, 255),
            0.6,
            2,
        )
        self._put_label(
            debug,
            f"black={count}px lat={lateral_px:+.1f}px "
            f"tol=+/-{self.tape_align_center_tol_px:.0f} "
            f"min={self.tape_align_min_pixels}",
            44,
        )
        self._put_label(
            debug,
            f"stable={self._tape_align_stable}/{self.tape_align_stable_frames}",
            62,
        )
        self._publish_annotated(debug)
