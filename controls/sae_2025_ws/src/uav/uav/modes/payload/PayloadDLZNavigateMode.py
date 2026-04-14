from __future__ import annotations

import math
from typing import Literal, Optional, Tuple

from typing_extensions import TypedDict

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from uav.utils import blue, red
from uav.vehicles.Payload import Payload
from uav.vision_nodes import PayloadAprilTagNode
from uav.vision_nodes.payload_perception_common import DEFAULT_TAG_FAMILY
from uav_interfaces.srv import PayloadAprilTagState

from ..Mode import Mode

# Angle thresholds
_QUARTER_TURN = math.pi / 2.0  # 90 degrees
_EIGHTH_TURN = math.pi / 4.0  # 45 degrees
_ANGLE_TOL = 0.05  # radians, stop slightly early to avoid overshoot

# Corner-turn vision thresholds (mirrors PayloadCornerNavigateMode defaults)
_CORNER_CENTER_TOL_PX = 30.0  # lateral error tolerance (px)
_CORNER_CENTER_MIN_PX = 150  # minimum target-colour pixels to trust centering
_CORNER_STABLE_FRAMES = 1  # consecutive centred frames before locking
_CORNER_MAX_RAD = 2.0 * math.pi  # safety timeout (radians)

_VALID_START_PHASES = ("wait_for_plane", "scan_tags", "line_follow")

# ---------------------------------------------------------------------------
# Inline colour detection (mirrors PayloadColorSquareNode exactly)
# ---------------------------------------------------------------------------

# Color A = red, Color B = blue (matches dlz_alternating_border model)
# Red wraps around the HSV hue wheel so two ranges are required.
_LOWER_A1 = np.array(red[0][0], dtype=np.uint8)
_UPPER_A1 = np.array(red[0][1], dtype=np.uint8)
_LOWER_A2 = np.array(red[1][0], dtype=np.uint8)
_UPPER_A2 = np.array(red[1][1], dtype=np.uint8)
_LOWER_B = np.array(blue[0], dtype=np.uint8)
_UPPER_B = np.array(blue[1], dtype=np.uint8)

# Fraction of the full frame height where the processing strip starts (bottom 4/9)
_STRIP_START_FRAC = 6 / 9
# Minimum tape pixels in the strip to trust the current_color reading
_MIN_COLOR_PIXELS = 20
# Ratio threshold: one color must be at least this many times more than the other
_COLOR_RATIO = 1.5
# Minimum tape pixels (A+B combined) per row to include in boundary estimate
_MIN_ROW_PIXELS = 3


def _red_mask(hsv: np.ndarray) -> np.ndarray:
    """OR the two hue ranges required to detect red in OpenCV HSV."""
    return cv2.bitwise_or(
        cv2.inRange(hsv, _LOWER_A1, _UPPER_A1),
        cv2.inRange(hsv, _LOWER_A2, _UPPER_A2),
    )


def _get_strip(bgr: np.ndarray) -> Tuple[np.ndarray, int]:
    """Return the bottom-4/9 strip and its y-offset in the full frame."""
    height = bgr.shape[0]
    strip_start = int(height * _STRIP_START_FRAC)
    return bgr[strip_start:, :], strip_start


def _detect_current_color(
    orange_mask: np.ndarray,
    blue_mask: np.ndarray,
) -> str:
    """Determine dominant tape color. Returns "A" (red), "B" (blue), or "none"."""
    a_count = int(np.count_nonzero(orange_mask))
    b_count = int(np.count_nonzero(blue_mask))
    total = a_count + b_count
    if total < _MIN_COLOR_PIXELS:
        return "none"
    if a_count > b_count * _COLOR_RATIO:
        return "A"
    if b_count > a_count * _COLOR_RATIO:
        return "B"
    return "none"


def _detect_tape_following(
    orange_mask: np.ndarray,
    blue_mask: np.ndarray,
) -> Tuple[bool, float, float]:
    """Find tape centre and return (boundary_detected, lateral_error_px, boundary_angle)."""
    strip_height, strip_width = orange_mask.shape[:2]
    frame_cx = strip_width / 2.0
    tape_mask = cv2.bitwise_or(orange_mask, blue_mask)

    row_cx: list[tuple[int, float]] = []
    for row in range(strip_height):
        cols = np.where(tape_mask[row] > 0)[0]
        if len(cols) >= _MIN_ROW_PIXELS:
            row_cx.append((row, float(cols.mean())))

    if not row_cx:
        return False, 0.0, 0.0

    xs = [cx for _, cx in row_cx]
    lateral_error_px = float(np.mean(xs)) - frame_cx

    if len(row_cx) >= 2:
        rows = np.array([r for r, _ in row_cx], dtype=float)
        cols = np.array([cx for _, cx in row_cx], dtype=float)
        if rows.std() > 0:
            slope = float(np.polyfit(rows, cols, 1)[0])
            boundary_angle = math.atan2(slope, 1.0)
        else:
            boundary_angle = 0.0
    else:
        boundary_angle = 0.0

    return True, lateral_error_px, boundary_angle


class TagTransitionRule(TypedDict):
    transitions: int
    direction: Literal["cw", "ccw"]


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
    required_vision_nodes = (PayloadAprilTagNode,)
    transition_labels = ("done",)

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        direction: Literal["cw", "ccw"] = "ccw",
        target_transitions: int = 1,
        turn_angular_speed: float = 0.5,
        corner_angular_speed: float = 0.3,
        line_follow_speed_mps: float = 0.10,
        k_lat: float = 0.003,
        k_ang: float = 0.4,
        max_angular: float = 0.5,
        # Scan / lookup params
        tag_transition_table: dict[str, TagTransitionRule] | None = None,
        detect_frames: int = 5,
        scan_duration_s: float = 1.0,
        start_phase: Literal[
            "wait_for_plane", "scan_tags", "line_follow"
        ] = "wait_for_plane",
        tag_size_m: float = 0.0508,
        tag_family: str = DEFAULT_TAG_FAMILY,
        compressed_image: bool = False,
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
        self.corner_angular_speed = float(corner_angular_speed)
        self.line_follow_speed_mps = float(line_follow_speed_mps)
        self.k_lat = float(k_lat)
        self.k_ang = float(k_ang)
        self.max_angular = float(max_angular)

        self.tag_transition_table = dict(tag_transition_table or {})
        self.detect_frames = int(detect_frames)
        self.scan_duration_s = float(scan_duration_s)
        self.start_phase = start_phase
        self.tag_size_m = float(tag_size_m)
        self.tag_family = str(tag_family) if tag_family else DEFAULT_TAG_FAMILY
        self.compressed_image = bool(compressed_image)

        self._bridge = CvBridge()
        self._image_sub = None
        self._image: Optional[object] = None
        self._annotated_pub = None

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _turn_angular(self) -> float:
        """Signed angular speed for the initial tape-alignment turn.
        CW direction → turn left (positive). CCW direction → turn right (negative).
        """
        return (
            self.turn_angular_speed
            if self.direction == "cw"
            else -self.turn_angular_speed
        )

    def _corner_transition(self, prev: str, curr: str) -> bool:
        """True when the A↔B transition is a corner for the current travel direction."""
        if self.direction == "ccw":
            return prev == "A" and curr == "B"
        else:  # cw
            return prev == "B" and curr == "A"

    def _request_apriltag_state(self) -> Optional[PayloadAprilTagState.Response]:
        req = PayloadAprilTagState.Request()
        req.tag_size_m = self.tag_size_m
        req.tag_family = self.tag_family
        return self.send_request(PayloadAprilTagNode, req)

    # ------------------------------------------------------------------
    # Camera helpers
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
                f"PayloadDLZNavigateMode: image decode failed: {exc}"
            )
            return None

    def _detect_color(
        self, bgr: np.ndarray
    ) -> Tuple[str, bool, float, float, np.ndarray, np.ndarray, np.ndarray, int]:
        """Run the full colour detection pipeline on a BGR frame.

        Returns (current_color, boundary_detected, lateral_error_px,
                 boundary_angle, orange_mask, blue_mask, strip, strip_start).
        """
        strip, strip_start = _get_strip(bgr)
        hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
        orange_mask = _red_mask(hsv)
        blue_mask = cv2.inRange(hsv, _LOWER_B, _UPPER_B)
        current_color = _detect_current_color(orange_mask, blue_mask)
        detected, lateral_error_px, boundary_angle = _detect_tape_following(
            orange_mask, blue_mask
        )
        return (
            current_color,
            detected,
            lateral_error_px,
            boundary_angle,
            orange_mask,
            blue_mask,
            strip,
            strip_start,
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
                f"PayloadDLZNavigateMode: annotated publish failed: {exc}"
            )

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
        self._lf_phase = "following"  # "following" | "corner_turn"
        self._corner_turned = 0.0
        self._corner_stable = 0
        self._corner_target_color: str = "none"
        self._done = False
        self._image = None

        # Camera subscription for inline colour detection
        cam_topic = self.vehicle.namespaced_path("camera")
        if self.compressed_image:
            self._image_sub = self.node.create_subscription(
                CompressedImage, f"{cam_topic}/compressed", self._image_cb, 1
            )
        else:
            self._image_sub = self.node.create_subscription(
                Image, cam_topic, self._image_cb, 1
            )

        annotated_topic = self.vehicle.namespaced_path("annotated_image/compressed")
        self._annotated_pub = self.node.create_publisher(
            CompressedImage, annotated_topic, 1
        )

        self.log(
            f"PayloadDLZNavigateMode: enter  start_phase={self.start_phase}  "
            f"direction={self.direction}  target_transitions={self.target_transitions}  "
            f"camera={cam_topic} annotated={annotated_topic}"
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
        if self._image_sub is not None:
            self.node.destroy_subscription(self._image_sub)
            self._image_sub = None
        if self._annotated_pub is not None:
            self.node.destroy_publisher(self._annotated_pub)
            self._annotated_pub = None
        # Expose the resolved travel direction so PayloadScanForTagMode can
        # spin in the same direction as the DLZ navigation just travelled.
        self.node.dlz_navigate_direction = self.direction

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
            self.log(
                "PayloadDLZNavigateMode: WAIT_FOR_PLANE — no tags visible, resetting counter"
            )
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
            self.log(
                "PayloadDLZNavigateMode: target_transitions=0 → already at dock → done"
            )
            return

        # start_phase=="scan_tags" means we're already on the tape → skip the alignment turn
        next_phase = (
            "line_follow" if self.start_phase == "scan_tags" else "turn_onto_tape"
        )
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

        bgr = self._decode_image()
        if bgr is None:
            self.vehicle.drive(0.0, 0.0)
            return

        # Inline colour detection
        (
            curr,
            boundary_detected,
            lateral_error_px,
            boundary_angle,
            orange_mask,
            blue_mask,
            strip,
            strip_start,
        ) = self._detect_color(bgr)

        transitioned = False
        is_corner = False
        if curr in ("A", "B") and curr != self._prev_color:
            transitioned = True
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
                self._annotate_line_follow(
                    bgr,
                    strip_start,
                    orange_mask,
                    blue_mask,
                    curr,
                    boundary_detected,
                    lateral_error_px,
                    boundary_angle,
                    0.0,
                    transitioned,
                    is_corner,
                )
                self._done = True
                self.log("PayloadDLZNavigateMode: target_transitions reached → done")
                return

            if is_corner:
                self._lf_phase = "corner_turn"
                self._corner_turned = 0.0
                self._corner_stable = 0
                self._corner_target_color = self._prev_color
                self._annotate_line_follow(
                    bgr,
                    strip_start,
                    orange_mask,
                    blue_mask,
                    curr,
                    boundary_detected,
                    lateral_error_px,
                    boundary_angle,
                    0.0,
                    transitioned,
                    is_corner,
                )
                self.vehicle.drive(0.0, 0.0)
                return

        # Boundary following
        if boundary_detected:
            angular = float(
                np.clip(
                    -self.k_lat * lateral_error_px + self.k_ang * boundary_angle,
                    -self.max_angular,
                    self.max_angular,
                )
            )
        else:
            angular = 0.0

        self._annotate_line_follow(
            bgr,
            strip_start,
            orange_mask,
            blue_mask,
            curr,
            boundary_detected,
            lateral_error_px,
            boundary_angle,
            angular,
            transitioned,
            is_corner,
        )
        self.vehicle.drive(self.line_follow_speed_mps, angular)

    def _corner_single_color_metrics(
        self, bgr: np.ndarray, color: str
    ) -> Tuple[int, float, np.ndarray, int]:
        """Return ``(pixel_count, lateral_error_px, mask, row_start)`` for a
        single colour in the bottom 40 % of the frame (full width).
        ``mask`` and ``row_start`` are returned for debug annotation.
        Returns ``(0, 0.0, empty_mask, row_start)`` if no pixels found."""
        h, w = bgr.shape[:2]
        row_start = int(h * 0.5)
        crop = bgr[row_start:]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        if color == "A":
            mask = _red_mask(hsv)
        else:
            mask = cv2.inRange(hsv, _LOWER_B, _UPPER_B)
        count = int(np.count_nonzero(mask))
        if count == 0:
            return 0, 0.0, mask, row_start
        _ys, xs = np.nonzero(mask)
        centroid_x = float(xs.mean())
        crop_center_x = mask.shape[1] / 2.0
        return count, centroid_x - crop_center_x, mask, row_start

    def _do_corner_turn(self, time_delta: float) -> None:
        # Corner turns are opposite to the initial alignment turn:
        #   CCW travel → turn left (+), CW travel → turn right (-)
        speed = self.corner_angular_speed
        angular = speed if self.direction == "ccw" else -speed
        self._corner_turned += abs(angular) * time_delta

        bgr = self._decode_image()
        if bgr is not None:
            total, lateral_error_px, mask, row_start = (
                self._corner_single_color_metrics(bgr, self._corner_target_color)
            )
            if (
                total >= _CORNER_CENTER_MIN_PX
                and abs(lateral_error_px) < _CORNER_CENTER_TOL_PX
            ):
                self._corner_stable += 1
                if self._corner_stable >= _CORNER_STABLE_FRAMES:
                    self._annotate_corner_turn(
                        bgr, mask, row_start, total, lateral_error_px
                    )
                    self.vehicle.drive(0.0, 0.0)
                    self._lf_phase = "following"
                    self.log(
                        f"PayloadDLZNavigateMode: corner turn centred on "
                        f"{self._corner_target_color} "
                        f"(lat={lateral_error_px:.1f}px, total={total}) "
                        f"→ FOLLOWING"
                    )
                    return
            else:
                self._corner_stable = 0

            self._annotate_corner_turn(
                bgr, mask, row_start, total, lateral_error_px
            )

        # Safety timeout
        if self._corner_turned >= _CORNER_MAX_RAD:
            self.vehicle.drive(0.0, 0.0)
            self._lf_phase = "following"
            self.log(
                f"PayloadDLZNavigateMode: corner turn TIMEOUT "
                f"({math.degrees(self._corner_turned):.1f}°) → FOLLOWING"
            )
            return

        self.vehicle.drive(0.0, angular)

    def _annotate_corner_turn(
        self,
        bgr: np.ndarray,
        mask: np.ndarray,
        row_start: int,
        total: int,
        lateral_error_px: float,
    ) -> None:
        if self._annotated_pub is None or bgr is None:
            return
        debug = bgr.copy()
        h, w = bgr.shape[:2]

        # Draw contours of the target colour mask
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        shifted = [c + np.array([[[0, row_start]]]) for c in contours]
        cv2.drawContours(debug, shifted, -1, (255, 255, 255), 2)

        # Crop boundary line
        cv2.line(debug, (0, row_start), (w, row_start), (80, 80, 80), 1)

        # Frame centre reference (white)
        frame_cx = w // 2
        cv2.line(debug, (frame_cx, row_start), (frame_cx, h), (255, 255, 255), 1)

        # Tolerance band (green)
        tol_left = int(frame_cx - _CORNER_CENTER_TOL_PX)
        tol_right = int(frame_cx + _CORNER_CENTER_TOL_PX)
        cv2.line(debug, (tol_left, row_start), (tol_left, h), (0, 255, 0), 1)
        cv2.line(debug, (tol_right, row_start), (tol_right, h), (0, 255, 0), 1)

        # Centroid line (green if in tolerance, orange otherwise)
        if total >= _CORNER_CENTER_MIN_PX:
            centroid_x = int(frame_cx + lateral_error_px)
            in_tol = abs(lateral_error_px) < _CORNER_CENTER_TOL_PX
            color = (0, 255, 0) if in_tol else (0, 140, 255)
            cv2.line(debug, (centroid_x, row_start), (centroid_x, h), color, 2)

        # Text overlay
        cv2.putText(
            debug,
            f"CORNER_TURN dir={self.direction} target={self._corner_target_color}",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            debug,
            f"total={total}px lat={lateral_error_px:+.1f}px "
            f"tol=+/-{_CORNER_CENTER_TOL_PX:.0f}",
            (8, 44),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )
        cv2.putText(
            debug,
            f"stable={self._corner_stable}/{_CORNER_STABLE_FRAMES} "
            f"turned={math.degrees(self._corner_turned):.1f}deg",
            (8, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )
        self._publish_annotated(debug)

    # ------------------------------------------------------------------
    # Annotated debug image
    # ------------------------------------------------------------------

    def _annotate_line_follow(
        self,
        bgr: np.ndarray,
        strip_start: int,
        orange_mask: np.ndarray,
        blue_mask: np.ndarray,
        current_color: str,
        boundary_detected: bool,
        lateral_error_px: float,
        boundary_angle: float,
        angular: float,
        transitioned: bool,
        is_corner: bool,
    ) -> None:
        if self._annotated_pub is None or bgr is None:
            return
        debug = bgr.copy()
        h, w = bgr.shape[:2]
        strip_w = w

        # Draw contours around detected colour regions — exactly what the
        # algorithm thresholded on.
        orange_contours, _ = cv2.findContours(
            orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        shifted_orange = [c + np.array([[[0, strip_start]]]) for c in orange_contours]
        cv2.drawContours(debug, shifted_orange, -1, (255, 255, 255), 2)

        blue_contours, _ = cv2.findContours(
            blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        shifted_blue = [c + np.array([[[0, strip_start]]]) for c in blue_contours]
        cv2.drawContours(debug, shifted_blue, -1, (255, 255, 255), 2)

        # Strip boundary line
        cv2.line(debug, (0, strip_start), (w, strip_start), (80, 80, 80), 1)

        # Frame centre reference (white) and tape centre (green) — exactly
        # the lateral_error_px the steering uses.
        frame_cx = strip_w // 2
        cv2.line(debug, (frame_cx, strip_start), (frame_cx, h), (255, 255, 255), 1)
        if boundary_detected:
            tape_cx = int(frame_cx + lateral_error_px)
            cv2.line(debug, (tape_cx, strip_start), (tape_cx, h), (0, 255, 0), 2)

        # Pixel counts (same values the algorithm used for dominance)
        a_count = int(np.count_nonzero(orange_mask))
        b_count = int(np.count_nonzero(blue_mask))

        trans_tag = ""
        if transitioned:
            trans_tag = " CORNER" if is_corner else " mid-side"

        cv2.putText(
            debug,
            f"LINE_FOLLOW dir={self.direction} color={current_color}{trans_tag}",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            debug,
            f"A={a_count}px B={b_count}px  trans={self._transitions}/{self.target_transitions}",
            (8, 44),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )
        if boundary_detected:
            err_label = (
                f"lat={lateral_error_px:.1f}px  "
                f"ang={math.degrees(boundary_angle):.1f}deg  "
                f"cmd={angular:+.2f}rad/s  v={self.line_follow_speed_mps:.2f}m/s"
            )
        else:
            err_label = "boundary=none"
        cv2.putText(
            debug,
            err_label,
            (8, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )
        self._publish_annotated(debug)
