from __future__ import annotations

import math
from typing import Literal, Optional, Tuple, override

from typing_extensions import TypedDict

import cv2
import numpy as np
from cv_bridge import CvBridge
from pydantic import BaseModel
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from vehicle_common.cv.dlz_convex_hull import build_dlz_hull_mask
from payload.payload import Payload
from uav.vision_nodes import PayloadAprilTagNode
from uav.vision_nodes.payload_perception_common import DEFAULT_TAG_FAMILY
from uav_interfaces.srv import PayloadAprilTagState

from vehicle_common.mode import Mode
from vehicle_common.mode_loader import register_mode

# Corner-turn vision thresholds (mirrors PayloadCornerNavigateMode defaults)
_CORNER_CENTER_TOL_PX = 75.0  # lateral error tolerance (px)
_CORNER_CENTER_MIN_PX = 400  # minimum target-colour pixels to trust centering
_CORNER_STABLE_FRAMES = 2  # consecutive centred frames before locking
_CORNER_MAX_RAD = 2.0 * math.pi  # safety timeout (radians)
# Pause held after detecting a corner transition before starting to rotate,
# so the vehicle fully stops and the camera de-blurs before vision centering.
_CORNER_PRE_TURN_WAIT_S = 0.0

# TURN_ONTO_TAPE vision thresholds
_TURN_CENTER_TOL_PX = 50.0  # lateral error tolerance (px)
_TURN_CENTER_MIN_PX = 150  # minimum tape pixels to trust centering
_TURN_STABLE_FRAMES = 2  # consecutive centred frames before locking
_TURN_MAX_RAD = 2.0 * math.pi  # safety timeout (radians)

_VALID_START_PHASES = ("wait_for_plane", "scan_tags", "line_follow")

# ---------------------------------------------------------------------------
# Inline colour detection. HSV bounds are now constructor parameters (see
# ccw_*_hsv / cw_*_hsv) so the same pink/red/blue calibrations as
# PayloadCornerNavigateMode can be reused.
# ---------------------------------------------------------------------------
# Internal A/B labelling:
#   A = "cw-start colour" (the colour you'd land on when travelling CW)
#   B = "ccw-start colour" (the colour you'd land on when travelling CCW)
# So cw_*_hsv configures A and ccw_*_hsv configures B. This is the opposite
# of PayloadCornerNavigateMode's internal mapping, because DLZNavigate's
# corner transition is A→B for ccw while Corner's is B→A — the yaml-level
# ccw/cw colour still refers to the same physical tape in both modes.

# Fraction of the full frame height where the processing strip starts (bottom 4/9)
_STRIP_START_FRAC = 6 / 9
# Minimum tape pixels in the strip to trust the current_color reading
_MIN_COLOR_PIXELS = 20
# Ratio threshold: one color must be at least this many times more than the other
_COLOR_RATIO = 1.5
# Minimum tape pixels (A+B combined) per row to include in boundary estimate
_MIN_ROW_PIXELS = 3
_COLOR_BLUR_KERNEL = (5, 5)


def _get_strip(bgr: np.ndarray) -> Tuple[np.ndarray, int]:
    """Return the bottom-4/9 strip and its y-offset in the full frame."""
    height = bgr.shape[0]
    strip_start = int(height * _STRIP_START_FRAC)
    return bgr[strip_start:, :], strip_start


def _preprocess_color_crop(bgr: np.ndarray) -> np.ndarray:
    """Light denoising before HSV thresholding to stabilize tape masks."""
    return cv2.GaussianBlur(bgr, _COLOR_BLUR_KERNEL, 0)


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


class PayloadDLZNavigateParams(BaseModel):
    direction: Literal["cw", "ccw"] = "ccw"
    target_transitions: int = 1
    turn_angular_speed: float = 0.5
    corner_angular_speed: float = 0.3
    line_follow_speed_mps: float = 0.10
    k_lat: float = 0.003
    k_d_lat: float = 0.002
    k_ang: float = 0.4
    max_angular: float = 0.5
    # Scan / lookup params
    tag_transition_table: dict[str, TagTransitionRule] | None = None
    detect_frames: int = 5
    scan_duration_s: float = 1.0
    start_phase: Literal["wait_for_plane", "scan_tags", "line_follow"] = (
        "wait_for_plane"
    )
    tag_size_m: float = 0.0508
    tag_family: str = DEFAULT_TAG_FAMILY
    compressed_image: bool = False
    # HSV bounds — same semantics as PayloadCornerNavigateMode. Defaults
    # match that mode (ccw=red, cw=blue) so calibrations can be shared.
    ccw_lower_hsv: list[int] = (0, 80, 80)
    ccw_upper_hsv: list[int] = (10, 255, 255)
    cw_lower_hsv: list[int] = (85, 120, 60)
    cw_upper_hsv: list[int] = (140, 255, 255)
    cw_lower_hsv2: list[int] = (255, 255, 255)
    cw_upper_hsv2: list[int] = (255, 255, 255)


@register_mode(
    id="payload.PayloadDLZNavigateMode",
    targets=[Payload],
    required_vision_nodes=[PayloadAprilTagNode],
    transition_labels=["done"],
    requires_camera=True,
)
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
        Rotate in place until the combined red/blue tape is centred in the
        detection strip (vision-based; safety timeout at 2π rad):
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

    required_vision_nodes = (PayloadAprilTagNode,)
    transition_labels = ("done",)
    requires_camera = True

    @override
    def initialize(
        self, node: Node, vehicle: Payload, params: PayloadDLZNavigateParams
    ) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params
        direction = str(params.direction).lower().strip()
        if direction not in ("cw", "ccw"):
            raise ValueError(f"direction must be 'cw' or 'ccw', got {direction!r}")
        start_phase = str(params.start_phase).lower().strip()
        if start_phase not in _VALID_START_PHASES:
            raise ValueError(
                f"start_phase must be one of {_VALID_START_PHASES}, got {start_phase!r}"
            )

        # Fallback defaults (overwritten by table lookup at runtime)
        self._default_direction = direction
        self._default_target_transitions = int(params.target_transitions)

        self.tag_transition_table = dict(params.tag_transition_table or {})
        self.start_phase = start_phase
        self.tag_family = (
            str(params.tag_family) if params.tag_family else DEFAULT_TAG_FAMILY
        )

        # A = cw-start colour, B = ccw-start colour (see module docstring).
        self._lower_a = np.array(params.cw_lower_hsv, dtype=np.uint8)
        self._upper_a = np.array(params.cw_upper_hsv, dtype=np.uint8)
        self._lower_a2 = np.array(params.cw_lower_hsv2, dtype=np.uint8)
        self._upper_a2 = np.array(params.cw_upper_hsv2, dtype=np.uint8)
        self._lower_b = np.array(params.ccw_lower_hsv, dtype=np.uint8)
        self._upper_b = np.array(params.ccw_upper_hsv, dtype=np.uint8)

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
            self.p.turn_angular_speed
            if self.direction == "cw"
            else -self.p.turn_angular_speed
        )

    def _corner_transition(self, prev: str, curr: str) -> bool:
        """True when the A↔B transition is a corner for the current travel direction."""
        if self.direction == "ccw":
            return prev == "A" and curr == "B"
        else:  # cw
            return prev == "B" and curr == "A"

    def _request_apriltag_state(self) -> Optional[PayloadAprilTagState.Response]:
        req = PayloadAprilTagState.Request()
        req.tag_size_m = self.p.tag_size_m
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
            if self.p.compressed_image:
                buf = np.frombuffer(self._image.data, dtype=np.uint8)
                return cv2.imdecode(buf, cv2.IMREAD_COLOR)
            return self._bridge.imgmsg_to_cv2(self._image, desired_encoding="bgr8")
        except Exception as exc:
            self.node.get_logger().warn(
                f"PayloadDLZNavigateMode: image decode failed: {exc}"
            )
            return None

    def _dlz_roi_mask(self, bgr: np.ndarray) -> np.ndarray:
        roi_mask = build_dlz_hull_mask(bgr)
        if roi_mask is None:
            return np.full(bgr.shape[:2], 255, dtype=np.uint8)
        return roi_mask

    def _threshold_color_mask(
        self,
        hsv: np.ndarray,
        roi_mask: np.ndarray,
        *,
        color: str,
    ) -> np.ndarray:
        if color == "A":
            mask = cv2.inRange(hsv, self._lower_a, self._upper_a) | cv2.inRange(
                hsv, self._lower_a2, self._upper_a2
            )
        else:
            mask = cv2.inRange(hsv, self._lower_b, self._upper_b)
        return cv2.bitwise_and(mask, roi_mask)

    def _darken_outside_roi(self, debug: np.ndarray, roi_mask: np.ndarray) -> None:
        outside = roi_mask == 0
        if np.any(outside):
            debug[outside] = (debug[outside] * 0.35).astype(np.uint8)

    def _detect_color(
        self, bgr: np.ndarray
    ) -> Tuple[str, bool, float, float, np.ndarray, np.ndarray, np.ndarray, int]:
        """Run the full colour detection pipeline on a BGR frame.

        Returns (current_color, boundary_detected, lateral_error_px,
                 boundary_angle, orange_mask, blue_mask, strip, strip_start).
        """
        roi_full = self._dlz_roi_mask(bgr)
        strip, strip_start = _get_strip(bgr)
        roi_strip = roi_full[strip_start:, :]
        hsv = cv2.cvtColor(_preprocess_color_crop(strip), cv2.COLOR_BGR2HSV)
        orange_mask = self._threshold_color_mask(hsv, roi_strip, color="A")
        blue_mask = self._threshold_color_mask(hsv, roi_strip, color="B")
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
        self._turn_stable = 0

        # LINE_FOLLOW state
        self._prev_color = "A" if self.direction == "cw" else "B"
        self._prev_lateral_error: Optional[float] = None
        self._transitions = 0
        self._lf_phase = "following"  # "following" | "corner_turn"
        self._corner_wait_elapsed = 0.0
        self._corner_turned = 0.0
        self._corner_stable = 0
        self._corner_target_color: str = "none"
        self._done = False
        self._image = None

        # Camera subscription for inline colour detection
        cam_topic = self.vehicle.namespaced_path("camera")
        if self.p.compressed_image:
            self._image_sub = self.node.create_subscription(
                CompressedImage, f"{cam_topic}/compressed", self._image_cb, 1
            )
        else:
            self._image_sub = self.node.create_subscription(
                Image, cam_topic, self._image_cb, 1
            )

        if getattr(self.node, "vision_debug", False):
            annotated_topic = self.vehicle.namespaced_path("annotated_image/compressed")
            self._annotated_pub = self.node.create_publisher(
                CompressedImage, annotated_topic, 1
            )

        self.log(
            f"PayloadDLZNavigateMode: enter  start_phase={self.start_phase}  "
            f"direction={self.direction}  target_transitions={self.target_transitions}  "
            f"camera={cam_topic} vision_debug={getattr(self.node, 'vision_debug', False)}"
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
                f"consecutive={self._consecutive_tag_frames}/{self.p.detect_frames}"
            )
            if self._consecutive_tag_frames >= self.p.detect_frames:
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

        if self._scan_elapsed < self.p.scan_duration_s:
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

    def _middle_third_single_color_metrics(
        self, bgr: np.ndarray, color: str
    ) -> Tuple[int, float, np.ndarray, int, int]:
        """Middle-third × bottom-40% crop, masked on a single expected colour.

        Returns ``(pixel_count, lateral_error_px, mask, row_start, col_start)``.
        ``lateral_error_px`` is the mask centroid x minus crop center x
        (positive = right). Only the expected colour is computed — the
        opposite-side tape cannot bias the centroid.
        """
        h, w = bgr.shape[:2]
        roi_full = self._dlz_roi_mask(bgr)
        row_start = int(h * 0.6)
        # col_start = w // 3
        col_start = 0
        # col_end = w - (w // 3)
        col_end = w
        crop = bgr[row_start:, col_start:col_end]
        roi_crop = roi_full[row_start:, col_start:col_end]
        hsv = cv2.cvtColor(_preprocess_color_crop(crop), cv2.COLOR_BGR2HSV)
        mask = self._threshold_color_mask(hsv, roi_crop, color=color)
        count = int(np.count_nonzero(mask))
        if count == 0:
            return 0, 0.0, mask, row_start, col_start
        _ys, xs = np.nonzero(mask)
        centroid_x = float(xs.mean())
        crop_center_x = mask.shape[1] / 2.0
        return count, centroid_x - crop_center_x, mask, row_start, col_start

    def _update_turn_onto_tape(self, time_delta: float) -> None:
        # Rotate in place until the expected-colour tape is centred in the
        # middle-third of the frame. Only the colour we're about to follow is
        # masked — the opposite-side tape cannot pull the centroid.
        #   direction="cw"  → turn left  (+), expect colour A (red)
        #   direction="ccw" → turn right (-), expect colour B (blue)
        angular = self._turn_angular()
        self._angle_turned += abs(angular) * time_delta
        expected_color = "A" if self.direction == "cw" else "B"

        bgr = self._decode_image()
        count = 0
        lateral_error_px = 0.0
        if bgr is not None:
            (
                count,
                lateral_error_px,
                mask,
                row_start,
                col_start,
            ) = self._middle_third_single_color_metrics(bgr, expected_color)

            # Stop condition depends on travel direction (lenient variant —
            # mirrors the corner-turn termination):
            #   cw  → centroid should be on the LEFT side of the crop
            #         (lateral_error_px < _TURN_CENTER_TOL_PX)
            #   ccw → centroid should be on the RIGHT side of the crop
            #         (lateral_error_px > -_TURN_CENTER_TOL_PX)
            if self.direction == "cw":
                side_ok = lateral_error_px > -_TURN_CENTER_TOL_PX
            else:
                side_ok = lateral_error_px < _TURN_CENTER_TOL_PX
            centred = count >= _TURN_CENTER_MIN_PX and side_ok
            if centred:
                self._turn_stable += 1
                if self._turn_stable >= _TURN_STABLE_FRAMES:
                    self.vehicle.stop()
                    # LINE_FOLLOW's first transition is off the expected colour.
                    self._prev_color = expected_color
                    self._annotate_turn_onto_tape(
                        bgr,
                        row_start,
                        col_start,
                        mask,
                        expected_color,
                        count,
                        lateral_error_px,
                        angular,
                    )
                    self._phase = "line_follow"
                    self.log(
                        f"PayloadDLZNavigateMode: TURN_ONTO_TAPE centred on "
                        f"{expected_color} (lat={lateral_error_px:.1f}px, "
                        f"count={count}px, "
                        f"turned={math.degrees(self._angle_turned):.1f}°) "
                        f"→ LINE_FOLLOW"
                    )
                    return
            else:
                self._turn_stable = 0

            self._annotate_turn_onto_tape(
                bgr,
                row_start,
                col_start,
                mask,
                expected_color,
                count,
                lateral_error_px,
                angular,
            )

        if self._angle_turned >= _TURN_MAX_RAD:
            self.vehicle.stop()
            self._phase = "line_follow"
            self.log(
                f"PayloadDLZNavigateMode: TURN_ONTO_TAPE TIMEOUT "
                f"({math.degrees(self._angle_turned):.1f}°) → LINE_FOLLOW"
            )
            return

        self.vehicle.drive(0.0, angular)

    def _annotate_turn_onto_tape(
        self,
        bgr: np.ndarray,
        row_start: int,
        col_start: int,
        mask: np.ndarray,
        expected_color: str,
        count: int,
        lateral_error_px: float,
        angular: float,
    ) -> None:
        if self._annotated_pub is None or bgr is None:
            return
        debug = bgr.copy()
        h, w = bgr.shape[:2]
        roi_full = self._dlz_roi_mask(bgr)
        self._darken_outside_roi(debug, roi_full)

        # Contours of the single expected-colour mask — exactly what the
        # detector thresholded on for the centroid.
        shift = np.array([[[col_start, row_start]]])
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, [c + shift for c in contours], -1, (255, 255, 255), 2)

        # Crop boundary (top row, left+right columns) — middle-third bottom-40%.
        col_end = w - col_start
        cv2.line(debug, (0, row_start), (w, row_start), (80, 80, 80), 1)
        cv2.line(debug, (col_start, row_start), (col_start, h - 1), (80, 80, 80), 1)
        cv2.line(debug, (col_end, row_start), (col_end, h - 1), (80, 80, 80), 1)

        # Crop centre (white).
        crop_cx = col_start + (col_end - col_start) // 2
        cv2.line(debug, (crop_cx, row_start), (crop_cx, h - 1), (255, 255, 255), 1)

        # Accept region (direction-dependent, lenient):
        #   cw  → accept side = right; boundary at crop_cx - tol
        #   ccw → accept side = left; boundary at crop_cx + tol
        if self.direction == "cw":
            accept_boundary_x = int(crop_cx - _TURN_CENTER_TOL_PX)
            accept_x0, accept_x1 = accept_boundary_x, col_end
        else:
            accept_boundary_x = int(crop_cx + _TURN_CENTER_TOL_PX)
            accept_x0, accept_x1 = col_start, accept_boundary_x
        overlay = debug.copy()
        cv2.rectangle(
            overlay,
            (accept_x0, row_start),
            (accept_x1, h - 1),
            (0, 255, 0),
            thickness=-1,
        )
        cv2.addWeighted(overlay, 0.18, debug, 0.82, 0, dst=debug)
        cv2.line(
            debug,
            (accept_boundary_x, row_start),
            (accept_boundary_x, h - 1),
            (0, 255, 0),
            2,
        )

        # Single-colour centroid — matches the actual stop condition exactly.
        if count >= _TURN_CENTER_MIN_PX:
            centroid_x = int(crop_cx + lateral_error_px)
            if self.direction == "cw":
                in_accept = lateral_error_px > -_TURN_CENTER_TOL_PX
            else:
                in_accept = lateral_error_px < _TURN_CENTER_TOL_PX
            color = (0, 255, 0) if in_accept else (0, 140, 255)
            cv2.line(debug, (centroid_x, row_start), (centroid_x, h - 1), color, 2)

        cv2.putText(
            debug,
            f"TURN_ONTO_TAPE dir={self.direction} expect={expected_color}",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            debug,
            f"{expected_color}={count}px lat={lateral_error_px:+.1f}px "
            f"tol=+/-{_TURN_CENTER_TOL_PX:.0f}",
            (8, 44),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )
        cv2.putText(
            debug,
            f"stable={self._turn_stable}/{_TURN_STABLE_FRAMES} "
            f"turned={math.degrees(self._angle_turned):.1f}deg "
            f"cmd={angular:+.2f}rad/s",
            (8, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )
        self._publish_annotated(debug)

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
                self._corner_wait_elapsed = 0.0
                self._corner_turned = 0.0
                self._corner_stable = 0
                self._corner_target_color = self._prev_color
                self._prev_lateral_error = None
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

        # PD boundary following
        if boundary_detected:
            # D term: rate of change of lateral error. When the error is
            # shrinking (tape returning to centre), d_lateral opposes P and
            # brakes the correction before it overshoots — this is what
            # damps the oscillation cycle.
            if self._prev_lateral_error is not None and time_delta > 0:
                d_lateral = (lateral_error_px - self._prev_lateral_error) / time_delta
            else:
                d_lateral = 0.0
            self._prev_lateral_error = lateral_error_px

            angular = float(
                np.clip(
                    -self.p.k_lat * lateral_error_px
                    - self.p.k_d_lat * d_lateral
                    + self.p.k_ang * boundary_angle,
                    -self.p.max_angular,
                    self.p.max_angular,
                )
            )
        else:
            # No boundary — reset so stale derivative can't kick in next frame
            self._prev_lateral_error = None
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
        self.vehicle.drive(self.p.line_follow_speed_mps, angular)

    def _corner_single_color_metrics(
        self, bgr: np.ndarray, color: str
    ) -> Tuple[int, float, np.ndarray, int, int]:
        """Return ``(pixel_count, lateral_error_px, mask, row_start, col_start)``
        for a single colour in the bottom third of the frame, full frame width.
        Returns ``(0, 0.0, empty_mask, row_start, col_start)`` if no pixels
        found."""
        h, w = bgr.shape[:2]
        roi_full = self._dlz_roi_mask(bgr)
        # Bottom third: top at 2h/3, bottom at h.
        row_start = int(h * 2 / 3)
        row_end = h * 9 // 10
        col_start = 0
        col_end = w
        crop = bgr[row_start:row_end, col_start:col_end]
        roi_crop = roi_full[row_start:row_end, col_start:col_end]
        hsv = cv2.cvtColor(_preprocess_color_crop(crop), cv2.COLOR_BGR2HSV)
        mask = self._threshold_color_mask(hsv, roi_crop, color=color)
        count = int(np.count_nonzero(mask))
        if count == 0:
            return 0, 0.0, mask, row_start, col_start
        _ys, xs = np.nonzero(mask)
        centroid_x = float(xs.mean())
        crop_center_x = mask.shape[1] / 2.0
        return count, centroid_x - crop_center_x, mask, row_start, col_start

    def _do_corner_turn(self, time_delta: float) -> None:
        # Hold still briefly after the corner transition fires before starting
        # to rotate — lets the chassis settle and camera de-blur.
        if self._corner_wait_elapsed < _CORNER_PRE_TURN_WAIT_S:
            self._corner_wait_elapsed += time_delta
            self.vehicle.drive(0.0, 0.0)
            return

        # Corner turns:
        #   CCW travel → turn left (+), CW travel → turn right (-)
        speed = self.p.corner_angular_speed
        angular = speed if self.direction == "ccw" else -speed
        self._corner_turned += abs(angular) * time_delta

        bgr = self._decode_image()
        if bgr is not None:
            total, lateral_error_px, mask, row_start, col_start = (
                self._corner_single_color_metrics(bgr, self._corner_target_color)
            )
            # Stop condition depends on travel direction (lenient):
            #   cw  → accept when centroid on the LEFT side of the crop
            #         (lateral_error_px <  _CORNER_CENTER_TOL_PX)
            #   ccw → accept when centroid on the RIGHT side of the crop
            #         (lateral_error_px > -_CORNER_CENTER_TOL_PX)
            if self.direction == "cw":
                side_ok = lateral_error_px < _CORNER_CENTER_TOL_PX
            else:
                side_ok = lateral_error_px > -_CORNER_CENTER_TOL_PX
            if total >= _CORNER_CENTER_MIN_PX and side_ok:
                self._corner_stable += 1
                if self._corner_stable >= _CORNER_STABLE_FRAMES:
                    self._annotate_corner_turn(
                        bgr, mask, row_start, col_start, total, lateral_error_px
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
                bgr, mask, row_start, col_start, total, lateral_error_px
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
        col_start: int,
        total: int,
        lateral_error_px: float,
    ) -> None:
        if self._annotated_pub is None or bgr is None:
            return
        debug = bgr.copy()
        h, w = bgr.shape[:2]
        roi_full = self._dlz_roi_mask(bgr)
        self._darken_outside_roi(debug, roi_full)
        row_end = h * 9 // 10
        col_end = w - col_start

        # Darken the rows OUTSIDE the crop band so the active region is
        # visually unambiguous — proves the crop is the full width of the
        # band between row_start and row_end.
        if row_start > 0:
            debug[:row_start] = (debug[:row_start] * 0.4).astype(np.uint8)
        if row_end < h:
            debug[row_end:] = (debug[row_end:] * 0.4).astype(np.uint8)

        # Draw contours of the target colour mask (offset by the crop origin).
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        shifted = [c + np.array([[[col_start, row_start]]]) for c in contours]
        cv2.drawContours(debug, shifted, -1, (255, 255, 255), 2)

        # Crop boundary — bottom third of the frame, full width.
        cv2.line(debug, (0, row_start), (w, row_start), (80, 80, 80), 1)
        cv2.line(debug, (0, row_end), (w, row_end), (80, 80, 80), 1)

        # Crop centre reference (white) — the centroid is measured relative
        # to this column, not the full frame centre.
        crop_cx = col_start + (col_end - col_start) // 2
        cv2.line(debug, (crop_cx, row_start), (crop_cx, row_end), (255, 255, 255), 1)

        # Accept region (direction-dependent, lenient):
        #   cw  → accept side = left; boundary line at crop_cx + tol
        #   ccw → accept side = right; boundary line at crop_cx - tol
        if self.direction == "cw":
            accept_boundary_x = int(crop_cx + _CORNER_CENTER_TOL_PX)
            accept_x0, accept_x1 = 0, accept_boundary_x
        else:
            accept_boundary_x = int(crop_cx - _CORNER_CENTER_TOL_PX)
            accept_x0, accept_x1 = accept_boundary_x, w
        overlay = debug.copy()
        cv2.rectangle(
            overlay,
            (accept_x0, row_start),
            (accept_x1, row_end),
            (0, 255, 0),
            thickness=-1,
        )
        cv2.addWeighted(overlay, 0.18, debug, 0.82, 0, dst=debug)
        cv2.line(
            debug,
            (accept_boundary_x, row_start),
            (accept_boundary_x, row_end),
            (0, 255, 0),
            2,
        )

        # Centroid line (green if in accept region, orange otherwise)
        if total >= _CORNER_CENTER_MIN_PX:
            centroid_x = int(crop_cx + lateral_error_px)
            if self.direction == "cw":
                in_accept = lateral_error_px < _CORNER_CENTER_TOL_PX
            else:
                in_accept = lateral_error_px > -_CORNER_CENTER_TOL_PX
            color = (0, 255, 0) if in_accept else (0, 140, 255)
            cv2.line(debug, (centroid_x, row_start), (centroid_x, row_end), color, 2)

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
        roi_full = self._dlz_roi_mask(bgr)
        self._darken_outside_roi(debug, roi_full)
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
                f"cmd={angular:+.2f}rad/s  v={self.p.line_follow_speed_mps:.2f}m/s"
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

    @classmethod
    @override
    def get_params_cls(cls) -> type[BaseModel]:
        return PayloadDLZNavigateParams
