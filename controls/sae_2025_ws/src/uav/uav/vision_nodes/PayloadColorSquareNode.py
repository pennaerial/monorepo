from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CompressedImage

from uav.utils import blue, red
from uav_interfaces.srv import PayloadColorSquareState

from .VisionNode import VisionNode

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


def _detect_tape_following(
    orange_mask: np.ndarray,
    blue_mask: np.ndarray,
) -> tuple[bool, float, float]:
    """
    Find the center of the combined tape strip (orange + blue) in the processing
    strip and return steering metrics.

    Returns:
        boundary_detected: True if tape is visible
        lateral_error_px:  tape centre x minus frame centre x (positive = tape
                           is to the right, steer right to re-centre)
        boundary_angle:    tilt of the tape's centre line in the frame (radians
                           from vertical; positive = leans right at the bottom)
    """
    strip_height, strip_width = orange_mask.shape[:2]
    frame_cx = strip_width / 2.0
    tape_mask = cv2.bitwise_or(orange_mask, blue_mask)

    # For each row find the mean x of tape pixels
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


def _detect_current_color(
    orange_mask: np.ndarray,
    blue_mask: np.ndarray,
) -> str:
    """
    Determine dominant tape color across the full processing strip.
    Returns "A" (orange), "B" (blue), or "none".
    """
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


def _red_mask(hsv: np.ndarray) -> np.ndarray:
    """OR the two hue ranges required to detect red in OpenCV HSV."""
    return cv2.bitwise_or(
        cv2.inRange(hsv, _LOWER_A1, _UPPER_A1),
        cv2.inRange(hsv, _LOWER_A2, _UPPER_A2),
    )


def _get_strip(bgr: np.ndarray) -> tuple[np.ndarray, int]:
    """Return the bottom-1/3 strip and its y-offset in the full frame."""
    height = bgr.shape[0]
    strip_start = int(height * _STRIP_START_FRAC)
    return bgr[strip_start:, :], strip_start


class PayloadColorSquareNode(VisionNode):
    srv = PayloadColorSquareState

    def __init__(self) -> None:
        super().__init__(self.__class__.srv, node_name=self.node_name())
        self.create_service(
            self.srv,
            self.vision_service,
            self.service_callback,
        )
        self._debug_pub: CompressedImage | None = None
        if self.debug:
            debug_topic = self.vision_service + "/debug_image/compressed"
            self._debug_pub = self.create_publisher(CompressedImage, debug_topic, 1)
            self.get_logger().info(f"Debug image publishing on: {debug_topic}")
            self.create_timer(1.0 / 30.0, self._debug_timer_callback)

    def service_callback(
        self,
        request: PayloadColorSquareState.Request,
        response: PayloadColorSquareState.Response,
    ) -> PayloadColorSquareState.Response:
        image_msg, _ = self.request_data(cam_image=True)
        response.has_image = image_msg is not None
        if image_msg is None:
            return response

        bgr = self.convert_image_msg_to_frame(image_msg)
        strip, _ = _get_strip(bgr)
        hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)

        orange_mask = _red_mask(hsv)
        blue_mask = cv2.inRange(hsv, _LOWER_B, _UPPER_B)

        response.current_color = _detect_current_color(orange_mask, blue_mask)
        detected, lateral_error_px, boundary_angle = _detect_tape_following(
            orange_mask, blue_mask
        )
        response.boundary_detected = detected
        response.lateral_error_px = lateral_error_px
        response.boundary_angle = boundary_angle

        return response

    def _debug_timer_callback(self) -> None:
        image_msg = self.image
        if image_msg is None:
            return
        bgr = self.convert_image_msg_to_frame(image_msg)
        strip, strip_start = _get_strip(bgr)
        hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
        orange_mask = _red_mask(hsv)
        blue_mask = cv2.inRange(hsv, _LOWER_B, _UPPER_B)
        current_color = _detect_current_color(orange_mask, blue_mask)
        detected, lateral_error_px, boundary_angle = _detect_tape_following(
            orange_mask, blue_mask
        )
        self.draw_debug(
            bgr,
            strip,
            strip_start,
            orange_mask,
            blue_mask,
            current_color,
            detected,
            lateral_error_px,
            boundary_angle,
        )

    def draw_debug(
        self,
        bgr: np.ndarray,
        strip: np.ndarray,
        strip_start: int,
        orange_mask: np.ndarray,
        blue_mask: np.ndarray,
        current_color: str,
        boundary_detected: bool,
        lateral_error_px: float,
        boundary_angle: float,
    ) -> None:
        debug = bgr.copy()
        _, strip_w = strip.shape[:2]

        # White contours around detected orange regions
        orange_contours, _ = cv2.findContours(
            orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        shifted_orange = [c + np.array([[[0, strip_start]]]) for c in orange_contours]
        cv2.drawContours(debug, shifted_orange, -1, (255, 255, 255), 2)

        # White contours around detected blue regions
        blue_contours, _ = cv2.findContours(
            blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        shifted_blue = [c + np.array([[[0, strip_start]]]) for c in blue_contours]
        cv2.drawContours(debug, shifted_blue, -1, (255, 255, 255), 2)

        # Faint grey line at the strip cut
        cv2.line(debug, (0, strip_start), (bgr.shape[1], strip_start), (80, 80, 80), 1)

        # Tape centre line and frame centre reference
        if boundary_detected:
            cx = int(strip_w / 2.0 + lateral_error_px)
            cv2.line(debug, (cx, strip_start), (cx, bgr.shape[0]), (0, 255, 0), 2)
            cv2.line(
                debug,
                (strip_w // 2, strip_start),
                (strip_w // 2, bgr.shape[0]),
                (255, 255, 255),
                1,
            )

        # Pixel counts (full strip)
        a_count = int(np.count_nonzero(orange_mask))
        b_count = int(np.count_nonzero(blue_mask))

        # Labels
        color_label = f"color={current_color}  A={a_count}px B={b_count}px"
        err_label = (
            f"lat={lateral_error_px:.1f}px  ang={math.degrees(boundary_angle):.1f}deg"
            if boundary_detected
            else "boundary=none"
        )
        cv2.putText(
            debug, color_label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
        )
        cv2.putText(
            debug,
            err_label,
            (8, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (200, 200, 200),
            1,
        )

        if self._debug_pub is not None:
            msg = self.bridge.cv2_to_compressed_imgmsg(debug, dst_format="jpeg")
            msg.header.stamp = self.get_clock().now().to_msg()
            self._debug_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = PayloadColorSquareNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
