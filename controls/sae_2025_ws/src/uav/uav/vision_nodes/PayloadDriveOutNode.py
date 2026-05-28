"""
PayloadDriveOutNode — detects whether the path ahead is clear for the payload to drive out.

Uses detect_payload_unreeled() to measure the white floor ratio in the bottom 25% of the frame.
A high white_ratio means the ramp/floor is visible and clear to drive out.
"""

from __future__ import annotations

from collections.abc import Sequence

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CompressedImage

from uav_interfaces.srv import PayloadDriveOutState

from .VisionNode import VisionNode
from .payload_perception_common import detect_payload_unreeled


def _request_hsv_or_default(
    values: Sequence[int] | np.ndarray,
    default: tuple[int, int, int],
) -> tuple[int, int, int]:
    hsv_values = np.asarray(values).reshape(-1)
    if hsv_values.size != 3:
        raise ValueError(f"HSV bounds must contain exactly 3 values, got {values!r}.")
    hsv = (int(hsv_values[0]), int(hsv_values[1]), int(hsv_values[2]))
    return default if hsv == (0, 0, 0) else hsv


class PayloadDriveOutNode(VisionNode):
    srv = PayloadDriveOutState

    # Fraction of the forward ROI that must be unobstructed to consider the path clear.
    # TODO: tune this threshold based on real camera output.
    CLEAR_THRESHOLD = 0.3

    def __init__(self) -> None:
        super().__init__(
            self.__class__.srv,
            node_name=self.node_name(),
        )
        self.create_service(
            self.srv,
            self.vision_service,
            self._service_callback,
        )
        self._debug_pub = None
        if self.debug:
            debug_topic = "vision/payload_drive_out_node/debug/compressed"
            self._debug_pub = self.create_publisher(CompressedImage, debug_topic, 1)
            self.get_logger().info(f"Debug stream publishing to {debug_topic}")

    def _service_callback(
        self,
        request: PayloadDriveOutState.Request,
        response: PayloadDriveOutState.Response,
    ) -> PayloadDriveOutState.Response:
        image_msg, _ = self.request_data(cam_image=True, cam_info=False)
        response.has_image = image_msg is not None

        if image_msg is None:
            response.can_drive_out = False
            response.clear_ratio = 0.0
            return response

        frame = self.convert_image_msg_to_frame(image_msg)
        lower = _request_hsv_or_default(request.lower_hsv, (0, 0, 180))
        upper = _request_hsv_or_default(request.upper_hsv, (180, 20, 255))
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if frame.shape[2] == 3 else frame
        _, clear_ratio, _, debug_frame = detect_payload_unreeled(
            bgr, lower, upper, debug=self.debug
        )

        if self._debug_pub is not None and debug_frame is not None:
            ok, buf = cv2.imencode(".jpg", debug_frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if ok:
                msg = CompressedImage()
                msg.header = image_msg.header
                msg.format = "jpeg"
                msg.data = buf.tobytes()
                self._debug_pub.publish(msg)

        response.clear_ratio = float(clear_ratio)
        response.can_drive_out = bool(clear_ratio >= self.CLEAR_THRESHOLD)
        return response


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = PayloadDriveOutNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
