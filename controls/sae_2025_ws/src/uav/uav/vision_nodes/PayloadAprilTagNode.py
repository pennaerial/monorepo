from __future__ import annotations

import cv2
import rclpy

from .VisionNode import VisionNode
from uav.vision_nodes.payload_perception_common import (
    AprilTagDetectorCache,
    DEFAULT_TAG_FAMILY,
    solve_payload_apriltags,
)
from uav_interfaces.srv import PayloadAprilTagState


class PayloadAprilTagNode(VisionNode):
    srv = PayloadAprilTagState

    def __init__(self) -> None:
        super().__init__(self.__class__.srv, node_name=self.node_name())
        self._detector_cache = AprilTagDetectorCache()
        self.create_service(
            self.srv,
            self.vision_service,
            self.service_callback,
        )

    def service_callback(
        self,
        request: PayloadAprilTagState.Request,
        response: PayloadAprilTagState.Response,
    ) -> PayloadAprilTagState.Response:
        image_msg, camera_info = self.request_data(cam_image=True, cam_info=True)
        response.has_image = image_msg is not None
        response.has_camera_info = camera_info is not None
        if image_msg is None or camera_info is None:
            return response

        detector = self._detector_cache.get(request.tag_family or DEFAULT_TAG_FAMILY)
        response.detector_available = detector is not None
        if detector is None:
            response.image_width = int(camera_info.width)
            return response

        bgr = self.convert_image_msg_to_frame(image_msg)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        tag_size_m = float(request.tag_size_m) if request.tag_size_m > 0.0 else 0.1
        observations = solve_payload_apriltags(gray, camera_info, detector, tag_size_m)

        response.image_width = int(camera_info.width)
        for observation in observations:
            response.tag_ids.append(int(observation.tag_id))
            response.pose_x.append(float(observation.pose_x))
            response.pose_y.append(float(observation.pose_y))
            response.pose_yaw.append(float(observation.pose_yaw))
            response.tvec_x.append(float(observation.tvec_x))
            response.tvec_y.append(float(observation.tvec_y))
            response.tvec_z.append(float(observation.tvec_z))
            response.center_x.append(float(observation.center_x))
            response.center_y.append(float(observation.center_y))

        return response


def main() -> None:
    rclpy.init()
    node = PayloadAprilTagNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
