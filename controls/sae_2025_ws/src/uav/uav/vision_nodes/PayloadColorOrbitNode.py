from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from uav.utils import green, pink
from .VisionNode import VisionNode
from uav.vision_nodes.payload_perception_common import (
    AprilTagDetectorCache,
    DEFAULT_TAG_FAMILY,
    back_view_angle_deg,
    compute_edge_follow_control,
    compute_edge_metrics,
    solve_payload_apriltags,
)
from uav_interfaces.srv import PayloadColorOrbitState


class PayloadColorOrbitNode(VisionNode):
    srv = PayloadColorOrbitState

    def __init__(self) -> None:
        super().__init__(self.__class__.srv, node_name=self.node_name())
        self._detector_cache = AprilTagDetectorCache()
        self._lower_pink = np.array(pink[0], dtype=np.uint8)
        self._upper_pink = np.array(pink[1], dtype=np.uint8)
        self._lower_green = np.array(green[0], dtype=np.uint8)
        self._upper_green = np.array(green[1], dtype=np.uint8)
        self.create_service(
            self.srv,
            self.vision_service,
            self.service_callback,
        )

    def service_callback(
        self,
        request: PayloadColorOrbitState.Request,
        response: PayloadColorOrbitState.Response,
    ) -> PayloadColorOrbitState.Response:
        detector = self._detector_cache.get(request.tag_family or DEFAULT_TAG_FAMILY)
        response.detector_available = detector is not None
        image_msg, camera_info = self.request_data(cam_image=True, cam_info=True)
        response.has_image = image_msg is not None
        response.has_camera_info = camera_info is not None
        if image_msg is None or camera_info is None:
            return response

        bgr = self.convert_image_msg_to_frame(image_msg)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        pink_ratio, pink_count, green_count = compute_edge_metrics(
            bgr,
            self._lower_pink,
            self._upper_pink,
            self._lower_green,
            self._upper_green,
        )
        boundary_detected, lateral_error_px, boundary_angle = (
            compute_edge_follow_control(
                bgr,
                self._lower_pink,
                self._upper_pink,
                self._lower_green,
                self._upper_green,
            )
        )
        response.pink_ratio = float(pink_ratio)
        response.pink_count = int(pink_count)
        response.green_count = int(green_count)
        response.boundary_detected = bool(boundary_detected)
        response.lateral_error_px = float(lateral_error_px)
        response.boundary_angle = float(boundary_angle)

        if detector is None:
            return response

        tag_size_m = float(request.tag_size_m) if request.tag_size_m > 0.0 else 0.1
        observations = solve_payload_apriltags(
            gray,
            camera_info,
            detector,
            self._detector_cache.backend,
            tag_size_m,
        )
        for observation in observations:
            response.tag_ids.append(int(observation.tag_id))
            response.pose_x.append(float(observation.pose_x))
            response.pose_y.append(float(observation.pose_y))
            response.pose_yaw.append(float(observation.pose_yaw))
            response.tvec_x.append(float(observation.tvec_x))
            response.tvec_y.append(float(observation.tvec_y))
            response.tvec_z.append(float(observation.tvec_z))
            if int(observation.tag_id) == int(request.dock_target_tag_id):
                response.dock_target_visible = True
                angle_deg = back_view_angle_deg(
                    observation.tvec_x,
                    observation.tvec_y,
                    observation.tvec_z,
                )
                response.dock_target_view_angle_deg = (
                    float(angle_deg) if angle_deg is not None else 0.0
                )

        return response


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = PayloadColorOrbitNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
