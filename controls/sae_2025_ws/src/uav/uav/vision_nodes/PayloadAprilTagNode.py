from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CameraInfo, CompressedImage

from vehicle_common.base import VisionNode
from uav.vision_nodes.payload_perception_common import (
    AprilTagDetectorCache,
    DEFAULT_TAG_FAMILY,
    AprilTagObservation,
    detect_payload_apriltags,
    solve_payload_apriltags,
    _iter_detections,
    _object_points_for_tag_size,
    camera_model_from_info,
)
from uav_interfaces.srv import PayloadAprilTagState


class PayloadAprilTagNode(VisionNode):
    srv = PayloadAprilTagState

    def __init__(self) -> None:
        super().__init__(
            self.__class__.srv,
            node_name=self.node_name(),
            use_service=False,
        )
        self._detector_cache = AprilTagDetectorCache()
        self._last_tag_family = DEFAULT_TAG_FAMILY
        self._last_tag_size_m = 0.1
        self._debug_bridge = CvBridge()
        self.create_service(
            self.srv,
            self.vision_service,
            self.service_callback,
        )
        self._debug_pub = None
        if self.debug:
            debug_topic = self.vision_service + "/debug_image/compressed"
            self._debug_pub = self.create_publisher(CompressedImage, debug_topic, 1)
            self.get_logger().info(f"Debug image publishing on: {debug_topic}")
            self.create_timer(1.0 / 30.0, self._debug_timer_callback)

    def service_callback(
        self,
        request: PayloadAprilTagState.Request,
        response: PayloadAprilTagState.Response,
    ) -> PayloadAprilTagState.Response:
        detector = self._detector_cache.get(request.tag_family or DEFAULT_TAG_FAMILY)
        response.detector_available = detector is not None
        image_msg, camera_info = self.request_data(cam_image=True, cam_info=True)
        response.has_image = image_msg is not None
        response.has_camera_info = camera_info is not None
        if image_msg is None or camera_info is None:
            return response

        if detector is None:
            response.image_width = int(camera_info.width)
            return response

        bgr = self.convert_image_msg_to_frame(image_msg)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        tag_size_m = float(request.tag_size_m) if request.tag_size_m > 0.0 else 0.1
        self._last_tag_family = request.tag_family or DEFAULT_TAG_FAMILY
        self._last_tag_size_m = tag_size_m
        all_observations = detect_payload_apriltags(
            gray,
            camera_info,
            detector,
            self._detector_cache.backend,
            tag_size_m,
        )
        observations = solve_payload_apriltags(
            gray,
            camera_info,
            detector,
            self._detector_cache.backend,
            tag_size_m,
        )

        response.image_width = int(camera_info.width)
        for observation in observations:
            pose_x = observation.pose_x
            pose_y = observation.pose_y
            pose_yaw = observation.pose_yaw
            if pose_x is None or pose_y is None or pose_yaw is None:
                continue
            response.tag_ids.append(int(observation.tag_id))
            response.pose_x.append(float(pose_x))
            response.pose_y.append(float(pose_y))
            response.pose_yaw.append(float(pose_yaw))
            response.tvec_x.append(float(observation.tvec_x))
            response.tvec_y.append(float(observation.tvec_y))
            response.tvec_z.append(float(observation.tvec_z))
            response.center_x.append(float(observation.center_x))
            response.center_y.append(float(observation.center_y))
            response.yaw_error.append(float(observation.yaw_error))

        for observation in all_observations:
            response.all_tag_ids.append(int(observation.tag_id))
            response.all_tvec_x.append(float(observation.tvec_x))
            response.all_tvec_y.append(float(observation.tvec_y))
            response.all_tvec_z.append(float(observation.tvec_z))
            response.all_center_x.append(float(observation.center_x))
            response.all_center_y.append(float(observation.center_y))
            response.all_yaw_error.append(float(observation.yaw_error))
            response.all_tag_area.append(float(observation.area))

        return response

    def _debug_timer_callback(self) -> None:
        image_msg = self.image
        camera_info = self.camera_info
        if image_msg is None or camera_info is None:
            return
        detector = self._detector_cache.get(self._last_tag_family)
        if detector is None:
            return
        bgr = self.convert_image_msg_to_frame(image_msg)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        all_observations = detect_payload_apriltags(
            gray,
            camera_info,
            detector,
            self._detector_cache.backend,
            self._last_tag_size_m,
        )
        observations = solve_payload_apriltags(
            gray,
            camera_info,
            detector,
            self._detector_cache.backend,
            self._last_tag_size_m,
        )
        self._draw_debug(
            bgr,
            gray,
            camera_info,
            all_observations,
            observations,
            self._last_tag_size_m,
            detector,
        )

    def _draw_debug(
        self,
        bgr: np.ndarray,
        gray: np.ndarray,
        camera_info: CameraInfo,
        all_observations: list[AprilTagObservation],
        observations: list[AprilTagObservation],
        tag_size_m: float,
        detector,
    ) -> None:
        vis = bgr.copy()
        h, w = vis.shape[:2]
        img_cx, img_cy = w / 2.0, h / 2.0

        camera_matrix, dist_coeffs = camera_model_from_info(camera_info)
        object_points = _object_points_for_tag_size(tag_size_m)
        solved_ids = {obs.tag_id for obs in observations}

        # Image center crosshair
        cv2.drawMarker(
            vis, (int(img_cx), int(img_cy)), (255, 255, 255), cv2.MARKER_CROSS, 20, 1
        )

        # Primary tag for HUD = largest area among all_observations
        primary = (
            max(all_observations, key=lambda o: o.area) if all_observations else None
        )

        for det in _iter_detections(gray, detector, self._detector_cache.backend):
            tag_id = int(getattr(det, "tag_id", -1))
            corners = det.corners.astype(int)
            tag_cx, tag_cy = int(det.center[0]), int(det.center[1])
            is_solved = tag_id in solved_ids
            border_color = (0, 255, 0) if is_solved else (0, 165, 255)

            # Tag border
            for i in range(4):
                cv2.line(
                    vis, tuple(corners[i]), tuple(corners[(i + 1) % 4]), border_color, 2
                )

            # Corner pixel labels
            for px, py in corners:
                cv2.circle(vis, (px, py), 4, border_color, -1)
                cv2.putText(
                    vis,
                    f"({px},{py})",
                    (px + 4, py - 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    border_color,
                    1,
                )

            # Tag center dot and ID
            cv2.circle(vis, (tag_cx, tag_cy), 6, (0, 0, 255), -1)
            cv2.putText(
                vis,
                f"id={tag_id}",
                (tag_cx + 8, tag_cy - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
            )

            # Steering arrow: image center → tag center
            cv2.arrowedLine(
                vis,
                (int(img_cx), int(img_cy)),
                (tag_cx, tag_cy),
                (0, 165, 255),
                2,
                tipLength=0.15,
            )
            cv2.putText(
                vis,
                "steer",
                ((int(img_cx) + tag_cx) // 2 + 4, (int(img_cy) + tag_cy) // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 165, 255),
                1,
            )

            # solvePnP for 3D axes
            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                det.corners.astype(np.float32),
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if ok:
                axis_len = tag_size_m * 0.5
                axes_3d = np.array(
                    [[0, 0, 0], [axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len]],
                    dtype=np.float32,
                )
                projected, _ = cv2.projectPoints(
                    axes_3d, rvec, tvec, camera_matrix, np.zeros((4, 1))
                )
                projected = projected.reshape(-1, 2).astype(int)
                origin = tuple(projected[0])
                cv2.arrowedLine(
                    vis, origin, tuple(projected[1]), (0, 0, 255), 2, tipLength=0.3
                )  # X red
                cv2.arrowedLine(
                    vis, origin, tuple(projected[2]), (0, 255, 0), 2, tipLength=0.3
                )  # Y green
                cv2.arrowedLine(
                    vis, origin, tuple(projected[3]), (255, 0, 0), 2, tipLength=0.3
                )  # Z blue

        # HUD overlay for primary tag
        if primary is not None:
            tx, ty, tz = primary.tvec_x, primary.tvec_y, primary.tvec_z
            lateral_error_px = primary.center_x - img_cx

            # Distance — large text top-left
            cv2.putText(
                vis,
                f"dist: {tz:.3f} m",
                (8, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.1,
                (0, 255, 255),
                2,
            )

            # Detail lines
            lines = [
                f"tvec:    x={tx:.3f}  y={ty:.3f}  z={tz:.3f}",
                f"lat err: {lateral_error_px:+.1f} px",
                f"yaw err: {primary.yaw_error:+.3f} rad  (0=orthogonal)",
            ]
            if (
                primary.pose_x is not None
                and primary.pose_y is not None
                and primary.pose_yaw is not None
            ):
                lines.append(
                    f"pose:    x={primary.pose_x:.3f}  y={primary.pose_y:.3f}  yaw={math.degrees(primary.pose_yaw):.1f} deg"
                )
            for i, line in enumerate(lines):
                cv2.putText(
                    vis,
                    line,
                    (8, 60 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 0),
                    2,
                )
                cv2.putText(
                    vis,
                    line,
                    (8, 60 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (200, 200, 200),
                    1,
                )

            # Tag count top-right
            label = f"{len(all_observations)} tag(s) detected"
            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.putText(
                vis,
                label,
                (w - lw - 8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                2,
            )
            cv2.putText(
                vis,
                label,
                (w - lw - 8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (200, 200, 200),
                1,
            )
        else:
            cv2.putText(
                vis,
                "no tags detected",
                (8, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )

        if self._debug_pub is not None:
            try:
                msg = self._debug_bridge.cv2_to_compressed_imgmsg(
                    vis, dst_format="jpeg"
                )
                msg.header.stamp = self.get_clock().now().to_msg()
                self._debug_pub.publish(msg)
            except Exception as exc:
                self.get_logger().warn(
                    f"PayloadAprilTagNode: debug publish failed: {exc}"
                )


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = PayloadAprilTagNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
