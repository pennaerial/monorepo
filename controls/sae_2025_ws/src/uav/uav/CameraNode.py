import os
import time
import uuid

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from uav_interfaces.srv import CameraData


class Camera(Node):
    """
    Node class to respond to requests with camera image or camera information.
    """

    def __init__(
        self,
        node_name: str = "camera",
        *,
        vehicle_namespace: str | None = None,
        service_name: str | None = None,
        image_topic: str | None = None,
        info_topic: str | None = None,
        queue_size: int = 10,
        display: bool = False,
        enable_failsafe: bool | None = None,
        failsafe_topic: str | None = None,
    ):
        super().__init__(node_name)

        self.declare_parameter("vehicle_name", "uav")
        self.declare_parameter("vehicle_namespace", "")
        self.declare_parameter("camera_service_name", "")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("display", display)
        self.declare_parameter("save_vision_milliseconds", 0)
        self.declare_parameter("debug", False)
        self.declare_parameter("enable_failsafe", True)
        self.declare_parameter("failsafe_topic", "/failsafe_trigger")

        resolved_namespace = self._resolve_namespace(vehicle_namespace)
        resolved_image_topic = self._resolve_transport_path(
            explicit=image_topic,
            param_name="image_topic",
            default_suffix="camera",
            namespace=resolved_namespace,
            legacy_default="/camera",
        )
        resolved_info_topic = self._resolve_transport_path(
            explicit=info_topic,
            param_name="camera_info_topic",
            default_suffix="camera_info",
            namespace=resolved_namespace,
            legacy_default="/camera_info",
        )
        resolved_service_name = self._resolve_transport_path(
            explicit=service_name,
            param_name="camera_service_name",
            default_suffix="camera_data",
            namespace=resolved_namespace,
            legacy_default="/camera_data",
        )
        resolved_display = bool(self.get_parameter("display").value)
        resolved_enable_failsafe = (
            bool(enable_failsafe)
            if enable_failsafe is not None
            else bool(self.get_parameter("enable_failsafe").value)
        )
        resolved_failsafe_topic = (
            failsafe_topic or str(self.get_parameter("failsafe_topic").value).strip()
        )

        self.vehicle_namespace = resolved_namespace
        self.image_topic = resolved_image_topic
        self.camera_info_topic = resolved_info_topic
        self.camera_service_name = resolved_service_name
        self.image = None
        self.camera_info = None
        self.display = resolved_display

        self.image_subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, queue_size
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, queue_size
        )

        self.save_vision_milliseconds = int(
            self.get_parameter("save_vision_milliseconds").value
        )
        self.vision_debug = bool(self.get_parameter("debug").value)
        self.uuid = str(uuid.uuid4())
        self.last_saved_time = time.time_ns() // 1_000_000

        self.service = self.create_service(
            CameraData, self.camera_service_name, self.service_callback
        )

        self.failsafe_publisher = None
        if resolved_enable_failsafe and resolved_failsafe_topic:
            qos_profile = QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.failsafe_publisher = self.create_publisher(
                Bool, resolved_failsafe_topic, qos_profile
            )

        self.get_logger().info(
            f"{node_name} subscribing to {self.image_topic} and serving {self.camera_service_name}."
        )
        self.get_logger().info(
            f"{node_name} subscribing to {self.camera_info_topic}."
        )

    def _resolve_namespace(self, explicit_namespace: str | None) -> str | None:
        if explicit_namespace is not None:
            return self._normalize_namespace(explicit_namespace)

        configured_namespace = str(self.get_parameter("vehicle_namespace").value).strip()
        if configured_namespace:
            return self._normalize_namespace(configured_namespace)

        vehicle_name = str(self.get_parameter("vehicle_name").value).strip()
        if vehicle_name:
            return self._normalize_namespace(vehicle_name)
        return None

    @staticmethod
    def _normalize_namespace(namespace: str | None) -> str | None:
        if namespace is None:
            return None
        normalized = str(namespace).strip().strip("/")
        if not normalized:
            return None
        return f"/{normalized}"

    @staticmethod
    def _join_namespace(namespace: str | None, suffix: str) -> str:
        clean_suffix = suffix.lstrip("/")
        if not namespace:
            return f"/{clean_suffix}"
        return f"{namespace.rstrip('/')}/{clean_suffix}"

    def _resolve_transport_path(
        self,
        *,
        explicit: str | None,
        param_name: str,
        default_suffix: str,
        namespace: str | None,
        legacy_default: str,
    ) -> str:
        if explicit:
            return explicit
        configured = str(self.get_parameter(param_name).value).strip()
        if configured:
            return configured
        if namespace:
            return self._join_namespace(namespace, default_suffix)
        return legacy_default

    def convert_image_msg_to_frame(self, msg: Image) -> np.ndarray:
        img_data = np.frombuffer(msg.data, dtype=np.uint8)
        return img_data.reshape((msg.height, msg.width, 3))

    def image_callback(self, msg: Image):
        """
        Callback for receiving image requests.
        """
        self.image = msg
        frame = self.convert_image_msg_to_frame(msg)
        if self.display:
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)
        if self.save_vision_milliseconds > 0:
            if (
                abs(time.time_ns() // 1_000_000 - self.last_saved_time)
                >= self.save_vision_milliseconds
            ):
                self.last_saved_time = time.time_ns() // 1_000_000
                timestamp = int(time.time())
                path = os.path.expanduser(f"~/vision_imgs/{self.uuid}/raw")
                os.makedirs(path, exist_ok=True)
                cv2.imwrite(os.path.join(path, f"{timestamp}.png"), frame)
                if self.vision_debug:
                    self.get_logger().error("Saving vision image @ " + str(timestamp))

    def camera_info_callback(self, msg: CameraInfo):
        """
        Callback for receiving camera info messages. Stores the camera info
        for later use.

        Args:
            msg (CameraInfo): The ROS 2 CameraInfo message.
        """
        self.camera_info = msg

    def service_callback(
        self, request: CameraData.Request, response: CameraData.Response
    ):
        """
        Callback for receiving camera requests.
        """
        self.get_logger().info("Received request for camera data.")
        if request.cam_image:
            if self.image is not None:
                response.image = self.image
            else:
                self.get_logger().warn("No image available.")

        if request.cam_info:
            if self.camera_info is not None:
                response.camera_info = self.camera_info
            else:
                self.get_logger().warn("No camera info available.")
        self.get_logger().info("Sending camera data.")
        return response

    def publish_failsafe(self) -> None:
        if self.failsafe_publisher is None:
            return
        self.failsafe_publisher.publish(Bool(data=True))


def main():
    rclpy.init()
    node = None
    try:
        node = Camera()
        rclpy.spin(node)
    except Exception as exc:
        print(exc)
        if node is not None:
            node.publish_failsafe()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


CameraNode = Camera
