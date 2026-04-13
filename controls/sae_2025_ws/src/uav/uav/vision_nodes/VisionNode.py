import os
import uuid

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import Trigger
from uav.utils import camel_to_snake
from uav_interfaces.srv import CameraData


class VisionNode(Node):
    """
    Base class for ROS 2 nodes that process vision data.
    Provides an interface for handling image streams, processing frames,
    and managing vision-based tasks such as tracking and calibration.
    """

    @classmethod
    def node_name(cls):
        return camel_to_snake(cls.__name__)

    @classmethod
    def service_name(cls, camera_namespace: str | None = None):
        normalized_namespace = cls._normalize_namespace(camera_namespace)
        if not normalized_namespace:
            return f"vision/{cls.node_name()}"
        return f"{normalized_namespace}/vision/{cls.node_name()}"

    @classmethod
    def __str__(cls):
        return cls.node_name()

    def __init__(
        self,
        custom_service,
        *,
        node_name: str | None = None,
        display: bool = False,
        use_service: bool = True,
        vehicle_namespace: str | None = None,
        camera_service_name: str | None = None,
        image_topic: str | None = None,
        camera_info_topic: str | None = None,
        vision_service_name: str | None = None,
        enable_failsafe: bool | None = None,
        failsafe_service_name: str | None = None,
    ):
        super().__init__(node_name or self.__class__.__name__)

        self.declare_parameter("debug", False)
        self.declare_parameter("sim", True)
        self.declare_parameter("save_vision", False)
        self.declare_parameter("display", display)
        self.declare_parameter("vehicle_name", "uav")
        self.declare_parameter("vehicle_namespace", "")
        self.declare_parameter("camera_service_name", "")
        self.declare_parameter("camera_image_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("preferred_image_transport", "raw")
        self.declare_parameter("vision_service_name", "")
        self.declare_parameter("use_camera_service", use_service)
        self.declare_parameter("enable_failsafe_service", False)
        self.declare_parameter("failsafe_service_name", "")

        self.debug = bool(self.get_parameter("debug").value)
        self.sim = bool(self.get_parameter("sim").value)
        self.save_vision = bool(self.get_parameter("save_vision").value)
        self.display = bool(self.get_parameter("display").value)
        self.custom_service_type = custom_service
        self.uuid = str(uuid.uuid4())
        if self.save_vision:
            os.makedirs(os.path.expanduser(f"~/vision_imgs/{self.uuid}"), exist_ok=True)

        self.vehicle_namespace = self._resolve_namespace(vehicle_namespace)
        self.camera_namespace = self.vehicle_namespace
        self.camera_service_name = self._resolve_transport_path(
            explicit=camera_service_name,
            param_name="camera_service_name",
            default_suffix="camera_data",
            legacy_default="/camera_data",
        )
        self.image_topic = self._resolve_transport_path(
            explicit=image_topic,
            param_name="camera_image_topic",
            default_suffix="camera",
            legacy_default="/camera",
        )
        self.compressed_image_topic = self._compressed_topic_for(self.image_topic)
        self.camera_info_topic = self._resolve_transport_path(
            explicit=camera_info_topic,
            param_name="camera_info_topic",
            default_suffix="camera_info",
            legacy_default="/camera_info",
        )
        self.preferred_image_transport = self._resolve_preferred_image_transport(
            str(self.get_parameter("preferred_image_transport").value).strip()
        )
        self.vision_service = self._resolve_vision_service_name(vision_service_name)
        self.use_service = bool(self.get_parameter("use_camera_service").value)

        self.client = None
        self.image = None
        self.image_transport = None
        self.camera_info = None
        self.bridge = CvBridge()
        self._camera_request_future = None
        self._camera_service_warned = False
        self._camera_wait_warned = False
        self._camera_fallback_warned = False
        if self.use_service:
            self.client = self.create_client(CameraData, self.camera_service_name)
            # Poll camera_data asynchronously so service callbacks can read cached data
            # without issuing nested blocking service requests.
            self._camera_poll_timer = self.create_timer(
                0.1, self._refresh_camera_cache_from_service
            )
        else:
            if self.preferred_image_transport == "compressed":
                self.image_subscription = self.create_subscription(
                    CompressedImage,
                    self.compressed_image_topic,
                    self.compressed_image_callback,
                    1,
                )
            else:
                self.image_subscription = self.create_subscription(
                    Image, self.image_topic, self.image_callback, 1
                )
            self.camera_info_subscription = self.create_subscription(
                CameraInfo, self.camera_info_topic, self.camera_info_callback, 1
            )

        resolved_enable_failsafe = (
            bool(enable_failsafe)
            if enable_failsafe is not None
            else bool(self.get_parameter("enable_failsafe_service").value)
        )
        resolved_failsafe_service = (
            failsafe_service_name
            or str(self.get_parameter("failsafe_service_name").value).strip()
        )
        self.failsafe_trigger_client = None
        if resolved_enable_failsafe and resolved_failsafe_service:
            self.failsafe_trigger_client = self.create_client(
                Trigger, resolved_failsafe_service
            )

    @staticmethod
    def _normalize_namespace(namespace: str | None) -> str | None:
        if namespace is None:
            return None
        normalized = str(namespace).strip().strip("/")
        if not normalized:
            return None
        return f"/{normalized}"

    def _resolve_namespace(self, explicit_namespace: str | None) -> str | None:
        if explicit_namespace is not None:
            return self._normalize_namespace(explicit_namespace)
        configured_namespace = str(
            self.get_parameter("vehicle_namespace").value
        ).strip()
        if configured_namespace:
            return self._normalize_namespace(configured_namespace)
        vehicle_name = str(self.get_parameter("vehicle_name").value).strip()
        if vehicle_name:
            return self._normalize_namespace(vehicle_name)
        return None

    def _resolve_transport_path(
        self,
        *,
        explicit: str | None,
        param_name: str,
        default_suffix: str,
        legacy_default: str,
    ) -> str:
        if explicit:
            return explicit
        configured = str(self.get_parameter(param_name).value).strip()
        if configured:
            return configured
        if self.vehicle_namespace:
            return f"{self.vehicle_namespace}/{default_suffix}"
        return legacy_default

    def _resolve_vision_service_name(self, explicit: str | None) -> str:
        if explicit:
            return explicit
        configured = str(self.get_parameter("vision_service_name").value).strip()
        if configured:
            return configured
        return self.service_name(self.vehicle_namespace)

    @staticmethod
    def _compressed_topic_for(raw_topic: str) -> str:
        return f"{raw_topic.rstrip('/')}/compressed"

    @staticmethod
    def _resolve_preferred_image_transport(transport: str) -> str:
        normalized = transport.lower() or "raw"
        if normalized not in {"raw", "compressed"}:
            raise ValueError(
                f"Unsupported preferred_image_transport '{transport}'. Expected raw or compressed."
            )
        return normalized

    def image_callback(self, msg: Image):
        """
        Callback for receiving image requests.
        """
        self._cache_image(msg, "raw")

    def compressed_image_callback(self, msg: CompressedImage):
        """
        Callback for receiving compressed image requests.
        """
        self._cache_image(msg, "compressed")

    def camera_info_callback(self, msg: CameraInfo):
        """
        Callback for receiving camera info messages. Stores the camera info
        for later use.

        Args:
            msg (CameraInfo): The ROS 2 CameraInfo message.
        """
        self.camera_info = msg

    def _cache_image(self, msg: Image | CompressedImage, transport: str) -> None:
        self.image = msg
        self.image_transport = transport
        if self.display:
            self.display_frame(self.convert_image_msg_to_frame(msg), self.node_name())

    def convert_image_msg_to_frame(self, msg: Image | CompressedImage) -> np.ndarray:
        """
        Converts a ROS 2 Image message to a NumPy array.
        """
        if isinstance(msg, CompressedImage):
            return self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.sim:
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            frame = img_data.reshape((msg.height, msg.width, 3))
            if msg.encoding == "rgb8":
                return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return frame

    def send_req(self, req):
        if self.client is None:
            raise RuntimeError("Camera service transport is not configured.")
        return self.client.call_async(req)

    def _refresh_camera_cache_from_service(self) -> None:
        if self.client is None:
            return
        if (
            self._camera_request_future is not None
            and not self._camera_request_future.done()
        ):
            return
        if not self.client.wait_for_service(timeout_sec=0.0):
            if not self._camera_wait_warned:
                self.get_logger().warn(
                    f"Waiting for camera service {self.camera_service_name}."
                )
                self._camera_wait_warned = True
            return

        self._camera_wait_warned = False
        request = CameraData.Request()
        request.cam_image_raw = self.preferred_image_transport == "raw"
        request.cam_image_compressed = self.preferred_image_transport == "compressed"
        request.cam_info = True
        self._camera_request_future = self.send_req(request)
        self._camera_request_future.add_done_callback(self._handle_camera_response)

    def _select_camera_response_image(
        self, response: CameraData.Response
    ) -> tuple[Image | CompressedImage | None, str | None]:
        if self.preferred_image_transport == "raw":
            candidates = (
                (response.image_raw, "raw"),
                (response.image_compressed, "compressed"),
            )
        else:
            candidates = (
                (response.image_compressed, "compressed"),
                (response.image_raw, "raw"),
            )

        for image, transport in candidates:
            if image is not None and getattr(image, "data", None):
                return image, transport
        return None, None

    def _handle_camera_response(self, future) -> None:
        self._camera_request_future = None
        try:
            response = future.result()
        except Exception as exc:
            if not self._camera_service_warned:
                self.get_logger().error(f"Camera service call failed: {exc}")
                self._camera_service_warned = True
            return

        self._camera_service_warned = False
        if response is None:
            return
        image_msg, transport = self._select_camera_response_image(response)
        if image_msg is not None and transport is not None:
            if (
                transport != self.preferred_image_transport
                and not self._camera_fallback_warned
            ):
                self.get_logger().warn(
                    f"Camera service returned {transport} instead of preferred {self.preferred_image_transport}.",
                    throttle_duration_sec=5.0,
                )
                self._camera_fallback_warned = True
            elif transport == self.preferred_image_transport:
                self._camera_fallback_warned = False
            self._cache_image(image_msg, transport)
        if response.camera_info.header.frame_id or response.camera_info.width > 0:
            self.camera_info = response.camera_info

    def request_data(self, cam_image: bool = False, cam_info: bool = False):
        """
        Sends request for camera image or camera information.
        """
        if not cam_info and not cam_image:
            return None, None

        if not self.use_service:
            image = self.image if cam_image else None
            camera_info = self.camera_info if cam_info else None
            return image, camera_info

        image = self.image if cam_image else None
        camera_info = self.camera_info if cam_info else None
        if (cam_image and image is None) or (cam_info and camera_info is None):
            self._refresh_camera_cache_from_service()
        return image, camera_info

    def display_frame(self, frame: np.ndarray, window_name: str) -> None:
        """
        Displays the given frame using OpenCV.
        """
        cv2.imshow(window_name, frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Shutting down display.")
            if rclpy.ok():
                rclpy.shutdown()
            cv2.destroyAllWindows()

    def cleanup(self):
        """
        Cleanup resources, such as OpenCV windows.
        """
        cv2.destroyAllWindows()

    def publish_failsafe(self):
        if self.failsafe_trigger_client is None or not rclpy.ok():
            return
        if not self.failsafe_trigger_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Failsafe service is not available.")
            return
        future = self.failsafe_trigger_client.call_async(Trigger.Request())
        try:
            rclpy.spin_until_future_complete(self, future)
        except ExternalShutdownException:
            return
        result = future.result()
        if result is not None and result.success:
            self.get_logger().info("Failsafe triggered.")
        else:
            self.get_logger().error("Failed to trigger failsafe.")
