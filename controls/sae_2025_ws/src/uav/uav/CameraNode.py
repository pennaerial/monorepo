import importlib
import os
import time
import uuid
from typing import Callable, cast

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool
from uav_interfaces.srv import CameraData

PreprocessHook = Callable[[np.ndarray], np.ndarray]


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
        queue_size: int = 1,
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
        self.declare_parameter("input_transport", "raw")
        self.declare_parameter("input_raw_topic", "")
        self.declare_parameter("input_compressed_topic", "")
        self.declare_parameter("input_camera_info_topic", "")
        self.declare_parameter("rotate_degrees", 0.0)
        self.declare_parameter("preprocess_hook", "")
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
        )
        resolved_info_topic = self._resolve_transport_path(
            explicit=info_topic,
            param_name="camera_info_topic",
            default_suffix="camera_info",
        )
        resolved_service_name = self._resolve_transport_path(
            explicit=service_name,
            param_name="camera_service_name",
            default_suffix="camera_data",
        )
        self.input_transport = self._resolve_transport_mode(
            str(self.get_parameter("input_transport").value).strip()
        )
        self.rotate_degrees = self._float_parameter("rotate_degrees")
        self.preprocess_hook_name = str(
            self.get_parameter("preprocess_hook").value
        ).strip()
        self.preprocess_hook = self._load_preprocess_hook(self.preprocess_hook_name)
        self._preprocess_active = (
            abs(self.rotate_degrees) > 1e-9 or self.preprocess_hook is not None
        )

        self.vehicle_namespace = resolved_namespace
        self.image_topic = resolved_image_topic
        self.compressed_topic = self._compressed_topic_for(self.image_topic)
        self.camera_info_topic = resolved_info_topic
        self.camera_service_name = resolved_service_name
        self.input_raw_topic = self._resolve_transport_path(
            explicit=None,
            param_name="input_raw_topic",
            default_suffix="camera",
        )
        self.input_compressed_topic = self._resolve_transport_path(
            explicit=None,
            param_name="input_compressed_topic",
            default_suffix="camera/compressed",
        )
        self.input_camera_info_topic = self._resolve_transport_path(
            explicit=None,
            param_name="input_camera_info_topic",
            default_suffix="camera_info",
        )

        self._raw_passthrough = (
            self.input_transport in {"raw", "both"}
            and self.input_raw_topic == self.image_topic
            and not self._preprocess_active
        )
        self._compressed_passthrough = (
            self.input_transport in {"compressed", "both"}
            and self.input_compressed_topic == self.compressed_topic
            and not self._preprocess_active
        )
        self._info_passthrough = self.input_camera_info_topic == self.camera_info_topic

        if (
            self._preprocess_active
            and self.input_transport in {"raw", "both"}
            and self.input_raw_topic == self.image_topic
        ):
            raise ValueError(
                "CameraNode cannot preprocess raw images when input_raw_topic equals image_topic."
            )
        if (
            self._preprocess_active
            and self.input_transport in {"compressed", "both"}
            and self.input_compressed_topic == self.compressed_topic
        ):
            raise ValueError(
                "CameraNode cannot preprocess compressed images when input_compressed_topic equals image_topic/compressed."
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

        self.display = resolved_display
        self.bridge = CvBridge()
        self.raw_image: Image | None = None
        self.compressed_image: CompressedImage | None = None
        self.camera_info: CameraInfo | None = None

        self.raw_publisher = (
            None
            if self._raw_passthrough
            else self.create_publisher(Image, self.image_topic, queue_size)
        )
        self.compressed_publisher = (
            None
            if self._compressed_passthrough
            else self.create_publisher(
                CompressedImage, self.compressed_topic, queue_size
            )
        )
        self.camera_info_publisher = (
            None
            if self._info_passthrough
            else self.create_publisher(CameraInfo, self.camera_info_topic, queue_size)
        )

        self.raw_subscription = None
        self.compressed_subscription = None
        if self.input_transport in {"raw", "both"}:
            self.raw_subscription = self.create_subscription(
                Image, self.input_raw_topic, self.raw_image_callback, queue_size
            )
        if self.input_transport in {"compressed", "both"}:
            self.compressed_subscription = self.create_subscription(
                CompressedImage,
                self.input_compressed_topic,
                self.compressed_image_callback,
                queue_size,
            )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.input_camera_info_topic,
            self.camera_info_callback,
            queue_size,
        )

        self.save_vision_milliseconds = self._int_parameter("save_vision_milliseconds")
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
            f"{node_name} input_transport={self.input_transport} "
            f"raw_in={self.input_raw_topic} compressed_in={self.input_compressed_topic} "
            f"camera_info_in={self.input_camera_info_topic} service={self.camera_service_name}"
        )
        self.get_logger().info(
            f"{node_name} outputs raw={self.image_topic} "
            f"compressed={self.compressed_topic} camera_info={self.camera_info_topic}"
        )

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

    @staticmethod
    def _normalize_namespace(namespace: str | None) -> str | None:
        if namespace is None:
            return None
        normalized = str(namespace).strip().strip("/")
        if not normalized:
            return None
        return f"/{normalized}"

    @staticmethod
    def _compressed_topic_for(raw_topic: str) -> str:
        return f"{raw_topic.rstrip('/')}/compressed"

    @staticmethod
    def _resolve_transport_mode(mode: str) -> str:
        normalized = mode.lower() or "raw"
        if normalized not in {"raw", "compressed", "both"}:
            raise ValueError(
                f"Unsupported input_transport '{mode}'. Expected raw, compressed, or both."
            )
        return normalized

    def _resolve_transport_path(
        self,
        *,
        explicit: str | None,
        param_name: str,
        default_suffix: str,
    ) -> str:
        if explicit:
            return self._require_relative_transport_path(explicit, label=param_name)
        configured = str(self.get_parameter(param_name).value).strip()
        if configured:
            return self._require_relative_transport_path(configured, label=param_name)
        return default_suffix

    @staticmethod
    def _require_relative_transport_path(path: str, *, label: str) -> str:
        cleaned = str(path).strip()
        if cleaned.startswith("/"):
            raise ValueError(
                f"{label} must be relative to the vehicle namespace, received absolute path {cleaned!r}."
            )
        return cleaned.lstrip("/")

    def _float_parameter(self, param_name: str) -> float:
        value = self.get_parameter(param_name).value
        if isinstance(value, bool) or not isinstance(value, int | float | str):
            raise ValueError(f"{param_name} must be a numeric parameter.")
        return float(value)

    def _int_parameter(self, param_name: str) -> int:
        value = self.get_parameter(param_name).value
        if isinstance(value, bool) or not isinstance(value, int | float | str):
            raise ValueError(f"{param_name} must be an integer parameter.")
        return int(value)

    def _load_preprocess_hook(self, hook_path: str) -> PreprocessHook | None:
        if not hook_path:
            return None
        module_path = hook_path
        attr_name = ""
        if ":" in hook_path:
            module_path, attr_name = hook_path.split(":", 1)
        else:
            module_path, attr_name = hook_path.rsplit(".", 1)
        module = importlib.import_module(module_path)
        hook = getattr(module, attr_name)
        if not callable(hook):
            raise TypeError(
                f"Configured preprocess_hook '{hook_path}' is not callable."
            )
        return cast(PreprocessHook, hook)

    def _rotate_frame(self, frame: np.ndarray) -> np.ndarray:
        normalized_degrees = self.rotate_degrees % 360.0
        if abs(normalized_degrees) <= 1e-9:
            return frame
        # Right-angle rotations are common on hardware and much cheaper than a
        # generic affine warp.
        if abs(normalized_degrees - 90.0) <= 1e-9:
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if abs(normalized_degrees - 180.0) <= 1e-9:
            return cv2.rotate(frame, cv2.ROTATE_180)
        if abs(normalized_degrees - 270.0) <= 1e-9:
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        height, width = frame.shape[:2]
        center = (width / 2.0, height / 2.0)
        matrix = cv2.getRotationMatrix2D(center, -self.rotate_degrees, 1.0)
        cos = abs(matrix[0, 0])
        sin = abs(matrix[0, 1])
        new_width = int((height * sin) + (width * cos))
        new_height = int((height * cos) + (width * sin))
        matrix[0, 2] += (new_width - width) / 2.0
        matrix[1, 2] += (new_height - height) / 2.0
        return cv2.warpAffine(frame, matrix, (new_width, new_height))

    def _apply_preprocessing(self, frame: np.ndarray) -> np.ndarray:
        processed = self._rotate_frame(frame)
        if self.preprocess_hook is None:
            return processed
        try:
            hooked = self.preprocess_hook(processed)
        except Exception as exc:
            self.get_logger().warn(
                f"Camera preprocess hook failed: {exc}",
                throttle_duration_sec=5.0,
            )
            return processed
        if not isinstance(hooked, np.ndarray):
            self.get_logger().warn(
                "Camera preprocess hook returned a non-ndarray result; using built-in processed frame.",
                throttle_duration_sec=5.0,
            )
            return processed
        return hooked

    def _to_bgr_from_raw(self, msg: Image) -> np.ndarray:
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _to_bgr_from_compressed(self, msg: CompressedImage) -> np.ndarray:
        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("Failed to decode compressed image data.")
        return frame

    def _frame_to_raw_image(self, frame: np.ndarray, header) -> Image:
        image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image.header = header
        return image

    def _frame_to_compressed_image(self, frame: np.ndarray, header) -> CompressedImage:
        ok, encoded = cv2.imencode(".jpg", frame)
        if not ok:
            raise ValueError("Failed to encode compressed image data.")
        image = CompressedImage()
        image.header = header
        image.format = "jpeg compressed bgr8"
        image.data = encoded.tobytes()
        return image

    def _raw_subscribers_present(self) -> bool:
        return (
            self.raw_publisher is not None
            and self.raw_publisher.get_subscription_count() > 0
        )

    def _compressed_subscribers_present(self) -> bool:
        return (
            self.compressed_publisher is not None
            and self.compressed_publisher.get_subscription_count() > 0
        )

    def _handle_frame_side_effects(self, frame: np.ndarray) -> None:
        if self.display:
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)
        if self.save_vision_milliseconds <= 0:
            return
        now_ms = time.time_ns() // 1_000_000
        if abs(now_ms - self.last_saved_time) < self.save_vision_milliseconds:
            return
        self.last_saved_time = now_ms
        timestamp = int(time.time())
        path = os.path.expanduser(f"~/vision_imgs/{self.uuid}/raw")
        os.makedirs(path, exist_ok=True)
        cv2.imwrite(os.path.join(path, f"{timestamp}.png"), frame)
        if self.vision_debug:
            self.get_logger().error(f"Saving vision image @ {timestamp}")

    def raw_image_callback(self, msg: Image):
        self.raw_image = msg

        needs_frame = (
            self._preprocess_active
            or self.display
            or (self.save_vision_milliseconds > 0)
            or (
                self.input_transport == "raw" and self._compressed_subscribers_present()
            )
        )
        frame = self._to_bgr_from_raw(msg) if needs_frame else None
        processed_frame = (
            self._apply_preprocessing(frame.copy())
            if self._preprocess_active and frame is not None
            else frame
        )

        canonical_raw = (
            self._frame_to_raw_image(processed_frame, msg.header)
            if self._preprocess_active and processed_frame is not None
            else msg
        )
        self.raw_image = canonical_raw

        if self.raw_publisher is not None:
            self.raw_publisher.publish(canonical_raw)

        if processed_frame is not None:
            self._handle_frame_side_effects(processed_frame)

        if (
            self.input_transport == "raw"
            and self.compressed_publisher is not None
            and self._compressed_subscribers_present()
        ):
            if processed_frame is None:
                processed_frame = self._to_bgr_from_raw(canonical_raw)
            self.compressed_publisher.publish(
                self._frame_to_compressed_image(processed_frame, canonical_raw.header)
            )

    def compressed_image_callback(self, msg: CompressedImage):
        self.compressed_image = msg

        needs_frame = (
            self._preprocess_active
            or self.display
            or (self.save_vision_milliseconds > 0)
            or (
                self.input_transport == "compressed" and self._raw_subscribers_present()
            )
        )
        frame = self._to_bgr_from_compressed(msg) if needs_frame else None
        processed_frame = (
            self._apply_preprocessing(frame.copy())
            if self._preprocess_active and frame is not None
            else frame
        )

        canonical_compressed = (
            self._frame_to_compressed_image(processed_frame, msg.header)
            if self._preprocess_active and processed_frame is not None
            else msg
        )
        self.compressed_image = canonical_compressed

        if self.compressed_publisher is not None:
            self.compressed_publisher.publish(canonical_compressed)

        if processed_frame is not None:
            self._handle_frame_side_effects(processed_frame)

        if (
            self.input_transport == "compressed"
            and self.raw_publisher is not None
            and self._raw_subscribers_present()
        ):
            if processed_frame is None:
                processed_frame = self._to_bgr_from_compressed(canonical_compressed)
            self.raw_publisher.publish(
                self._frame_to_raw_image(processed_frame, canonical_compressed.header)
            )

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        if self.camera_info_publisher is not None:
            self.camera_info_publisher.publish(msg)

    def service_callback(
        self, request: CameraData.Request, response: CameraData.Response
    ):
        self.get_logger().debug("Received request for camera data.")

        if request.cam_image_raw or request.cam_image_compressed:
            if request.cam_image_raw:
                image = self._select_service_image("raw")
                self._assign_service_image(response, image)
            if request.cam_image_compressed:
                image = self._select_service_image("compressed")
                self._assign_service_image(response, image)
            if self.raw_image is None and self.compressed_image is None:
                self.get_logger().warn(
                    "Waiting for first image on configured camera transport.",
                    throttle_duration_sec=5.0,
                )

        if request.cam_info:
            if self.camera_info is not None:
                response.camera_info = self.camera_info
            else:
                self.get_logger().warn(
                    f"Waiting for first camera info on {self.input_camera_info_topic}.",
                    throttle_duration_sec=5.0,
                )

        self.get_logger().debug("Sending camera data.")
        return response

    def _select_service_image(
        self, requested_transport: str
    ) -> Image | CompressedImage | None:
        if self.input_transport == "both":
            return (
                self.raw_image
                if requested_transport == "raw"
                else self.compressed_image
            )

        if requested_transport == "raw":
            preferred_transports = (
                ("raw", self.raw_image),
                ("compressed", self.compressed_image),
            )
            if self.input_transport == "compressed":
                preferred_transports = (
                    ("compressed", self.compressed_image),
                    ("raw", self.raw_image),
                )
        else:
            preferred_transports = (
                ("compressed", self.compressed_image),
                ("raw", self.raw_image),
            )
            if self.input_transport == "raw":
                preferred_transports = (
                    ("raw", self.raw_image),
                    ("compressed", self.compressed_image),
                )

        for _, image in preferred_transports:
            if image is not None:
                return image
        return None

    @staticmethod
    def _assign_service_image(
        response: CameraData.Response, image: Image | CompressedImage | None
    ) -> None:
        if image is None:
            return
        if isinstance(image, Image):
            response.image_raw = image
        else:
            response.image_compressed = image

    def publish_failsafe(self) -> None:
        if self.failsafe_publisher is None or not rclpy.ok():
            return
        self.failsafe_publisher.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Camera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        print(exc)
        if node is not None and rclpy.ok():
            node.publish_failsafe()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


CameraNode = Camera
