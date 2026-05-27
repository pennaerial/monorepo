from abc import ABC, abstractmethod

from rclpy.node import Node


class Vehicle(ABC):
    """Abstract control target for mission modes."""

    def __init__(
        self,
        node: Node,
        name: str,
        *,
        has_camera: bool = False,
        camera_namespace: str | None = None,
        image_topic: str | None = None,
        camera_info_topic: str | None = None,
        camera_service_name: str | None = None,
    ):
        self.node = node
        self.name = name
        self.has_camera = bool(has_camera)
        self.camera_namespace = self._normalize_namespace(camera_namespace)
        self.image_topic = None
        self.camera_info_topic = None
        self.camera_service_name = None
        if self.has_camera:
            self.image_topic = image_topic or self._default_camera_path("camera")
            self.camera_info_topic = camera_info_topic or self._default_camera_path(
                "camera_info"
            )
            self.camera_service_name = camera_service_name or self._default_camera_path(
                "camera_data"
            )

    @staticmethod
    def _normalize_namespace(namespace: str | None) -> str | None:
        if namespace is None:
            return None
        normalized = str(namespace).strip().strip("/")
        if not normalized:
            return None
        return f"/{normalized}"

    @staticmethod
    def _vision_node_name(vision_node: object) -> str:
        node_name = getattr(vision_node, "node_name", None)
        if callable(node_name):
            return str(node_name())
        class_name = getattr(vision_node, "__name__", None)
        if isinstance(class_name, str):
            return class_name
        return str(vision_node)

    def _default_camera_path(self, suffix: str) -> str | None:
        if not self.has_camera:
            return None
        return self.namespaced_path(suffix)

    def px4_transport_path(self, suffix: str) -> str:
        clean_suffix = suffix.lstrip("/")
        return f"fmu/{clean_suffix}"

    @property
    def logger(self):
        return self.node.get_logger()

    def namespaced_path(self, suffix: str, *, namespace: str | None = None) -> str:
        clean_suffix = suffix.lstrip("/")
        if namespace is None:
            return clean_suffix
        resolved_namespace = str(namespace).strip()
        if not resolved_namespace:
            return clean_suffix
        if resolved_namespace.startswith("/"):
            return f"{resolved_namespace.rstrip('/')}/{clean_suffix}"
        return f"{resolved_namespace.strip('/')}/{clean_suffix}"

    def vision_service_name(self, vision_node: object) -> str:
        if not self.has_camera:
            raise RuntimeError(
                f"Vehicle '{self.name}' does not expose a namespaced camera contract."
            )
        service_name = getattr(vision_node, "service_name", None)
        if callable(service_name):
            return str(service_name())
        return self.namespaced_path(f"vision/{self._vision_node_name(vision_node)}")

    @abstractmethod
    def stop(self) -> None:
        """Cease active motion in a target-appropriate way."""
