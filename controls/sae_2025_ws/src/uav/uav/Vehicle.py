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
            self.camera_info_topic = (
                camera_info_topic or self._default_camera_path("camera_info")
            )
            self.camera_service_name = (
                camera_service_name or self._default_camera_path("camera_data")
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
        if hasattr(vision_node, "node_name"):
            return vision_node.node_name()
        if hasattr(vision_node, "__name__"):
            return vision_node.__name__
        return str(vision_node)

    def _default_camera_path(self, suffix: str) -> str | None:
        if not self.has_camera or not self.camera_namespace:
            return None
        return self.namespaced_path(suffix)

    @property
    def logger(self):
        return self.node.get_logger()

    def namespaced_path(self, suffix: str, *, namespace: str | None = None) -> str:
        resolved_namespace = self._normalize_namespace(namespace) or self.camera_namespace
        clean_suffix = suffix.lstrip("/")
        if not resolved_namespace:
            return f"/{clean_suffix}"
        return f"{resolved_namespace}/{clean_suffix}"

    def vision_service_name(self, vision_node: object) -> str:
        if not self.has_camera or not self.camera_namespace:
            raise RuntimeError(
                f"Vehicle '{self.name}' does not expose a namespaced camera contract."
            )
        if hasattr(vision_node, "service_name"):
            return vision_node.service_name(self.camera_namespace)
        return self.namespaced_path(f"vision/{self._vision_node_name(vision_node)}")

    @abstractmethod
    def stop(self) -> None:
        """Cease active motion in a target-appropriate way."""
