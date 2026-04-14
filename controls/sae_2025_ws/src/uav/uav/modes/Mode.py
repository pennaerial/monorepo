from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, ClassVar

from rclpy.node import Node

from uav.vehicles.Vehicle import Vehicle
from uav.runtime.vision_loader import canonical_vision_node_path

if TYPE_CHECKING:
    from uav.vision_nodes import VisionNode


class Mode(ABC):
    """
    Base class for UAV operational modes within a ROS 2 node.
    Provides a structured template for implementing autonomous behaviors.
    """

    mission_target: ClassVar[str | None] = None
    required_vision_nodes: ClassVar[tuple[object, ...]] = ()
    peer_vehicle_names: ClassVar[tuple[str, ...]] = ()
    requires_camera: ClassVar[bool] = False
    transition_labels: ClassVar[tuple[str, ...]] = ()

    def __init__(self, node: Node, vehicle: Vehicle):
        """
        Initialize the mode with a reference to the ROS 2 node.

        Args:
            node (Node): The ROS 2 node instance managing this mode.
            vehicle (Vehicle): The controlled entity.
        """
        self.node = node
        self.active = False
        self.vehicle: Vehicle = vehicle
        self.pending_requests = {}

    @classmethod
    def required_vision_node_paths(cls) -> tuple[str, ...]:
        return tuple(
            canonical_vision_node_path(node) for node in cls.required_vision_nodes
        )

    @classmethod
    def declared_transition_labels(cls) -> tuple[str, ...]:
        return tuple(cls.transition_labels)

    def on_enter(self) -> None:
        """
        Logic executed when this mode is activated.
        Should include any initialization required for the mode.
        """
        pass

    def send_request(self, vision_node: type["VisionNode"], request):
        """
        Send a request to a service.

        Args:
            request (SrvRequestT): The request to send.
            service_name (VIsionNode): The name of the service.
        """
        service_name = self.vehicle.vision_service_name(vision_node)
        future = self.pending_requests.get(service_name)
        if future is None:
            client = self.node.get_vision_client(vision_node)
            self.pending_requests[service_name] = client.call_async(request)
            return None

        if not future.done():
            return None

        response = future.result()
        self.pending_requests.pop(service_name, None)
        assert type(response) is vision_node.srv.Response, (
            f"Expected response type {vision_node.srv.Response}, got {type(response)}."
        )
        return response

    def on_exit(self) -> None:
        """
        Logic executed when this mode is deactivated.
        Should include any cleanup required for the mode.
        """
        pass

    @abstractmethod
    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic executed while the mode is active.
        This should implement the mode's core behavior.

        Args:
            time_delta (float): Time in seconds since the last update.
        """
        pass

    def on_disconnect(
        self, time_delta: float, disconnected_peers: tuple[str, ...]
    ) -> None:
        """
        Periodic logic executed while one or more required peers are disconnected.

        Args:
            time_delta (float): Time in seconds since the last update.
            disconnected_peers (tuple[str, ...]): Required peers currently disconnected.
        """
        pass

    @abstractmethod
    def check_status(self) -> str:
        """
        Check if the mode should deactivate.
        """
        pass

    def activate(self) -> None:
        """
        Activate the mode. Calls the `on_enter` method.
        """
        self.active = True
        self.node.get_logger().info(f"Activating mode: {self.__class__.__name__}")
        self.on_enter()

    def deactivate(self) -> None:
        """
        Deactivate the mode. Calls the `on_exit` method.
        """
        self.active = False
        self.pending_requests.clear()
        self.node.get_logger().info(f"Deactivating mode: {self.__class__.__name__}")
        self.on_exit()

    def update(self, time_delta: float) -> None:
        """
        Update the mode if it is active. Calls the `on_update` method.

        Args:
            time_delta (float): Time in seconds since the last update.
        """
        if self.active:
            self.on_update(time_delta)

    def disconnect(
        self, time_delta: float, disconnected_peers: tuple[str, ...]
    ) -> None:
        """
        Update the mode's disconnected behavior if it is active.

        Args:
            time_delta (float): Time in seconds since the last update.
            disconnected_peers (tuple[str, ...]): Required peers currently disconnected.
        """
        if self.active:
            self.on_disconnect(time_delta, disconnected_peers)

    def log(self, message: str) -> None:
        """
        Log a message using the ROS 2 node's logger.

        Args:
            message (str): The message to log.
        """
        self.node.get_logger().info(f"[{self.__class__.__name__}] {message}")
