from abc import ABC, abstractmethod
from typing import ClassVar, Mapping

from rclpy.node import Node
from pydantic import BaseModel

from vehicle_common.vehicle import Vehicle
from vehicle_common.runtime.vision_loader import canonical_vision_node_path

class Mode[VehicleT: Vehicle, ParamsT: BaseModel](ABC):
    """
    Base class for UAV operational modes within a ROS 2 node.
    Provides a structured template for implementing autonomous behaviors.
    """

    required_vision_nodes: ClassVar[tuple[object, ...]] = ()
    peer_vehicle_names: ClassVar[tuple[str, ...]] = ()
    requires_camera: ClassVar[bool] = False
    transition_labels: ClassVar[tuple[str, ...]] = ()

    # self attributes
    active = False
    pending_requests = {}

    @abstractmethod
    def initialize(self, node: Node, vehicle: VehicleT, params: ParamsT) -> None:
        """Abstract method for initializing a mode

        Args:
            node: The ROS 2 node that gets passed into the mode
            vehicle: The vehicle instance that the current mode has control over
            params: A validated pydantic params model that the mode class has defined
        """

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
            future = client.call_async(request)
            if future is None:
                return None
            self.pending_requests[service_name] = future
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
        self, time_delta: float, connection_status: Mapping[str, bool]
    ) -> None:
        """
        Periodic logic executed while one or more required peers are disconnected.

        Args:
            time_delta (float): Time in seconds since the last update.
            connection_status (Mapping[str, bool]): Current mission peer connection map.
        """
        pass

    def connection_ready(self, connection_status: Mapping[str, bool]) -> bool:
        """
        Return whether the mode has enough peer connectivity to run `on_update()`.

        `connection_status` contains only this mode's relevant remote peers.
        The default implementation treats every provided peer as required.
        """
        if not connection_status:
            return True
        return all(bool(is_connected) for is_connected in connection_status.values())

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
        self, time_delta: float, connection_status: Mapping[str, bool]
    ) -> None:
        """
        Update the mode's disconnected behavior if it is active.

        Args:
            time_delta (float): Time in seconds since the last update.
            connection_status (Mapping[str, bool]): Current mission peer connection map.
        """
        if self.active:
            self.on_disconnect(time_delta, connection_status)

    def log(self, message: str) -> None:
        """
        Log a message using the ROS 2 node's logger.

        Args:
            message (str): The message to log.
        """
        self.node.get_logger().info(f"[{self.__class__.__name__}] {message}")
