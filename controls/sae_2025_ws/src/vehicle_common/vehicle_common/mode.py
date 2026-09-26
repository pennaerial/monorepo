from abc import ABC, abstractmethod
from typing import ClassVar, Mapping

from pydantic import BaseModel
from rclpy.node import Node

from vehicle_common.vehicle import Vehicle


class Mode[VehicleT: Vehicle, ParamsT: BaseModel](ABC):
    """
    Base class for UAV operational modes within a ROS 2 node.
    Provides a structured template for implementing autonomous behaviors.
    """

    peer_vehicle_names: ClassVar[tuple[str, ...]] = ()
    transition_labels: ClassVar[tuple[str, ...]] = ()

    # self attributes
    active = False

    @abstractmethod
    def initialize(self, node: Node, vehicle: VehicleT, params: ParamsT) -> None:
        """Abstract method for initializing a mode

        Args:
            node: The ROS 2 node that gets passed into the mode
            vehicle: The vehicle instance that the current mode has control over
            params: A validated pydantic params model that the mode class has defined
        """

    @classmethod
    def declared_transition_labels(cls) -> tuple[str, ...]:
        return tuple(cls.transition_labels)

    def on_enter(self) -> None:
        """
        Logic executed when this mode is activated.
        Should include any initialization required for the mode.
        """
        pass

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

    def on_disconnect(self, time_delta: float, connection_status: Mapping[str, bool]) -> None:
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

    def disconnect(self, time_delta: float, connection_status: Mapping[str, bool]) -> None:
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
