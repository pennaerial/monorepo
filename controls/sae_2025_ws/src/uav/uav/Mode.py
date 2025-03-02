from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
# from rclpy.type_support import SrvRequestT
from typing import Type
from rclpy.task import Future
from typing import List
from uav.vision_nodes import VisionNode
from uav import UAV

class Mode(ABC):
    """
    Base class for UAV operational modes within a ROS 2 node.
    Provides a structured template for implementing autonomous behaviors.
    """

    def __init__(self, node: Node, uav: UAV, vision_nodes: List[VisionNode]):
        """
        Initialize the mode with a reference to the ROS 2 node.

        Args:
            node (Node): The ROS 2 node instance managing the UAV and this mode.
            uav (UAV): The UAV instance to control.
            vision_nodes (List[VisionNode]): The vision nodes to setup for this mode.
        """
        self.node = node
        self.active = False
        self.uav = uav

        self.vision_clients = {}
        self.vision_nodes = {}
        self._vision_nodes_list = vision_nodes

    def on_enter(self) -> None:
        """
        Logic executed when this mode is activated.
        Should include any initialization required for the mode.
        """
        pass

    def send_request(self, vision_node_name: str, req) -> None:
        """
        Send a request to a service.

        Args:
            request (SrvRequestT): The request to send.
            service_name (str): The name of the service.
        """
        vision_node = self.vision_nodes[vision_node_name]
        assert type(req) == vision_node.custom_service_type.Request

        client = self.vision_clients[vision_node.service_name]

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

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

    def log(self, message: str) -> None:
        """
        Log a message using the ROS 2 node's logger.

        Args:
            message (str): The message to log.
        """
        self.node.get_logger().info(f"[{self.__class__.__name__}] {message}")