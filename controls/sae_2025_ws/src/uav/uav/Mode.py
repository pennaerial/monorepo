from abc import ABC, abstractmethod
from rclpy.node import Node

class Mode(ABC):
    """
    Base class for UAV operational modes within a ROS 2 node.
    Provides a structured template for implementing autonomous behaviors.
    """

    def __init__(self, node: Node, clients: {}):
        """
        Initialize the mode with a reference to the ROS 2 node.

        Args:
            node (Node): The ROS 2 node instance managing the UAV and this mode.
            clients (dict): A dictionary of service clients.
        """
        self.node = node
        self.active = False

        for service_name, service_type in clients.items():
            self.initialize_client(service_type, service_name)

    @abstractmethod
    def on_enter(self) -> None:
        """
        Logic executed when this mode is activated.
        Should include any initialization required for the mode.
        """
        pass

    def initialize_client(self, service_type: Type[Srv[SrvRequestT, SrvResponseT]], service_name: str) -> None:
        """
        Create a client for a service.

        Args:
            service_type (Type[Srv[SrvRequestT, SrvResponseT]]): The type of the service.
            service_name (str): The name of the service.
        """
        self.client = self.create_client(service_type, service_name)

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.request = service_type.Request()

        self.future = self.client.call_async(self.request)

        self.future.add_done_callback(self.service_response_callback)

    @abstractmethod
    def service_response_callback(self, future: Future):
        """
        Callback for when a service response is received.
        """
        pass

    def setup_vision(self, vision_nodes: [VisionNode]) -> None:
        """
        Setup the vision node for this mode.

        Args:
            vision (VisionNode): The vision nodes to setup for this mode.
        """
        self.vision_nodes = vision_nodes

    @abstractmethod
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