from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from uav import UAV

class Mode(ABC):
    """
    Base class for UAV operational modes within a ROS 2 node.
    Provides a structured template for implementing autonomous behaviors.
    """

    def __init__(self, node: Node, uav: UAV):
        """
        Initialize the mode with a reference to the ROS 2 node.

        Args:
            node (Node): The ROS 2 node instance managing the UAV and this mode.
            uav (UAV): The UAV instance to control.
        """
        self.node = node
        self.active = False
        self.uav: UAV = uav
        self.vision_subscriptions = {}
        self.latest_vision_data = {}

    def on_enter(self) -> None:
        """
        Logic executed when this mode is activated.
        Should include any initialization required for the mode.
        """
        pass
    
    def subscribe_to_vision(self, topic: str, msg_type, key: str = None):
        """
        Subscribe to a vision topic and store the latest message.
        
        Args:
            topic (str): The topic to subscribe to.
            msg_type: The message type.
            key (str): Optional key for storing the data. Defaults to topic name.
        """
        if key is None:
            key = topic
        
        def callback(msg):
            self.latest_vision_data[key] = msg
        
        sub = self.node.create_subscription(msg_type, topic, callback, 10)
        self.vision_subscriptions[key] = sub
    
    def get_vision_data(self, key: str):
        """
        Get the latest vision data for a given key.
        
        Args:
            key (str): The key for the vision data.
            
        Returns:
            The latest message, or None if no data received yet.
        """
        return self.latest_vision_data.get(key, None)

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