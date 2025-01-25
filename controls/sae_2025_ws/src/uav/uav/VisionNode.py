import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from typing import Type
from rclpy.type_support import Srv, SrvRequestT, SrvResponseT
import cv2
import numpy as np
from abc import ABC, abstractmethod


class VisionNode(Node, ABC):
    """
    Base class for ROS 2 nodes that process vision data.
    Provides an interface for handling image streams, processing frames,
    and managing vision-based tasks such as tracking and calibration.
    """

    def __init__(self, node_name: str, image_topic: str = '/camera', queue_size: int = 10):
        """
        Initialize the VisionNode.

        Args:
            node_name (str): The name of the ROS 2 node.
            image_topic (str): The name of the image topic to subscribe to.
            queue_size (int): The size of the message queue for the subscription.
        """
        super().__init__(node_name)

        # ROS 2 Subscription TODO: replace with service
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.listener_callback,
            queue_size
        )

        self.get_logger().info(f"{node_name} has started, subscribing to {image_topic}.")

        # Internal state
        self.prev_frame = None

        self.curr_frame = None

        self.threshold_range = None

    def request_image(self):
        pass

    @abstractmethod
    def service_callback(self, request: SrvRequestT, response: SrvResponseT):
        """
        Callback for receiving image messages. Converts the image data
        and processes the frame.

        Args:
            msg (Image): The ROS 2 Image message.
        """

        pass

    def initialize_service(self, custom_service: Type[Srv[SrvRequestT, SrvResponseT]], service_name: str):
        self.service = self.create_service(
            custom_service,
            service_name,
            self.service_callback
        )

        self.get_logger().info(f"{node_name} has started, subscribing to {service_name}.")

    def display_frame(self, frame: np.ndarray, window_name: str = "Camera Feed") -> None:
        """
        Displays the given frame using OpenCV.

        Args:
            frame (np.ndarray): The image frame to display.
            window_name (str): The name of the display window.
        """
        cv2.imshow(window_name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutting down display.")
            rclpy.shutdown()
            cv2.destroyAllWindows()

    def cleanup(self):
        """
        Cleanup resources, such as OpenCV windows.
        """
        cv2.destroyAllWindows()