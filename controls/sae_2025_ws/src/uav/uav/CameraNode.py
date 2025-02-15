import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from uav.src import CameraData
import cv2


class CameraNode(Node):
    """
    Node class to respond to requests with camera image or camera information.
    """

    def __init__(self, node_name: str, service_name: str = '/camera_data', image_topic: str = '/camera', info_topic: str = '/camera_info', queue_size: int = 10):
        """
        Initialize the CameraNode.

        Args:
            node_name (str): The name of the ROS 2 node.
            service_name (Optional[str]): The name of the ROS 2 service. Defaults to 'vision/{node_name}'.
            image_topic (str): The name of the image topic to subscribe to.
            info_topic (str): The name of the image info topic to subscribe to.
            queue_size (int): The size of the message queue for the subscription.
        """
        super().__init__(node_name)

        # ROS 2 Subscription
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            queue_size
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            info_topic,
            self.camera_info_callback,
            queue_size
        )
        
        self.processed_image = None
        self.camera_info = None

        self.service = self.create_service(
            CameraData,
            service_name,
            self.service_callback
        )

        self.get_logger().info(f"{node_name} has started, subscribing to {image_topic}.")
        self.get_logger().info(f"{node_name} has started, subscribing to {info_topic}.")

    def image_callback(self, msg: Image):
        """
        Callback for receiving image requests. Converts the image data
        and processes the frame.
        """
        try:
            frame = self.convert_image_msg_to_frame(msg)
            self.processed_image = frame
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")
    
    def camera_info_callback(self, msg: CameraInfo):
        """
        Callback for receiving camera info messages. Stores the camera info
        for later use.

        Args:
            msg (CameraInfo): The ROS 2 CameraInfo message.
        """
        self.camera_info = msg

    def service_callback(self, request: CameraData.Request, response: CameraData.Response):
        """
        Callback for receiving image messages. Converts the image data
        and processes the frame.

        Args:
            msg (Image): The ROS 2 Image message.
        """

        if request.cam_image:
            if self.processed_image:
                processed_image_data = self.processed_image.tolist()
                response.processed_image = processed_image_data
            else:
                self.get_logger().warn("No processed image available.")
        
        if request.cam_info:
            if self.camera_info:
                response.camera_info = self.camera_info
            else:
                self.get_logger().warn("No camera info available.")

        return response

    def convert_image_msg_to_frame(self, msg: Image) -> np.ndarray:
        """
        Converts a ROS 2 Image message to a NumPy array.

        Args:
            msg (Image): The ROS 2 Image message.

        Returns:
            np.ndarray: The decoded image as a NumPy array.
        """
        img_data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = img_data.reshape((msg.height, msg.width, 3))  # Assuming BGR8 encoding
        return frame
