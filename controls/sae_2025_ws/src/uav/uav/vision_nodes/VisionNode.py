# from typing import Type
# from rclpy.type_support import Srv, SrvRequestT, SrvResponseT
from typing import Optional
from uav_interfaces.srv import CameraData
from sensor_msgs.msg import Image
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from abc import abstractmethod
from uav.utils import camel_to_snake

class VisionMeta(type):
    """
    Metaclass for VisionNode.
    """
    def __new__(cls, name, bases, dct):
        cls = super().__new__(cls, name, bases, dct)
        cls.__name__ = camel_to_snake(name)
        return cls

    def node_name(cls):
        return cls.__name__
    
    def __str__(cls):
        return cls.__name__
    
    def service_name(cls):
        return f'vision/{cls.__name__}'

class VisionNode(Node, metaclass=VisionMeta):
    """
    Base class for ROS 2 nodes that process vision data.
    Provides an interface for handling image streams, processing frames,
    and managing vision-based tasks such as tracking and calibration.
    """

    def __init__(self, custom_service, display: bool = False):
        """
        Initialize the VisionNode.

        Args:
            custom_service (Type[Srv[SrvRequestT, SrvResponseT]]): The custom service type.
            display (bool): Whether to display the image in a window. 
        """
        super().__init__(self.__class__.__name__)
        
        self.node_name = self.__class__.node_name()
        self.custom_service_type = custom_service
        self.service_name = self.__class__.service_name()


        self.client = self.create_client(CameraData, '/camera_data')

        self.image = None
        self.camera_info = None
        self.display = display

        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available.')
            return

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

    def request_data(self, cam_image: bool = False, cam_info: bool = False) -> CameraData.Response:
        """
        Sends request for camera image or camera information.
        """
        request = CameraData.Request()
        if not cam_info and not cam_image:
            return CameraData.Response()
        if cam_image:
            request.cam_image = self.convert_image_msg_to_frame(cam_image)
        if cam_info:
            request.cam_info = cam_info
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future) 
        try:
            response = future.result()
            if response.image:
                self.get_logger().info(f"Received image: {response.image}")
            if response.camera_info:
                self.get_logger().info(f"Received camera info: {response.camera_info}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        self.image = response.image
        self.camera_info = response.camera_info
        if self.display:
            self.display_frame(self.image, self.node_name)
        return response

    def display_frame(self, frame: np.ndarray, window_name: str) -> None:
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