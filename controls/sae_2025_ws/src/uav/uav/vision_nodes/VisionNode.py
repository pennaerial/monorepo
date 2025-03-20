# from typing import Type
# from rclpy.type_support import Srv, SrvRequestT, SrvResponseT
from typing import Optional
from uav_interfaces.srv import CameraData
from sensor_msgs.msg import Image
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from uav.utils import camel_to_snake

class VisionNode(Node):
    """
    Base class for ROS 2 nodes that process vision data.
    Provides an interface for handling image streams, processing frames,
    and managing vision-based tasks such as tracking and calibration.
    """

    @classmethod
    def node_name(cls):
        return camel_to_snake(cls.__name__)

    @classmethod
    def service_name(cls):
        return f'vision/{cls.node_name()}'
    
    @classmethod
    def __str__(cls):
        return cls.node_name()

    def __init__(self, custom_service, display: bool = False, use_service: bool = False):
        """
        Initialize the VisionNode.

        Args:
            custom_service (Type[Srv[SrvRequestT, SrvResponseT]]): The custom service type.
            display (bool): Whether to display the image in a window. 
            use_service (bool): Whether to use the custom CameraNode service.
        """
        super().__init__(self.__class__.__name__)

        self.image = None
        
        self.custom_service_type = custom_service

        self.use_service = use_service

        if use_service:
            self.client = self.create_client(CameraData, '/camera_data')

            if not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Service not available.')
                return

        else:
            self.image_subscription = self.create_subscription(
                CameraData,
                '/camera_node_data',
                self.image_callback,
                10
            )

        self.display = display

        
    def image_callback(self, msg: CameraData):
        self.image = msg.image
        self.camera_info = msg.camera_info

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
    
    def send_req(self, req):
        return self.client.call_async(req)

    def request_data(self, cam_image: bool = False, cam_info: bool = False) -> CameraData.Response:
        """
        Sends request for camera image or camera information.
        """

        if not self.use_service:
            if self.image and self.camera_info:
                return self.image, self.camera_info
            else:
                self.get_logger().info('No camera data available.')
                return

        self.get_logger().info('Requesting camera data from service...')
        request = CameraData.Request()
        if not cam_info and not cam_image:
            return CameraData.Response()
        request.cam_image = cam_image
        request.cam_info = cam_info
            
        future = self.send_req(request)
        self.get_logger().info('Sending request to CameraNode...')
        rclpy.spin_until_future_complete(self, future) 
        self.get_logger().info('Request sent.')
        try:
            response = future.result()
            if response.image:
                self.get_logger().info(f"Received image: {response.image}")
            if response.camera_info:
                self.get_logger().info(f"Received camera info: {response.camera_info}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        if self.display:
            self.display_frame(self.convert_image_msg_to_frame(response.image), self.node_name())
        return response.image, response.camera_info

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