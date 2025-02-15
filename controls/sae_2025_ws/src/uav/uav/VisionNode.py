from typing import Type
from rclpy.type_support import Srv, SrvRequestT, SrvResponseT
from typing import Optional
from uav.src import CameraData
import cv2
import numpy as np
from abc import ABC, abstractmethod

class VisionNode(Node):
    """
    Base class for ROS 2 nodes that process vision data.
    Provides an interface for handling image streams, processing frames,
    and managing vision-based tasks such as tracking and calibration.
    """

    def __init__(self, node_name: str, custom_service: Type[Srv[SrvRequestT, SrvResponseT]], service_name: Optional[str]):
        """
        Initialize the VisionNode.

        Args:
            node_name (str): The name of the ROS 2 node.
            custom_service (Type[Srv[SrvRequestT, SrvResponseT]]): The custom service type.
            service_name (Optional[str]): The name of the ROS 2 service. Defaults to 'vision/{node_name}'.
        """
        super().__init__(node_name)

        if service_name is None:
            service_name = f'vision/{node_name}'
        
        self.node_name = node_name
        self.custom_service_type = custom_service
        self.service_name = service_name

        self.initialize_service(custom_service, service_name)

        self.client = self.create_client(CameraData, service_name)

        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available.')
            return


    def request_image(self, cam_image: bool = False, cam_info: bool = False) -> CameraData.Response:
        """
        Sends request for camera image or camera information.
        """
        request = CameraData.Request()
        if not cam_info and not cam_image:
            return CameraData.Response()
        if cam_image:
            request.cam_image = cam_image
        if cam_info:
            request.cam_info = cam_info
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future) 
        try:
            response = future.result()
            if response.processed_image:
                self.get_logger().info(f"Received processed image: {response.processed_image}")
            if response.camera_info:
                self.get_logger().info(f"Received camera info: {response.camera_info}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        return response

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