import rclpy
from rclpy.node import Node
from uav.src import ImageSend
import cv2
import numpy as np

class VisionNode(Node):
    """
    Base class for ROS 2 nodes that process vision data.
    Provides an interface for handling image streams, processing frames,
    and managing vision-based tasks such as tracking and calibration.
    """

    def __init__(self, node_name: str, camera_node_service_name: str):
        """
        Initialize the VisionNode.

        Args:
            node_name (str): The name of the ROS 2 node.
            custom_service (Type[Srv[SrvRequestT, SrvResponseT]]): The custom service type.
            service_name (Optional[str]): The name of the ROS 2 service. Defaults to 'vision/{node_name}'.
            image_topic (str): The name of the image topic to subscribe to.
            queue_size (int): The size of the message queue for the subscription.
        """
        super().__init__(node_name)

        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available.')
            return

        self.client = self.create_client(ImageSend, camera_node_service_name)

    def request_image(self, cam_image: bool = False, cam_info: bool = False):
        """
        Sends request for camera image or camera information.
        """
        request = ImageSend.Request()
        request.cam_image = cam_image
        request.cam_info = cam_info
        
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        """
        Handles the response from the service.
        """
        try:
            response = future.result()
            if response.processed_image:
                self.get_logger().info(f"Received processed image: {response.processed_image}")
            if response.camera_info:
                self.get_logger().info(f"Received camera info: {response.camera_info}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

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