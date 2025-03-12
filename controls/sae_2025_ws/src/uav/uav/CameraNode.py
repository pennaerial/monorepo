import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from uav_interfaces.srv import CameraData
from sensor_msgs.msg import Image
import cv2

class CameraNode(Node):
    """
    Node class to respond to requests with camera image or camera information.
    """

    def __init__(self, node_name: str, service_name: str = '/camera_data', image_topic: str = '/camera', info_topic: str = '/camera_info', queue_size: int = 10, display: bool = False):
        """
        Initialize the CameraNode.

        Args:
            node_name (str): The name of the ROS 2 node.
            service_name (Optional[str]): The name of the ROS 2 service. Defaults to 'vision/{node_name}'.
            image_topic (str): The name of the image topic to subscribe to.
            info_topic (str): The name of the image info topic to subscribe to.
            queue_size (int): The size of the message queue for the subscription.
            display (bool): Whether to display the image in a window.
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
        
        self.image = None
        self.camera_info = None
        self.display = display

        self.service = self.create_service(
            CameraData,
            service_name,
            self.service_callback
        )

        self.get_logger().info(f"{node_name} has started, subscribing to {image_topic}.")
        self.get_logger().info(f"{node_name} has started, subscribing to {info_topic}.")

    def image_callback(self, msg: Image):
        """
        Callback for receiving image requests. 
        """
        self.image = msg
        if self.display:
            frame = self.convert_image_msg_to_frame(msg)
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)
    
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
        Callback for receiving image messages. 

        Args:
            msg (Image): The ROS 2 Image message.
        """
        self.get_logger().info("Received request for camera data.")

        if request.cam_image:
            if self.image is not None:
                response.image = self.image
            else:
                self.get_logger().warn("No image available.")
        
        if request.cam_info is not None:
            if self.camera_info:
                response.camera_info = self.camera_info
            else:
                self.get_logger().warn("No camera info available.")
        self.get_logger().info("Sending camera data.")
        return response
    
def main():
    rclpy.init()
    node = CameraNode("camera_node")
    rclpy.spin(node)
    rclpy.shutdown()