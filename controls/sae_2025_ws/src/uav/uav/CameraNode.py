import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from uav_interfaces.srv import CameraData
from sensor_msgs.msg import Image
import cv2
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class CameraNode(Node):
    """
    Node class to respond to requests with camera image or camera information.
    """

    def __init__(
        self,
        node_name: str,
        service_name=None,
        image_topic=None,
        info_topic=None,
        queue_size: int = 10,
        display: bool = False,
    ):
        """
        Initialize the CameraNode. Topics/service are namespaced by vehicle_id for multi-vehicle.
        """
        super().__init__(node_name)
        self.declare_parameter("vehicle_id", 0)
        self._vehicle_id = self.get_parameter("vehicle_id").value

        prefix = f"/vehicle_{self._vehicle_id}"
        if service_name is None:
            service_name = f"{prefix}/camera_data"
        if image_topic is None:
            image_topic = f"{prefix}/camera"
        if info_topic is None:
            info_topic = f"{prefix}/camera_info"

        # ROS 2 Subscription
        self.image_subscription = self.create_subscription(
            Image, image_topic, self.image_callback, queue_size
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo, info_topic, self.camera_info_callback, queue_size
        )

        self.image = None
        self.camera_info = None
        self.display = display

        self.service = self.create_service(
            CameraData, service_name, self.service_callback
        )

        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.failsafe_publisher = self.create_publisher(
            Bool, "/failsafe_trigger", qos_profile
        )

        self.get_logger().info(
            f"{node_name} (vehicle_id={self._vehicle_id}) subscribing to {image_topic}, {info_topic}."
        )

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

    def service_callback(
        self, request: CameraData.Request, response: CameraData.Response
    ):
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

    def publish_failsafe(self):
        self.failsafe_publisher.publish(Bool(data=True))


def main():
    rclpy.init()
    try:
        node = CameraNode("camera_node")  # vehicle_id from param (launch)
    except Exception as e:
        print(e)
        node.publish_failsafe()
        return
    rclpy.spin(node)
    rclpy.shutdown()
