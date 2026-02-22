# from typing import Type
# from rclpy.type_support import Srv, SrvRequestT, SrvResponseT
from uav_interfaces.srv import CameraData
from sensor_msgs.msg import Image, CameraInfo
import cv2
import os
import numpy as np
import uuid
import rclpy
from rclpy.node import Node
from uav.utils import camel_to_snake
from std_srvs.srv import Trigger


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
        return f"vision/{cls.node_name()}"

    @classmethod
    def __str__(cls):
        return cls.node_name()

    def __init__(
        self, custom_service, display: bool = False, use_service: bool = False
    ):
        """
        Initialize the VisionNode.

        Args:
            custom_service (Type[Srv[SrvRequestT, SrvResponseT]]): The custom service type.
            display (bool): Whether to display the image in a window.
            use_service (bool): Whether to use the custom CameraNode service.
        """
        super().__init__(self.__class__.__name__)

        self.declare_parameter("debug", False)
        self.debug = self.get_parameter("debug").value
        self.declare_parameter("sim", True)
        self.sim = self.get_parameter("sim").value
        self.declare_parameter("save_vision", False)
        self.save_vision = self.get_parameter("save_vision").value
        self.declare_parameter("vehicle_id", 0)
        self.vehicle_id = int(self.get_parameter("vehicle_id").value)
        self.custom_service_type = custom_service
        self.uuid = str(uuid.uuid4())
        if self.save_vision:
            os.makedirs(os.path.expanduser(f"~/vision_imgs/{self.uuid}"), exist_ok=True)

        self.use_service = use_service
        cam_prefix = f"/vehicle_{self.vehicle_id}"

        if use_service:
            self.client = self.create_client(CameraData, f"{cam_prefix}/camera_data")

            if not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("Service not available.")
                return
        else:
            self.image_subscription = self.create_subscription(
                Image, f"{cam_prefix}/camera", self.image_callback, 10
            )

            self.camera_info_subscription = self.create_subscription(
                CameraInfo, f"{cam_prefix}/camera_info", self.camera_info_callback, 10
            )

            self.image = None
            self.camera_info = None

        if not self.sim:
            from cv_bridge import CvBridge

            self.bridge = CvBridge()
        self.display = display
        self.failsafe_trigger_client = self.create_client(
            Trigger, f"/mode_manager/vehicle_{self.vehicle_id}/failsafe"
        )

    def image_callback(self, msg: Image):
        """
        Callback for receiving image requests.
        """
        self.image = msg

    def camera_info_callback(self, msg: CameraInfo):
        """
        Callback for receiving camera info messages. Stores the camera info
        for later use.

        Args:
            msg (CameraInfo): The ROS 2 CameraInfo message.
        """
        self.camera_info = msg

    def convert_image_msg_to_frame(self, msg: Image) -> np.ndarray:
        """
        Converts a ROS 2 Image message to a NumPy array.

        Args:
            msg (Image): The ROS 2 Image message.

        Returns:
            np.ndarray: The decoded image as a NumPy array.
        """
        if self.sim:
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            frame = img_data.reshape(
                (msg.height, msg.width, 3)
            )  # Assuming BGR8 encoding
        else:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return frame

    def send_req(self, req):
        return self.client.call_async(req)

    def request_data(
        self, cam_image: bool = False, cam_info: bool = False
    ) -> CameraData.Response:
        """
        Sends request for camera image or camera information.
        """
        if not self.use_service:
            response = CameraData.Response()
            if cam_image:
                response.image = self.image
            if cam_info:
                response.camera_info = self.camera_info
        else:
            self.get_logger().info("Requesting camera data from service...")
            request = CameraData.Request()
            if not cam_info and not cam_image:
                return CameraData.Response()
            request.cam_image = cam_image
            request.cam_info = cam_info

            future = self.send_req(request)
            self.get_logger().info("Sending request to CameraNode...")
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info("Request sent.")
            try:
                response = future.result()
                if response.image:
                    self.get_logger().info(f"Received image: {response.image}")
                if response.camera_info:
                    self.get_logger().info(
                        f"Received camera info: {response.camera_info}"
                    )
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
            if self.display:
                self.display_frame(
                    self.convert_image_msg_to_frame(response.image), self.node_name()
                )
        return response.image, response.camera_info

    def display_frame(self, frame: np.ndarray, window_name: str) -> None:
        """
        Displays the given frame using OpenCV.

        Args:
            frame (np.ndarray): The image frame to display.
            window_name (str): The name of the display window.
        """
        cv2.imshow(window_name, frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Shutting down display.")
            rclpy.shutdown()
            cv2.destroyAllWindows()

    def cleanup(self):
        """
        Cleanup resources, such as OpenCV windows.
        """
        cv2.destroyAllWindows()

    def publish_failsafe(self):
        future = self.failsafe_trigger_client.call_async(Trigger.Request(data=True))
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info("Failsafe triggered.")
        else:
            self.get_logger().error("Failed to trigger failsafe.")
