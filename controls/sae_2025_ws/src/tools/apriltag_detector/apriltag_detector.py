import cv2
import apriltag
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from sim_interfaces.srv import GetAprilTagId
from std_msgs.msg import Int32MultiArray


class AprilTagDetector(Node):
    """Detect AprilTags in camera images and provide their id in a service."""
    def __init__(self) -> None:
        super().__init__("apriltag_detector")
        self.declare_parameter("image_topic", "/uav_0/camera")
        self.declare_parameter("ids_topic", "/uav_0/apriltag_ids")
        self.declare_parameter("get_id_service", "/get_apriltag_id")
        image_topic = str(self.get_parameter("image_topic").value)
        ids_topic = str(self.get_parameter("ids_topic").value)
        get_id_service = str(self.get_parameter("get_id_service").value)
        self.bridge = CvBridge()
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        self.publisher = self.create_publisher(Int32MultiArray, ids_topic, 10)
        self.subscription = self.create_subscription(
            Image, image_topic, self._image_callback, 10
        )
        self.service = self.create_service(
            GetAprilTagId, get_id_service, self._handle_get_apriltag_id
        )
        self.latest_image = None
        self.get_logger().info(
            f"Detecting tag36h11 from {image_topic}, serving {get_id_service}"
        )

    def _image_callback(self, message: Image) -> None:
        self.latest_image = message
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
        except Exception as exc:
            self.get_logger().error(f"Could not convert camera image: {exc}")
            return
        detections = self.detector.detect(image)
        output = Int32MultiArray()
        output.data = sorted({int(detection.tag_id) for detection in detections})
        self.publisher.publish(output)

    def _handle_get_apriltag_id(
        self, request: GetAprilTagId.Request, response: GetAprilTagId.Response
    ) -> GetAprilTagId.Response:
        if self.latest_image is None:
            self.get_logger().warn("GetAprilTagId called before any image was received")
            response.success = False
            response.tag_ids = []
            return response

        try:
            image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="mono8")
        except Exception as exc:
            self.get_logger().error(f"Could not convert camera image: {exc}")
            response.success = False
            response.tag_ids = []
            return response

        detections = self.detector.detect(image)
        response.success = True
        response.tag_ids = sorted({int(detection.tag_id) for detection in detections})
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()