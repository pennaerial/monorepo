import json

import apriltag
import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String


class AprilTagNode(Node):
    def __init__(self):
        super().__init__("apriltag_node")

        self.declare_parameter("image_topic", "/uav_0/camera")
        self.declare_parameter("camera_info_topic", "/uav_0/camera_info")
        self.declare_parameter("tag_size", 0.16)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.tag_size = float(self.get_parameter("tag_size").value)

        self.bridge = CvBridge()

        self.detector = apriltag.Detector(
            apriltag.DetectorOptions(families="tag36h11")
        )

        self.camera_params = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10,
        )

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10,
        )

        self.detection_pub = self.create_publisher(
            String,
            "/apriltag_detections",
            10,
        )

        self.get_logger().info(
            f"AprilTag node listening to {image_topic}"
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_params = [
            float(msg.k[0]),  # fx
            float(msg.k[4]),  # fy
            float(msg.k[2]),  # cx
            float(msg.k[5]),  # cy
        ]

    def image_callback(self, msg: Image):
        if self.camera_params is None:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="mono8",
            )
        except Exception as exc:
            self.get_logger().error(
                f"Could not convert image: {exc}"
            )
            return

        detections = self.detector.detect(image)
        results = []

        for detection in detections:
            pose, _, _ = self.detector.detection_pose(
                detection,
                self.camera_params,
                self.tag_size,
            )

            translation = pose[:3, 3]

            results.append({
                "id": int(detection.tag_id),
                "position": {
                    "x": float(translation[0]),
                    "y": float(translation[1]),
                    "z": float(translation[2]),
                },
            })

        output = String()
        output.data = json.dumps(results)
        self.detection_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)

    node = AprilTagNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
