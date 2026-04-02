import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge


class ImageRotate(Node):
    def __init__(self):
        super().__init__("image_rotate")
        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/camera/image_rotated")
        self.declare_parameter("compressed", False)
        self.declare_parameter("degrees", 0.0)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.compressed = (
            self.get_parameter("compressed").get_parameter_value().bool_value
        )
        self.degrees = (
            self.get_parameter("degrees").get_parameter_value().double_value
        )

        self.bridge = CvBridge()

        if self.compressed:
            self.pub = self.create_publisher(CompressedImage, output_topic, 10)
            self.create_subscription(
                CompressedImage, input_topic, self._compressed_cb, 10
            )
            self.get_logger().info(
                f"Rotating compressed stream {input_topic} -> {output_topic} by {self.degrees}°"
            )
        else:
            self.pub = self.create_publisher(Image, output_topic, 10)
            self.create_subscription(Image, input_topic, self._image_cb, 10)
            self.get_logger().info(
                f"Rotating raw stream {input_topic} -> {output_topic} by {self.degrees}°"
            )

    def _rotate(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        center = (w / 2, h / 2)
        # Positive degrees = clockwise (negate for cv2 which is counter-clockwise)
        M = cv.getRotationMatrix2D(center, -self.degrees, 1.0)
        cos, sin = abs(M[0, 0]), abs(M[0, 1])
        new_w = int(h * sin + w * cos)
        new_h = int(h * cos + w * sin)
        M[0, 2] += (new_w - w) / 2
        M[1, 2] += (new_h - h) / 2
        return cv.warpAffine(frame, M, (new_w, new_h))

    def _image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        rotated = self._rotate(frame)
        out = self.bridge.cv2_to_imgmsg(rotated, encoding=msg.encoding)
        out.header = msg.header
        self.pub.publish(out)

    def _compressed_cb(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv.imdecode(buf, cv.IMREAD_COLOR)
        rotated = self._rotate(frame)
        ok, enc = cv.imencode(".jpg", rotated)
        if not ok:
            self.get_logger().error("Failed to encode rotated frame")
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = "rgb8; jpeg compressed bgr8"
        out.data = enc.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImageRotate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
