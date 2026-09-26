from typing import override

import cv2
import rclpy

from rclpy.executors import ExternalShutdownException

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class BasicVision(LifecycleNode):
    """Example implementation of a python vision node managed by VisionManager"""

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.camera_topic: str = str(self.get_parameter("camera_topic").value)
        self.bridge = CvBridge()
        self.camera_sub = None
        self.count_pub = None
        self.debug_image_pub = None
        self.image_count = 0  # counts how many images have been received over total lifetime
        self.get_logger().info("started!")

    # Don't need on_configure here
    # @override
    # def on_configure(self, state):
    #     self.get_logger().info("Configuring")
    #     return TransitionCallbackReturn.SUCCESS

    @override
    def on_activate(self, state):
        self.get_logger().info("Activating")
        self.camera_sub = self.create_subscription(Image, self.camera_topic, self.process_frame, 10)
        self.count_pub = self.create_publisher(Int32, "/basic_vision_py/image_count", 10)
        self.debug_image_pub = self.create_publisher(Image, "basic_vision_py/debug", 10)
        return TransitionCallbackReturn.SUCCESS

    @override
    def on_deactivate(self, state):
        self.get_logger().info("Deactivating")

        # destroy camera subscriber
        if self.camera_sub is not None:
            self.destroy_subscription(self.camera_sub)
            self.camera_sub = None

        return TransitionCallbackReturn.SUCCESS

    @override
    def on_cleanup(self, state):
        self.get_logger().info("Cleaning up")

        if self.camera_sub is not None:
            self.destroy_subscription(self.camera_sub)
            self.camera_sub = None

        # destroy image count publisher
        if self.count_pub is not None:
            self.destroy_publisher(self.count_pub)
            self.count_pub = None

        if self.debug_image_pub is not None:
            self.destroy_publisher(self.debug_image_pub)
            self.debug_image_pub = None

        self.image_count = 0

        return TransitionCallbackReturn.SUCCESS

    @override
    def on_shutdown(self, state):
        return self.on_cleanup(state)

    # process frames here
    def process_frame(self, image: Image):
        self.image_count += 1

        if self.count_pub is None or self.debug_image_pub is None:
            self.get_logger().warn("publishers were not initialized correctly!")
            return

        # publish self.image_count
        count_msg = Int32()
        count_msg.data = self.image_count
        self.count_pub.publish(count_msg)

        # publish grayscaled image
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        msg = self.bridge.cv2_to_imgmsg(gray, encoding="mono8")
        msg.header = image.header
        self.debug_image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BasicVision("basic_vision_py")
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
