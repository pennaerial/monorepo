# payload_tracking_node.py
import cv2
import numpy as np
from uav.cv.find_hoop import find_hoop
from uav.vision_nodes.VisionNode import VisionNode
import rclpy
from cv_bridge import CvBridge
from uav.utils import pink, green, blue, yellow

class TemuVisionNode(VisionNode):
    """
    ROS node for hoop tracking with OpenCV Algorithm.
    """
    def __init__(self):
        super().__init__('hoop_tracking', display=False, use_service=False)
        self.bridge = CvBridge()


    def image_callback(self, msg):
        self.get_logger().info("Received image for hoop tracking.")
        self.sim = True
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        x, y, img = find_hoop(image)
        self.get_logger().info(str(type(img)))
        self.display_frame(img, "result")


def main():
    rclpy.init()
    node = TemuVisionNode()
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()

    rclpy.shutdown()
