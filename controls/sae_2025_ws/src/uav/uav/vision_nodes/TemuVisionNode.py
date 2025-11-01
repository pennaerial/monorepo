# payload_tracking_node.py
import cv2
import numpy as np
from uav.cv.find_hoop import find_hoop
from uav.vision_nodes import VisionNode
import rclpy
from uav.utils import pink, green, blue, yellow

class TemuVisionNode(VisionNode):
    """
    ROS node for hoop tracking with OpenCV Algorithm.
    """
    def __init__(self):
        super().__init__('hoop_tracking', display=False, use_service=False)

    def image_callback(self, msg):
        image = self.convert_image_msg_to_frame(msg)
        x, y, img = find_hoop(image)
        cv2.imshow("Result", img)


def main():
    rclpy.init()
    node = TemuVisionNode()
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()

    rclpy.shutdown()
