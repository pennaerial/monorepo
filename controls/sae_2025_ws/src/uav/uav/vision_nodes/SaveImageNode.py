# payload_tracking_node.py
import cv2
import numpy as np
from uav.vision_nodes.VisionNode import VisionNode
import rclpy
from cv_bridge import CvBridge
from uav.utils import pink, green, blue, yellow
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Float64MultiArray

from std_srvs.srv import Empty

from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class SaveImageNode(VisionNode):
    """
    ROS node for hoop tracking with OpenCV Algorithm.
    """
    srv = Empty
    def __init__(self):
        super().__init__('save_vision', display=False, use_service=False)
        self.bridge = CvBridge()

        # add an empty service server for ModeManager
        self.create_service(self.srv, self.service_name(), self.service_callback)
        self.get_logger().info(f"save_vision empty service '{self.service_name()}' started.")        


    def service_callback(self, request, response):
        self.get_logger().info("Empty service called (placeholder).")
        return response
    def image_callback(self, msg):
        # self.get_logger().info("Received image for hoop tracking.")
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        import time
        import os
        time = int(time.time())
        path = os.path.expanduser(f"~/vision_imgs/{image}")
        cv2.imwrite(os.path.join(path, f"payload_{time}.png"), image)

def main():
    rclpy.init()
    node = SaveImageNode()
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()

    rclpy.shutdown()