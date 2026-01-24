# payload_tracking_node.py
import cv2
import numpy as np
from uav.vision_nodes.VisionNode import VisionNode
import rclpy
from cv_bridge import CvBridge
from uav.utils import pink, green, blue, yellow
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Float64MultiArray
import time
import os
from std_srvs.srv import Empty

from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class SaveImageNode(VisionNode):
    """
    ROS node for hoop tracking with OpenCV Algorithm.
    """
    srv = Empty
    def __init__(self):
        super().__init__('save_image_node', display=False, use_service=False)
        self.bridge = CvBridge()
        self.begin_time = time.time()
        self.time_elapsed = self.begin_time
        # add an empty service server for ModeManager
        self.create_service(self.srv, self.service_name(), self.service_callback)
        self.get_logger().info(f"save_image empty service '{self.service_name()}' started.")        

    def service_callback(self, request, response):
        self.get_logger().info("Empty service called (placeholder).")
        return response
    def image_callback(self, msg):
        if(time.time() - self.time_elapsed > 2):
            self.time_elapsed = time.time()
            self.get_logger().info("Received image for save")
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_time = int(time.time())
            os.system(f"mkdir -p ~/saved_imgs/{int(self.begin_time)}")
            path = os.path.expanduser(f"~/saved_imgs/{int(self.begin_time)}")
            self.get_logger().info(os.path.join(path, f"payload_{image_time}.png"))
            cv2.imwrite(os.path.join(path, f"payload_{image_time}.png"), image)

def main():
    rclpy.init()
    node = SaveImageNode()
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()

    rclpy.shutdown()