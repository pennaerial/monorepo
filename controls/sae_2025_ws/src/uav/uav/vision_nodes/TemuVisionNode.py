# payload_tracking_node.py
import cv2
import numpy as np
from uav.cv.find_hoop import find_hoop
from uav.cv.find_hoop_depth import find_hoop_w_depth
from uav.vision_nodes.VisionNode import VisionNode
import rclpy
from cv_bridge import CvBridge
from uav.utils import pink, green, blue, yellow
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Float64MultiArray

from std_srvs.srv import Empty

from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class TemuVisionNode(VisionNode):
    """
    ROS node for hoop tracking with OpenCV Algorithm.
    """
    srv = Empty
    def __init__(self):
        super().__init__('hoop_tracking', display=False, use_service=False)
        self.bridge = CvBridge()
        self.uav_local_position = None

        self.hoop_directions_publisher = self.create_publisher(msg_type=Vector3, topic='/hoop_directions', qos_profile=10, )

        # add an empty service server for ModeManager
        self.create_service(self.srv, self.service_name(), self.service_callback)
        self.get_logger().info(f"Hoop tracking publisher AND empty service '{self.service_name()}' started.")        

    def local_pos_callback(self, msg: VehicleLocalPosition):
        """save uav local position"""
        self.uav_local_position = msg
    def service_callback(self, request, response):
        self.get_logger().info("Empty service called (placeholder).")
        return response
    def image_callback(self, msg):
        # self.get_logger().info("Received image for hoop tracking.")
        self.sim = True
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        x, y, z, vector_dist, img = find_hoop_w_depth(image)
        self.display_frame(img, "result")

        if x is None or y is None or z is None:
            return

        self.publish_directions(x, y, z, vector_dist)
    
    def publish_directions(self, x, y, z, d):
        '''
        NOTE: 
        x is left right relative to the drone
        y is top down relative to the drone
        z is forward backward relative to the drone
        '''
        msg = Float64MultiArray()
        msg.data = [x, y, z, d]
        self.hoop_directions_publisher.publish(msg)


def main():
    rclpy.init()
    node = TemuVisionNode()
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()

    rclpy.shutdown()
