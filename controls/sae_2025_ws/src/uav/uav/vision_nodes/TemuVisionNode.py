# payload_tracking_node.py
import cv2
import numpy as np
from uav.cv.find_hoop import find_hoop
from uav.vision_nodes.VisionNode import VisionNode
import rclpy
from cv_bridge import CvBridge
from uav.utils import pink, green, blue, yellow
from geometry_msgs.msg import Vector3

class TemuVisionNode(VisionNode):
    """
    ROS node for hoop tracking with OpenCV Algorithm.
    """
    def __init__(self):
        super().__init__('hoop_tracking', display=False, use_service=False)
        self.bridge = CvBridge()

        self.hoop_directions_publisher = self.create_publisher(msg_type=Vector3, topic='/hoop_directions', qos_profile=10, )


    def image_callback(self, msg):
        self.sim = True
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        x, y, img = find_hoop(image)
        self.display_frame(img, "result")
        z = 1.0
        self.publish_directions(x, y, z)
    
    def publish_directions(self, x, y, z):
        msg = Vector3()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
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
