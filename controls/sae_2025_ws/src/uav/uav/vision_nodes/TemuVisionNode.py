# payload_tracking_node.py
import cv2
import numpy as np
from uav.cv.find_hoop import find_hoop
from uav.vision_nodes.VisionNode import VisionNode
import rclpy
from cv_bridge import CvBridge
from uav.utils import pink, green, blue, yellow
from geometry_msgs.msg import Vector3

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

        # TEST
        # self.hoop_waypoints = [
        #     np.array([1.0, 0.0, -2.0]),  # Hoop 1 (start_x + 0*spacing)
        #     np.array([3.0, 0.0, -2.0]),  # Hoop 2 (start_x + 1*spacing)
        #     np.array([5.0, 0.0, -2.0]),  # Hoop 3
        #     np.array([7.0, 0.0, -2.0]),  # Hoop 4
        #     np.array([9.0, 0.0, -2.0]),  # Hoop 5
        # ]
        # self.target_hoop_index = 0
        # self.distance_threshold = 0.5  
        # self.mission_complete = False
        
        # save uav local position
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
        print("image callback but currently does nothing")
        # self.sim = True
        # image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # x, y, img = find_hoop(image)
        # self.display_frame(img, "result")
        x = 10.0
        y = 10.0
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
