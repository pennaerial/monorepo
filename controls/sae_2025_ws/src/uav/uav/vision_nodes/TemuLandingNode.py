# payload_tracking_node.py
import cv2
import numpy as np
from uav.vision_nodes.VisionNode import VisionNode
import rclpy
from cv_bridge import CvBridge
from uav.utils import pink, green, blue, yellow
from geometry_msgs.msg import Vector3

from std_srvs.srv import Empty

from px4_msgs.msg import VehicleLocalPosition

class TemuLandingNode(VisionNode):
    """
    ROS node for hoop tracking with OpenCV Algorithm.
    """
    srv = Empty
    def __init__(self):
        super().__init__('landing_zone_detection', display=False, use_service=False)
        self.bridge = CvBridge()
        self.uav_local_position = None

        targetR = 255
        targetG = 0
        targetB = 0

        bgrPixel = np.uint8([[[targetB, targetG, targetR]]])
        hsvPixel = cv2.cvtColor(bgrPixel, cv2.COLOR_BGR2HSV)
        H, S, V = hsvPixel[0][0]

        dH, dS, dV = 10, 20, 20

        # threshold padding
        dH, dS, dV = 10, 80, 80
        self.h_low = max(H - dH, 0)
        self.h_high = min(H + dH, 179)
        self.s_low = max(S - dS, 0)
        self.s_high = min(S + dS, 255)
        self.v_low = max(V - dV, 0)
        self.v_high = min(V + dV, 255)

        self.arrival_tol = 0.05  # 5% of image dimension

        # publisher 
        self.landing_direcion_publisher = self.create_publisher(msg_type=Vector3, topic='/landing_direction', qos_profile=10, )

        # add an empty service server for ModeManager
        self.create_service(self.srv, self.service_name(), self.service_callback)
        self.get_logger().info(f"landing_zone tracking publisher AND empty service '{self.service_name()}' started.")   


    def local_pos_callback(self, msg: VehicleLocalPosition):
        """save uav local position"""
        self.uav_local_position = msg

    def service_callback(self, request, response):
        self.get_logger().info("Empty service called (placeholder).")
        return response
    
    def image_callback(self, msg):
        self.get_logger().info("Received image for hoop tracking.")
        self.sim = True
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        x, y, z, img = find_landing_zone(self, image)
        self.display_frame(img, "result")
        self.publish_directions(x, y, z)
    
    def publish_directions(self, x, y, z):
        '''
        NOTE: 
        x is left right relative to the drone
        y is top down relative to the drone
        z is forward backward relative to the drone
        '''
        msg = Vector3() 
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.hoop_directions_publisher.publish(msg)


    def find_landing_zone(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


        mask = cv2.inRange(
            hsv, 
            np.array([self.h_low, self.s_low, self.v_low]),
            np.array([self.h_high, self.s_high, self.v_high])
        )

        # clean mask by filling 
        mask = cv2.erode(mask, None, 2)
        mask = cv2.dilate(mask, None, 2)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest = max(contours, key=cv2.contourArea)
        
        x, y, w, h = cv2.boundingRect(largest)
        cx = x + w/2
        y = y + h/2

        





        

        

def main():
    rclpy.init()
    node = TemuLandingNode()
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()

    rclpy.shutdown()
