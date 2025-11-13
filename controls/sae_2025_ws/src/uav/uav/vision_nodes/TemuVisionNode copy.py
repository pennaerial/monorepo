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
        self.hoop_waypoints = [
            np.array([1.0, 0.0, -2.0]),  # Hoop 1 (start_x + 0*spacing)
            np.array([3.0, 0.0, -2.0]),  # Hoop 2 (start_x + 1*spacing)
            np.array([5.0, 0.0, -2.0]),  # Hoop 3
            np.array([7.0, 0.0, -2.0]),  # Hoop 4
            np.array([9.0, 0.0, -2.0]),  # Hoop 5
        ]
        self.target_hoop_index = 0
        self.distance_threshold = 0.5  # 飞到 0.5 米内算“到达”
        self.mission_complete = False
        # save uav local position
        self.uav_local_position = None

        self.hoop_directions_publisher = self.create_publisher(msg_type=Vector3, topic='/hoop_directions', qos_profile=10, )

        # add subscriber to uav local position
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position', # (来自 UAV.py)
            self.local_pos_callback,
            qos_profile
        )
        ### END TEST

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
        # x = 10.0
        # y = 10.0
        # z = 1.0

        # TEST
        # 1. check if uav_local_postion is ready
        if self.uav_local_position is None:
            self.get_logger().warn("Fake navigator waiting for UAV local position...")
            return
        # 2. check if mission_complete
        if self.mission_complete:
            self.publish_directions(0.0, 0.0, 0.0) # 发布 (0,0,0) 向量让无人机悬停
            return
        # 3. get curent uav local position
        uav_pos = np.array([
            self.uav_local_position.x,
            self.uav_local_position.y,
            self.uav_local_position.z
        ])

        # 4. get the target hoop position
        target_pos = self.hoop_waypoints[self.target_hoop_index]

        # 5.check if arrived
        distance = np.linalg.norm(target_pos - uav_pos)
        if distance < self.distance_threshold:
            self.get_logger().info(f"REACHED HOOP {self.target_hoop_index} at {target_pos}")
            self.target_hoop_index += 1

            # check if all hoops are reached
            if self.target_hoop_index >= len(self.hoop_waypoints):
                self.get_logger().info("ALL HOOPS COMPLETED! Mission complete.")
                self.mission_complete = True
                return
            
            # update target to next hoop
            self.get_logger().info(f"Proceeding to HOOP {self.target_hoop_index} at {self.hoop_waypoints[self.target_hoop_index]}")
            # update target position
            target_pos = self.hoop_waypoints[self.target_hoop_index]
        # 6. 计算相对向量 (从无人机指向目标, NED坐标系)
        # relative_vector_ned = [rel_N (前), rel_E (右), rel_D (下)]
        relative_vector_ned = target_pos - uav_pos

        # 7. 坐标转换：将 NED 相对向量 转换为 TemuNavMode 期望的 "视觉" 向量
        #
        # TemuNavMode 执行的转换是：
        #   uav_x_rel (前) = self.z (vision_z)
        #   uav_y_rel (右) = self.x (vision_x)
        #   uav_z_rel (下) = self.y (vision_y)
        #
        # 因此，我们必须发布：
        #   vision_x (TemuNavMode 用的 x) = relative_vector_ned[1] (相对 Y / East / 右)
        #   vision_y (TemuNavMode 用的 y) = relative_vector_ned[2] (相对 Z / Down / 下)
        #   vision_z (TemuNavMode 用的 z) = relative_vector_ned[0] (相对 X / North / 前)
        
        # 为了模仿原始 TemuVisionNode 的行为（z=1.0），我们只使用 x 和 y 进行 P 控制
        
        K_gain = 0.5 # 比例增益 (您可以调整这个值)
        
        vision_x_cmd = relative_vector_ned[1] * K_gain # 对应 Y (右)
        vision_y_cmd = relative_vector_ned[2] * K_gain # 对应 Z (下)
        
        # 保持恒定的前飞指令（Z），这是 TemuNavMode 所期望的
        vision_z_cmd = relative_vector_ned[0] * K_gain

        self.get_logger().info(f"Nav to Hoop {self.target_hoop_index}. Dist: {distance:.2f}. NED Rel Vec: {relative_vector_ned}. Vision Cmd: (x={vision_x_cmd:.2f}, y={vision_y_cmd:.2f}, z={vision_z_cmd:.2f})")

        # 8. 发布模拟的视觉指令
        self.publish_directions(vision_x_cmd, vision_y_cmd, vision_z_cmd)
        # END TEST
        # self.publish_directions(x, y, z)
    
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
