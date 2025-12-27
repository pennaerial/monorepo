from uav.autonomous_modes.Mode import Mode
from rclpy.node import Node
from rclpy.duration import Duration
from uav.UAV import UAV
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from uav.vision_nodes import TemuVisionNode
from px4_msgs.msg import TrajectorySetpoint
import numpy as np
import math
class TemuNavMode(Mode):
    """
    Subsriber of Topic "/hoop_directions"
    """

    def __init__(self, node: Node, uav: UAV):
        """
        Initialize TemuNavMode
        """
        super().__init__(node, uav)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.node.create_subscription(
            Float64MultiArray,
            '/hoop_directions',
            self.directions_callback,
            qos_profile
        )

        self.x = None
        self.y = None
        self.z = None
        self.vector_dist = None

        self.state = 0
        self.state_name = {
            "align": 0,
            "straight": 1,
            "in_buffer": 2,
        }

        self.straight_velocity = 0.5
        self.buffer_ratio = 1/2
        self.buffer_target_pos = None
        self.buffer_start = None
        self.buffer_end = None

    def directions_callback(self, msg: Float64MultiArray) -> None:
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.z = msg.data[2]

        if msg.data[3] is not None:
            self.vector_dist = msg.data[3]
    
    def on_update(self, time_delta: float) ->None:
        if self.uav.local_position is None:
            self.log("Waiting for local position data...")
            return

        self.node.get_logger().info(f"STATE: {self.state}")
        if self.state == self.state_name['align']:
            # scaling_factor_x = -0.5
            # scaling_factor_y = -0.5

            scaling_factor_x = -0.3
            scaling_factor_y = -0.3

            x = self.x * scaling_factor_x
            x = max(x, 0.1) if x >= 0 else max(x, -0.1)
            z = self.y * scaling_factor_y
            z = max(z, 0.1) if z >= 0 else max(z, -0.1)
            #TODO: USE IF STATEMENTS            
            velocity = (
                x,
                0.0, # Y 
                z
            )
            # self.uav.publish_position_setpoint(point, relative=True)
            self.uav.publish_velocity(velocity, yaw=math.pi/2)
        
            # check for switching conditions
            # thresh = np.clip(self.z / 100, 0.01, 0.05)
            vector_thresh = 10
            # self.node.get_logger().info(f'ALIGN THRESHOLD: {float(thresh):.4f}')
            self.node.get_logger().info(f'VECTOR_DIST: {float(self.vector_dist):.4f}')
            # if abs(self.x) <= thresh and abs(self.y) <= thresh:
            #     self.state = self.state_name['straight']
            if self.vector_dist <= vector_thresh:
                self.state = self.state_name['straight']

        elif self.state == self.state_name['straight']:
            scaling_factor_x = -1.0
            scaling_factor_y = -1.0

            velocity = (
                self.x * scaling_factor_x,
                self.straight_velocity,
                self.y * scaling_factor_y
            )

            if self.x is not None and self.y is not None:

                #if previous pos is within hoop, but current is tracking next hoop, change state
                x_dist = abs(self.x)
                y_dist = abs(self.y)
                z_dist = abs(self.z)
                self.node.get_logger().info(f'\nX_DIST: {x_dist:.4f} \n Y_DIST:{y_dist:.4f}')
                
                if x_dist >= 0.5 or y_dist >= 0.5:
                    self.state = self.state_name['in_buffer']
                    buffer_distance = np.clip(z_dist * self.buffer_ratio, 0.9, 2.0)
                    self.node.get_logger().info(f'\nBUF_DIST: {z_dist * self.buffer_ratio:.4f} \nCLIPPED_BUF_DIST:{buffer_distance:.4f}')
                    time_elapsed = buffer_distance / self.straight_velocity
                    self.buffer_end = self.node.get_clock().now() + Duration(seconds=time_elapsed)
                    return 
            self.uav.publish_velocity(velocity, yaw=math.pi/2)
        
        elif self.state == self.state_name['in_buffer']:
            velocity = [0.0, self.straight_velocity, 0.0]
            self.uav.publish_velocity(velocity, yaw=math.pi/2)
            now = self.node.get_clock().now()
            self.node.get_logger().info(f"Current time (s): {now.nanoseconds / 1e9:.6f}")
            if now >= self.buffer_end:
                self.state = self.state_name['align']

    
    
    def check_status(self):
        # return super().check_status()
        return "continue"
    def on_exit(self) -> None:
        """
        Exit mode
        """
        self.log("TemuNavMode exiting.")
        #
        pass