from uav.autonomous_modes.Mode import Mode
from rclpy.node import Node
from uav.UAV import UAV
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from uav.vision_nodes import TemuVisionNode
from px4_msgs.msg import TrajectorySetpoint
import numpy as np
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
            Vector3,
            '/hoop_directions',
            self.directions_callback,
            qos_profile
        )
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0 
        self.within_hoop = False
        self.set_once = None
        self.go_fwd = False
    
    def directions_callback(self, msg: Vector3) -> None:
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        # self.log(f"Received directions: x={self.x}, y={self.y}, z={self.z}") 

    def on_update(self, time_delta: float) ->None:
        if self.uav.local_position is None:
            self.log("Waiting for local position data...")
            return
        uav_x_rel = self.z  # forward
        uav_y_rel = self.x  # right
        uav_z_rel = self.y  # down
        uav_x_rel = self.z 

        
        
        # if self.set_once is not None:
        #     self.node.get_logger().info("SET ONCE")
        #     self.uav.publish_position_setpoint(self.set_once)


        #     dist = np.linalg.norm(np.array((self.uav.local_position.x, self.uav.local_position.y, self.uav.local_position.z)) - np.array(self.set_once))
        #     if dist <= 0.1:
        #         self.set_once =  None
        #         self.within_hoop = False 
        #     return

        
        if self.within_hoop:
            scaling_factor_x = -0.5
            scaling_factor_y = -0.5 # [-1, 1] -> [-0.5m, 0.5m]
            scaling_factor_z = 0.5  # forward
        else:
            scaling_factor_x = -1.0
            scaling_factor_y = -1.0 # [-1, 1] -> [-0.5m, 0.5m]
            scaling_factor_z = 0  # forward

        if abs(self.x) <= 0.05 and abs(self.y) <= 0.05:
            self.within_hoop = True
        elif abs(self.x) > 1.5 or abs(self.y) > 1.5:
            self.timer = 1000000
            # self.within_hoop = False

        # if self.z <= 1.0:
        #     x = float(self.uav.local_position.x)
        #     y = float(self.uav.local_position.y + 1.2)
        #     z = float(self.uav.local_position.z)

        #     self.set_once = (x, y, z)
        #     return

        
        

        point = (
            self.x * scaling_factor_x,  # X 
            self.z * scaling_factor_z, # Y 
            self.y * scaling_factor_y  # Z 
        )

        # point = (
        #     0, 0, -2
        # )
        
        self.log(f"Publishing relative setpoint: {point}")
        self.uav.publish_position_setpoint(point, relative=True)
    
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