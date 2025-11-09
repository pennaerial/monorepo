from uav.autonomous_modes.Mode import Mode
from rclpy.node import Node
from uav.UAV import UAV
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from uav.vision_nodes import TemuVisionNode
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
    
    def directions_callback(self, msg: Vector3) -> None:
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.log(f"Received directions: x={self.x}, y={self.y}, z={self.z}") 

    def on_update(self, time_delta: float) ->None:
        if self.uav.local_position is None:
            self.log("Waiting for local position data...")
            return
        uav_x_rel = self.z  # forward
        uav_y_rel = self.x  # right
        uav_z_rel = self.y  # down
        uav_x_rel = self.z 
        
        
        scaling_factor_xy = 1.0 # [-1, 1] -> [-0.5m, 0.5m]
        scaling_factor_z = 1.0  # forward
        
        point = (
            self.z * scaling_factor_z,  # X 
            self.x * scaling_factor_xy, # Y 
            self.y * scaling_factor_xy  # Z 
        )
        
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