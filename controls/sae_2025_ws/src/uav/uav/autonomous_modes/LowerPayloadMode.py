import random
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import PayloadTracking
from uav.vision_nodes import PayloadTrackingNode

class LowerPayloadMode(Mode):
    """
    A mode for lowering the payload.
    """

    def __init__(self, node: Node, uav: UAV):
        """
        Initialize the LowerPayload.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
        """
        super().__init__(node, uav)

        self.payload_pose = None

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for lowering payload and handling obstacles.
        """
        request = PayloadTracking.Request()
        request.altitude = self.uav.get_gps()[2]
        self.payload_pose = self.send_request(PayloadTrackingNode, request)
        
        if self.payload_pose is None:
            self.log("Current pose not available yet.")
        else:
            self.uav.set_target_position(self.payload_pose)

    def check_status(self) -> str:
        """
        Check the status of the payload lowering.

        Returns:
            str: The status of the payload lowering.
        """
        if random.random() < 0.1:
            print('lower payload continue')
            return 'continue'