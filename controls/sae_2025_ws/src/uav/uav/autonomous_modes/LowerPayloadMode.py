from uav import Mode, UAV
from rclpy.node import Node
from uav.src import PayloadTracking
from rclpy.task import Future
from vision_nodes import PayloadTrackingNode

class LowerPayloadMode(Mode):
    """
    A mode for lowering the payload.
    """

    def __init__(self, node: Node):
        """
        Initialize the LowerPayload.

        Args:
            node (Node): ROS 2 node managing the UAV.
        """
        super().__init__(node, [PayloadTrackingNode()])

        self.payload_pose = None

    def on_update(self):
        """
        Periodic logic for lowering payload and handling obstacles.
        """

        self.payload_pose = self.send_request(self.vision_nodes[0])
        
        if self.payload_pose is None:
            self.log("Current pose not available yet.")
        else:
            self.uav.go_to_target(self.payload_pose)
