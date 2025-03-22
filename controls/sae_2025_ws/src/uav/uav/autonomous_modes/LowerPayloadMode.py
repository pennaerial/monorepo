import numpy as np
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
        self.altitude_constant = 3
        self.done = False

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for lowering payload and handling obstacles.
        """
        if self.uav.roll > 0.1 or self.uav.pitch > 0.1:
            self.node.get_logger().info("Roll or pitch detected. Waiting for stabilization.")
            return
        request = PayloadTracking.Request()
        request.altitude = -self.uav.get_local_position()[2]
        request.yaw = float(self.uav.yaw)
        payload_pose = self.send_request(PayloadTrackingNode, request)
        if payload_pose is None:
            pass
        else:
            if np.abs(payload_pose.direction[0]) < 0.1 and np.abs(payload_pose.direction[1]) < 0.1:
                dir = [0, 0, request.altitude / self.altitude_constant]
                if request.altitude < 0.1:
                    self.done = True
            else:
                dir = [-payload_pose.direction[1], payload_pose.direction[0], 0]
            self.node.get_logger().info(f"Direction: {dir}")

            self.uav.publish_position_setpoint(dir, relative=True)

    def check_status(self) -> str:
        """
        Check the status of the payload lowering.

        Returns:
            str: The status of the payload lowering.
        """
        if self.done:
            return 'complete'
        return 'continue'