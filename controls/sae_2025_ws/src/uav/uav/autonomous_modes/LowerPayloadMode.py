import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import PayloadTracking
from uav.vision_nodes import PayloadTrackingNode
from typing import Optional, Tuple

class LowerPayloadMode(Mode):
    """
    A mode for lowering the payload.
    """

    def __init__(self, node: Node, uav: UAV, offsets: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)):
        """
        Initialize the LowerPayload.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            offsets (Optional[Tuple[float, float, float]]): The offsets for the payload lowering. 
                Should denote the position of the camera relative to the payload mechanism, in meters
                In NED frame: x is forward, y is right, and z is down.
        """
        super().__init__(node, uav)

        self.payload_pose = None
        self.altitude_constant = 2
        self.done = False
        self.offsets = offsets
        self.goal_pos = None

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for lowering payload and handling obstacles.
        """
        if self.goal_pos:
            if self.uav.distance_to_waypoint('LOCAL', self.goal_pos) > 0.05:
                self.uav.publish_position_setpoint(self.goal_pos)
            else:
                if True: # TODO: Add logic for successful payload capture
                    self.done = True
                else:
                    pass # TODO: Add logic for payload mechanism lowering
            return
        # If UAV is unstable, skip the update
        if self.uav.roll > 0.1 or self.uav.pitch > 0.1:
            self.node.get_logger().info("Roll or pitch detected. Waiting for stabilization.")
            return
          
        request = PayloadTracking.Request()
        request.altitude = -self.uav.get_local_position()[2]
        request.yaw = float(self.uav.yaw)
        payload_pose = self.send_request(PayloadTrackingNode, request)
        
        # If no payload pose is received, exit early
        if payload_pose is None:
            return

        # Determine the direction vector based on altitude and payload pose
        if request.altitude < 1:
            # If payload pose direction is within a small threshold
            if (np.abs(payload_pose.direction[0]) < request.altitude / 10 and
                np.abs(payload_pose.direction[1]) < request.altitude / 10):
                direction = [0, 0, request.altitude / self.altitude_constant]
                if request.altitude < 0.5:
                    self.goal_pos = self.uav.uav_to_local(self.offsets)
                    self.uav.publish_position_setpoint(self.goal_pos)
            else:
                direction = [-payload_pose.direction[1], payload_pose.direction[0], 0]
        else:
            direction = [-payload_pose.direction[1], payload_pose.direction[0],
                        payload_pose.direction[2] / self.altitude_constant]

        self.node.get_logger().info(f"Direction: {direction}")
        self.uav.publish_position_setpoint(direction, relative=True)

    def check_status(self) -> str:
        """
        Check the status of the payload lowering.

        Returns:
            str: The status of the payload lowering.
        """
        if self.done:
            return 'complete'
        return 'continue'