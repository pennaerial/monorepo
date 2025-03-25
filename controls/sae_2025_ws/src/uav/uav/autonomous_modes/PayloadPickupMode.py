import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import PayloadTracking
from uav.vision_nodes import PayloadTrackingNode
from typing import Optional, Tuple
import cv2

class PayloadPickupMode(Mode):
    """
    A mode for picking up a payload.
    """

    def __init__(self, node: Node, uav: UAV, color: str = 'green', camera_offsets: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)):
        """
        Initialize the LowerPayload.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            color (str): The color of the payload to track.
            camera_offsets (Optional[Tuple[float, float, float]]): The camera_offsets for the payload lowering. 
                Should denote the position of the camera relative to the payload mechanism, in meters
                In NED frame: x is forward, y is right, and z is down.
        """
        super().__init__(node, uav)

        self.response = None
        self.altitude_constant = 3
        self.done = False
        self.camera_offsets = camera_offsets
        self.color = color
        self.goal_pos = None

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for lowering payload and handling obstacles.
        """
        # If UAV is unstable, skip the update
        if self.uav.roll > 0.1 or self.uav.pitch > 0.1:
            self.node.get_logger().info("Roll or pitch detected. Waiting for stabilization.")
            return
          
        request = PayloadTracking.Request()
        request.altitude = -self.uav.get_local_position()[2]
        request.yaw = float(self.uav.yaw)
        request.payload_color = self.color
        response = self.send_request(PayloadTrackingNode, request)
        
        # If no payload pose is received, exit early
        if response is None:
            return

        if self.goal_pos:
            self.uav.publish_position_setpoint(self.goal_pos)
            if self.uav.distance_to_waypoint('LOCAL', self.goal_pos) <= 0.05:
                if response.dlz_empty or True:
                    self.done = True
                else:
                    pass # TODO: Extend servo
            return
        
        direction = [-response.direction[1], response.direction[0],
                        response.direction[2] / self.altitude_constant]
        
        camera_offsets = tuple(x / request.altitude for x in self.camera_offsets) if request.altitude > 1 else self.camera_offsets
        direction = [x + y for x, y in zip(direction, self.uav.uav_to_local(camera_offsets))]

        # Determine the direction vector based on altitude and payload pose
        if request.altitude < 1:
            # If payload pose direction is within a small threshold
            if (np.abs(direction[0]) < request.altitude / 50 and
                np.abs(direction[1]) < request.altitude / 50):
                if request.altitude < 0.5:
                    self.goal_pos = self.uav.get_local_position()
                    return
                else:
                    direction = [0, 0, request.altitude / self.altitude_constant]
            else:
                direction[2] = 0

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