import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import PayloadTracking
from uav.vision_nodes import PayloadTrackingNode
from typing import Optional, Tuple
from px4_msgs.msg import VehicleStatus
import cv2

class DropoffTestMode(Mode):
    """
    A mode for dropping off the payload.
    """

    def __init__(self, node: Node, uav: UAV, offsets: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0), camera_offsets: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)):
        """
        Initialize the LowerPayload.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            offsets (Optional[Tuple[float, float, float]]):
                Should denote the position of dropoff relative to the center of zone, in meters
                In NED frame: x is forward, y is right, and z is down.
            camera_offsets (Optional[Tuple[float, float, float]]):
                Should denote the position of the camera relative to the payload mechanism, in meters
                In NED frame: x is forward, y is right, and z is down.
        """
        super().__init__(node, uav)

        self.timer = 0
        self.lower_time = 1.87
        self.done = False
        # 0 for uav centering, 1 for landing, 2 for retracting, 3 for taking off

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for lowering payload and handling obstacles.
        """
        # If UAV is unstable, skip the update
        if self.timer < self.lower_time and self.timer >= 0:
            self.uav.drop_payload()
            self.timer += time_delta
        if self.timer >= self.lower_time:
            self.lowering = False
            self.uav.pickup_payload()
            self.timer -= time_delta
        if self.timer < 0:
            self.uav.disable_servo()
            self.done = True

    def check_status(self) -> str:
        """
        Check the status of the payload lowering.

        Returns:
            str: The status of the payload lowering.
        """
        if self.done:
            return 'complete'
        return 'continue'