import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import PayloadTracking
from uav.vision_nodes import PayloadTrackingNode
from typing import Optional, Tuple
from px4_msgs.msg import VehicleStatus
import cv2

class PayloadDropoffMode(Mode):
    """
    A mode for dropping off the payload.
    """

    def __init__(self, node: Node, uav: UAV, offsets: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)):
        """
        Initialize the LowerPayload.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            offsets (Optional[Tuple[float, float, float]]):
                Should denote the position of dropoff relative to the center of zone, in meters
                In NED frame: x is forward, y is right, and z is down.
        """
        super().__init__(node, uav)

        self.response = None
        self.done = False
        self.offsets = offsets
        self.mode = 0 # 0 for uav centering, 1 for landing, 2 for retracting, 3 for taking off

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for lowering payload and handling obstacles.
        """
        # If UAV is unstable, skip the update
        if self.uav.roll > 0.1 or self.uav.pitch > 0.1:
            self.log("Roll or pitch detected. Waiting for stabilization.")
            return
        
        if self.mode == 1:
            if self.uav.vehicle_status != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self.mode = 2
            else:
                return
        if self.mode == 2:
            if True: # TODO: Check if payload is dropped off (servo is fully retracted)
                self.mode = 3
                self.uav.attempted_takeoff = False
            else:
                return
        if self.mode == 3:
            if self.uav.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self.done = True
            return
          
        request = PayloadTracking.Request()
        request.altitude = -self.uav.get_local_position()[2]
        request.yaw = float(self.uav.yaw)
        request.payload_color = 'pink'
        response = self.send_request(PayloadTrackingNode, request)
        
        # If no payload pose is received, exit early
        if response is None:
            return

        direction = [-response.direction[1], response.direction[0], response.direction[2]]
        
        offsets = tuple(x / request.altitude for x in self.offsets) if request.altitude > 1 else self.offsets
        direction = [x + y + z for x, y, z in zip(direction, offsets, self.uav.uav_to_local(self.uav.camera_offsets))]

        if request.altitude < 1:
            # If payload pose direction is within a small threshold
            if (np.abs(direction[0]) < request.altitude / 25 and
                np.abs(direction[1]) < request.altitude / 25):
                self.uav.land()
                self.mode = 1
                return
            else:
                direction[2] = 0

        self.log(f"Direction: {direction}")
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