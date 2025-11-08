import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import HoopTracking
from uav.vision_nodes import HoopTrackingNode
from typing import Optional, Tuple
from px4_msgs.msg import VehicleStatus
import cv2

class HoopMode(Mode):
    """
    A mode for flying through a hoop.
    """

    def __init__(self, node: Node, uav: UAV, offsets: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)):
        """
        Initialize the HoopMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            offsets (Optional[Tuple[float, float, float]]):
                Should denote the position of flight path relative to the center of hoop, in meters
                In NED frame: x is forward, y is right, and z is down.
        """
        super().__init__(node, uav)

        self.response = None
        self.done = False
        self.offsets = offsets
        self.camera_offsets = self.uav.camera_offsets
        self.mode = 0 # 0 for uav centering, 1 for landing, 2 for retracting, 3 for taking off
        # New: 0 for uav centering, 1 for flying to hoop center, 2 for through hoop, 3 for done

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for finding hoop and flying through it.
        """
        self.log(f"=== MODE {self.mode} ===")

        # If UAV is unstable, skip the update
        if self.uav.roll > 0.1 or self.uav.pitch > 0.1:
            self.log("Roll or pitch detected. Waiting for stabilization.")
            return

        # Mode transitions for post-hoop states
        if self.mode == 1:
            # After starting forward motion, advance to clearing phase
            self.mode = 2
            self.forward_start_pos = self.uav.get_local_position()
            return

        if self.mode == 2:
            # Check if we've traveled enough distance forward (e.g., 2 meters)
            current_pos = self.uav.get_local_position()
            distance_traveled = np.linalg.norm(
                np.array(current_pos[:2]) - np.array(self.forward_start_pos[:2])
            )
            self.log(f"Distance traveled: {distance_traveled:.2f}m")
            if distance_traveled > 2.0:  # 2 meters clearance
                self.mode = 3
                self.done = True
                self.log("DONE! Clearing complete")
            else:
                # Keep moving forward
                self.log("Mode 2: Publishing (1, 0, 0)")
                self.uav.publish_position_setpoint((1, 0, 0), relative=True)
            return

        if self.mode == 3:
            return  # Done, waiting for transition
    
        request = HoopTracking.Request()
        request.altitude = -self.uav.get_local_position()[2]
        request.yaw = float(self.uav.yaw)
        request.payload_color = 'red'
        response = self.send_request(HoopTrackingNode, request)
        
        # If no hoop pose is received, exit early
        if response is None:
            return

        # Transform from camera frame to UAV NED frame (forward-facing camera)
        # Camera: X=right, Y=down, Z=forward -> UAV NED: X=forward, Y=right, Z=down
        # Note: Camera Y down = positive, UAV Z down = positive, so direct mapping
        self.log(f"Raw response.direction: {response.direction}")
        direction = [response.direction[2], response.direction[0], response.direction[1]]
        self.log(f"After frame transform: {direction}")

        offsets = tuple(x / request.altitude for x in self.offsets) if request.altitude > 1 else self.offsets
        camera_offsets = tuple(x / request.altitude for x in self.camera_offsets) if request.altitude > 1 else self.camera_offsets
        direction = [x + y + z for x, y, z in zip(direction, offsets, self.uav.uav_to_local(camera_offsets))]

        # Check if centered on hoop (left/right and up/down)
        threshold = 0.2
        if (np.abs(direction[1]) < threshold and
            np.abs(direction[2]) < threshold):
            # Centered! Fly forward through the hoop
            self.log(f"CENTERED! Publishing: (1, 0, 0)")
            self.uav.publish_position_setpoint((1, 0, 0), relative=True)
            self.mode = 1
            return

        # Not centered yet - adjust position without moving forward
        # Negate the direction because we want to move TOWARD the target, not in the direction OF the target
        direction[0] = 0  # Zero out forward movement until centered
        direction[1] = -direction[1]  # Negate Y
        direction[2] = -direction[2]  # Negate Z
        
        self.log(f"Centering - Publishing direction: {direction}")
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