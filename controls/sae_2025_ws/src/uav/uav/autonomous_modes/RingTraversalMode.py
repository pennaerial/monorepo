import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
# from uav_interfaces.srv import PayloadTracking
from uav.vision_nodes import RingTrackingNode
from typing import Optional, Tuple
import cv2

class RingTraversalMode(Mode):
    """Simple mode that listens to /ring_tracking and moves the UAV along the published direction vector until it is nearly centred, then reports completion."""

    def __init__(self, node: Node, uav: UAV, threshold: float = 0.1):
        super().__init__(node, uav)
        from std_msgs.msg import Float64MultiArray
        self.threshold = threshold  # magnitude when considered centred
        self.latest_vec = None  # store last Float64MultiArray

        self.sub = node.create_subscription(
            Float64MultiArray,
            '/ring_tracking',
            self._ring_cb,
            10
        )

    def _ring_cb(self, msg):
        #msg.data = [x, y, dir_x, dir_y, dir_z, flag]
        if len(msg.data) >= 6:
            self.latest_vec = msg.data

    def on_update(self, dt: float):
        if self.latest_vec is None:
            self.log("Waiting for ring data ...")
            return
        self.log("Shit is working")
        # Use dir vector for guidance
        dir_x, dir_y, dir_z = self.latest_vec[2:5]
        vec = np.array([dir_x, dir_y, dir_z])
        mag = np.linalg.norm(vec)
        if mag < self.threshold:
            self.done = True
            return
        # Scale to small step proportional to magnitude
        step = vec  # already unit-ish; UAV class caps velocity
        self.uav.publish_position_setpoint(step, relative=True)

    def check_status(self):
        return 'complete' if getattr(self, 'done', False) else 'continue'
