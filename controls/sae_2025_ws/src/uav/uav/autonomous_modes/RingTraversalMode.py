import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
import math
# from uav_interfaces.srv import PayloadTracking
from uav.vision_nodes import RingTrackingNode
from typing import Optional, Tuple
import cv2

class RingTraversalMode(Mode):
    """Simple mode that listens to /ring_tracking and moves the UAV along the published direction vector until it is nearly centred, then reports completion."""

    def __init__(self, node: Node, uav: UAV, threshold: float = 0.1):
        super().__init__(node, uav)
        from std_msgs.msg import Float64MultiArray
        self.threshold = threshold  # magnitude when close enough
        self.latest_vec = None  # store last Float64MultiArray
        self.STATE = 'lateral'
        self.bullrush_start_ns = None  # timer anchor for bullrush duration

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

        #STATE MACHINE LOGIC
        self.log(f"State: {self.STATE}")

        
        if self.STATE == 'lateral':
            #when aligned, move to forward state
            if math.fabs(dir_x) < 0.04 and math.fabs(dir_z) < 0.04: #working with 0.02
                self.STATE = 'forward'
            else:
                vec = np.array([dir_x / 1.0, 0.0, dir_z / 1.0]).astype('float32')
                self.uav.publish_velocity(vec)
                return
            
        if self.STATE == 'forward': 
            if math.fabs(dir_y) < 0.4:
                self.STATE = 'bullrush'
            else:
                vec = np.array([dir_x / 10.0, dir_y / 1.5, dir_z / 10.0]).astype('float32')
                self.uav.publish_velocity(vec)
                return

        if self.STATE == 'bullrush':
            # Initialize timer on first entry
            if self.bullrush_start_ns is None:
                self.bullrush_start_ns = self.node.get_clock().now().nanoseconds
                self.log("Entering BULLRUSH: starting 2-second forward drive")
            # Command constant forward velocity (Y axis 0.5 m/s per publish_velocity default)
            self.uav.publish_velocity([0.0, 0.5, 0.0])
            elapsed = (self.node.get_clock().now().nanoseconds - self.bullrush_start_ns) / 1e9
            if elapsed >= 3.7:
                self.STATE = 'lateral'
                self.bullrush_start_ns = None
                self.log("BULLRUSH finished – switching back to LATERAL")
            # During bullrush we already commanded velocity; skip rest of on_update
        
        
        # vec = np.array([dir_x / 2.0 , dir_y / 5.0, dir_z / 2.0]).astype('float32')
        # mag = np.linalg.norm(vec)
        # if mag < self.threshold:
        #     self.done = True
        #     return
        # Scale to small step proportional to magnitude
        # step = vec # already unit-ish; UAV class caps velocity
        # Use time-based movement helper to apply this velocity for a short time
        # Start the timed movement only if one is not already active.
        # if getattr(self, '_move_velocity', None) is None:
        #     # adjust duration as needed (seconds)
        #     self.move_for_duration(step, duration_s=0.2, rate_hz=10)

    def check_status(self):
        return 'complete' if getattr(self, 'done', False) else 'continue'
