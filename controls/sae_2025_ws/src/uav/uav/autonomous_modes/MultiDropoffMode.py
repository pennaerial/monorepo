import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from uav.autonomous_modes.payload_pickup_mode import PayloadPickupMode
from rclpy.node import Node
from uav_interfaces.srv import PayloadTracking
from uav.vision_nodes import PayloadTrackingNode
from typing import Optional, List, Dict, Tuple
from px4_msgs.msg import VehicleStatus
import cv2

class MultiPickupMode(PayloadPickupMode):
   """
    A mode for picking up multiple payloads sequentially.
    """

def __init__(self, node: Node, uav: UAV, payloads: List[str] = None):
        """
        Initialize the LowerPayload.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            payloads: list of payload colors['blue', 'red', 'green', ...] 
        """
        super().__init__(node, uav)

        self.payloads = payloads if payloads else []
        self.altitude_constant = 3
        self.current_index = 0
        self.current_pickup: Optional[PayloadPickupMode] = None
        self.done = False

def _start_next_pickup(self):
       """
       Initialize the next PayloadPickupMode for the next payload color.
       """
       if self.current_index >= len(self.payloads):
           self.current_pikcup = None
           self.done = True
           return
       
       color = self.payloads[self.current_index]
       self.current_pickup = PayloadPickupMode (self.node, self.uav, color=color)
       self.log(f"Starting pickup {self.current_index + 1}/{len(self.payloads)}: color={color}")


def on_update(self, time_delta: float) -> None:
        """
        Periodic update the current pickup. Move to the next payload when done.
        """
        if self.done:
           return

        if self.current_pickup is None:
              self._start_next_pickup()
              if self.done:
                  self.log("All payloads picked up.")
                  return
        
        self.current_pickup.on_update(time_delta)

        if self.current_pickup.check_status() == 'complete':
            self.log(f"Pickup {self.current_index + 1} complete.")
            self.current_index += 1
            self.current_pickup = None


def check_status(self) -> str:
        """
        Check if all payloads have been picked up.

        Returns:
            str: The status of the payload pickups.
        """
        if self.done:
            return 'complete'
        return 'continue'