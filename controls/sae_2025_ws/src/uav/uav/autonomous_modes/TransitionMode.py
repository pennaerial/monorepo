from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV
import numpy as np

class TransitionMode(Mode):
    """
    A mode for transitioning between VTOL types
    """
    def __init__(self, node: Node, uav: UAV, to_mode: str):
        """
        Initialize the Transition Mode

        Args:
            node (Node): The ROS2 node
            uav (UAV): The UAV object
            to_mode (str): The mode to transition to (MC or FW)
        """
        super().__init__(node, uav)
        self.to_mode = to_mode
        self.transition_commanded = False  # Track if transition command has been sent

    def on_update(self, time_delta: float):
        """
        Update the mode
        """
        assert self.uav.is_vtol, "UAV is not a VTOL"

        # If vehicle_type is None, we haven't received VTOL status yet, so wait
        if self.uav.vehicle_type is None:
            self.log("Waiting for VTOL status...")
            # Still need to publish setpoints to maintain offboard connection
            if self.uav.local_position:
                self.uav.publish_position_setpoint(
                    (self.uav.local_position.x, self.uav.local_position.y, self.uav.local_position.z),
                    lock_yaw=True
                )
            return
            
        if self.uav.vehicle_type != self.to_mode:
            # Send transition command only once
            if not self.transition_commanded:
                self.log(f"Transitioning from {self.uav.vehicle_type} to {self.to_mode}")
                self.uav.vtol_transition_to(self.to_mode)
                self.transition_commanded = True
            
            # During transition, publish appropriate setpoints
            if self.uav.local_position:
                if self.to_mode == 'FW':
                    # For MC->FW transition: set position ahead to ensure forward velocity
                    # publish_position_setpoint will handle FW mode velocity automatically
                    if self.uav.yaw is not None:
                        # Set position ahead in current yaw direction
                        ahead_dist = 10.0  # meters ahead
                        target_pos = (
                            self.uav.local_position.x + np.cos(self.uav.yaw) * ahead_dist,
                            self.uav.local_position.y + np.sin(self.uav.yaw) * ahead_dist,
                            self.uav.local_position.z
                        )
                        self.uav.publish_position_setpoint(target_pos)
                    else:
                        # Fallback: use current position (publish_position_setpoint will handle FW velocity)
                        # Lock yaw to prevent spinning during transition
                        self.uav.publish_position_setpoint(
                            (self.uav.local_position.x, self.uav.local_position.y, self.uav.local_position.z),
                            lock_yaw=True
                        )
                else:
                    # For FW->MC transition: hover at current position (MC can hover)
                    # Lock yaw to prevent spinning during transition
                    self.uav.publish_position_setpoint(
                        (self.uav.local_position.x, self.uav.local_position.y, self.uav.local_position.z),
                        lock_yaw=True
                    )
        else:
            self.log(f"Already in {self.to_mode} mode")
            # Maintain setpoints even after transition completes
            if self.uav.local_position:
                if self.to_mode == 'FW' and self.uav.yaw is not None:
                    # For FW mode: maintain forward velocity by setting position ahead
                    ahead_dist = 10.0
                    target_pos = (
                        self.uav.local_position.x + np.cos(self.uav.yaw) * ahead_dist,
                        self.uav.local_position.y + np.sin(self.uav.yaw) * ahead_dist,
                        self.uav.local_position.z
                    )
                    self.uav.publish_position_setpoint(target_pos)
                else:
                    # MC mode or no yaw: maintain current position
                    # Lock yaw to prevent spinning
                    self.uav.publish_position_setpoint(
                        (self.uav.local_position.x, self.uav.local_position.y, self.uav.local_position.z),
                        lock_yaw=True
                    )
    
    def check_status(self):
        """
        Check the status of the mode
        """
        # If vehicle_type is None, we haven't received VTOL status yet, so continue waiting
        if self.uav.vehicle_type is None:
            return "continue"
        if self.uav.vehicle_type != self.to_mode:
            return "continue"
        else:
            self.log(f"Transition to {self.to_mode} complete")
            return "complete"