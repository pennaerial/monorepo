import numpy as np
from rclpy.node import Node

from uav.UAV import UAV

from .Mode import Mode


class TransitionMode(Mode):
    """
    A mode for transitioning between VTOL types
    """

    mission_target = "uav"

    def __init__(self, node: Node, vehicle: UAV, to_mode: str):
        """
        Initialize the Transition Mode

        Args:
            node (Node): The ROS2 node
            vehicle (UAV): The UAV object
            to_mode (str): The mode to transition to (MC or FW)
        """
        super().__init__(node, vehicle)
        self.to_mode = to_mode
        self.transition_commanded = False  # Track if transition command has been sent

    def on_update(self, time_delta: float):
        """
        Update the mode
        """
        assert self.vehicle.is_vtol, "UAV is not a VTOL"

        # If vehicle_type is None, we haven't received VTOL status yet, so wait
        if self.vehicle.vehicle_type is None:
            self.log("Waiting for VTOL status...")
            # Still need to publish setpoints to maintain offboard connection
            if self.vehicle.local_position:
                self.vehicle.publish_position_setpoint(
                    (
                        self.vehicle.local_position.x,
                        self.vehicle.local_position.y,
                        self.vehicle.local_position.z,
                    ),
                    lock_yaw=True,
                )
            return

        if self.vehicle.vehicle_type != self.to_mode:
            # Send transition command only once
            if not self.transition_commanded:
                self.log(
                    f"Transitioning from {self.vehicle.vehicle_type} to {self.to_mode}"
                )
                self.vehicle.vtol_transition_to(self.to_mode)
                self.transition_commanded = True

            # During transition, publish appropriate setpoints
            if self.vehicle.local_position:
                if self.to_mode == "FW":
                    # For MC->FW transition: set position ahead to ensure forward velocity
                    # publish_position_setpoint will handle FW mode velocity automatically
                    if self.vehicle.yaw is not None:
                        # Set position ahead in current yaw direction
                        ahead_dist = 10.0  # meters ahead
                        target_pos = (
                            self.vehicle.local_position.x
                            + np.cos(self.vehicle.yaw) * ahead_dist,
                            self.vehicle.local_position.y
                            + np.sin(self.vehicle.yaw) * ahead_dist,
                            self.vehicle.local_position.z,
                        )
                        self.vehicle.publish_position_setpoint(target_pos)
                    else:
                        # Fallback: use current position (publish_position_setpoint will handle FW velocity)
                        # Lock yaw to prevent spinning during transition
                        self.vehicle.publish_position_setpoint(
                            (
                                self.vehicle.local_position.x,
                                self.vehicle.local_position.y,
                                self.vehicle.local_position.z,
                            ),
                            lock_yaw=True,
                        )
                else:
                    # For FW->MC transition: hover at current position (MC can hover)
                    # Lock yaw to prevent spinning during transition
                    self.vehicle.publish_position_setpoint(
                        (
                            self.vehicle.local_position.x,
                            self.vehicle.local_position.y,
                            self.vehicle.local_position.z,
                        ),
                        lock_yaw=True,
                    )
        else:
            self.log(f"Already in {self.to_mode} mode")
            # Maintain setpoints even after transition completes
            if self.vehicle.local_position:
                if self.to_mode == "FW" and self.vehicle.yaw is not None:
                    # For FW mode: maintain forward velocity by setting position ahead
                    ahead_dist = 10.0
                    target_pos = (
                        self.vehicle.local_position.x + np.cos(self.vehicle.yaw) * ahead_dist,
                        self.vehicle.local_position.y + np.sin(self.vehicle.yaw) * ahead_dist,
                        self.vehicle.local_position.z,
                    )
                    self.vehicle.publish_position_setpoint(target_pos)
                else:
                    # MC mode or no yaw: maintain current position
                    # Lock yaw to prevent spinning
                    self.vehicle.publish_position_setpoint(
                        (
                            self.vehicle.local_position.x,
                            self.vehicle.local_position.y,
                            self.vehicle.local_position.z,
                        ),
                        lock_yaw=True,
                    )

    def check_status(self):
        """
        Check the status of the mode
        """
        # If vehicle_type is None, we haven't received VTOL status yet, so continue waiting
        if self.vehicle.vehicle_type is None:
            return "continue"
        if self.vehicle.vehicle_type != self.to_mode:
            return "continue"
        else:
            self.log(f"Transition to {self.to_mode} complete")
            return "complete"
