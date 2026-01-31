from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV, VTOL
from px4_msgs.msg import VehicleStatus


class TakeoffMode(Mode):
    """
    A mode for takeoff (vertical or horizontal).
    Works for both multicopters and VTOLs.
    - Multicopters: always vertical takeoff.
    - VTOLs: vertical or horizontal (fixed-wing style) based on takeoff_type param.
    """

    def __init__(self, node: Node, uav: UAV, takeoff_type: str = 'vertical'):
        """
        Initialize the TakeoffMode.

        Args:
            node (Node): The ROS 2 node.
            uav (UAV): The UAV instance.
            takeoff_type (str): 'vertical' or 'horizontal'. Default 'vertical'.
                Multicopters ignore this (always vertical).
                Horizontal is only valid for VTOLs.
        """
        super().__init__(node, uav)
        self.takeoff_type = takeoff_type.lower()
        self.takeoff_commanded = False  # For vertical: only call takeoff() once

    def on_update(self, time_delta: float) -> None:
        """
        Update the mode
        """
        if self.uav.local_position is None or self.uav.global_position is None:
            self.log("Waiting for position data...")
            return

        if self.takeoff_type == 'horizontal':
            if not self.uav.is_vtol:
                self.log("Horizontal takeoff only valid for VTOL - cannot proceed.")
                return
            if not isinstance(self.uav, VTOL):
                return
            takeoff_done = self.uav.fixed_wing_takeoff()
            if not takeoff_done:
                self.log("Horizontal takeoff in progress...")
                return
        else:
            # Vertical takeoff (multicopter or VTOL)
            if not self.takeoff_commanded:
                self.takeoff_commanded = True
                self.log("Attempting vertical takeoff")
                self.uav.takeoff()

        # Publish heartbeat for offboard handoff
        self.uav.publish_offboard_control_heartbeat_signal()

        # When in AUTO_LOITER, engage offboard mode
        if self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            self.log("Takeoff complete. Engaging offboard mode.")
            self.uav.engage_offboard_mode()

    def check_status(self) -> str:
        """
        Check if takeoff is complete.

        Returns:
            str: "continue" while takeoff in progress, "complete" when takeoff is finished.
        """
        if self.uav.local_position is None or self.uav.global_position is None:
            return "continue"

        # Invalid config: horizontal takeoff on non-VTOL
        if self.takeoff_type == 'horizontal' and not self.uav.is_vtol:
            return "error"

        if self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.log("Takeoff complete, in offboard mode.")
            return "complete"
        return "continue"
