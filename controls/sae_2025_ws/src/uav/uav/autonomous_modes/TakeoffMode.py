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
                Horizontal is only valid for VTOLs.
        """
        super().__init__(node, uav)
        self.takeoff_type = takeoff_type.lower()
        self.takeoff_commanded = False  # For vertical: only call takeoff() once
        self.takeoff_elapsed_time = 0.0 # PX4-Autopilot on ARM has a race condition when changing vehicle state

    def on_update(self, time_delta: float) -> None:
        """
        Update the mode
        """

        # Increment elapsed time by the time delta
        self.takeoff_elapsed_time += time_delta

        if self.uav.local_position is None or self.uav.global_position is None:
            self.log("Waiting for position data...")
            return
        
        # Takeoff command already sent
        if self.takeoff_commanded:
            self.log(f"{self.takeoff_type.capitalize()} takeoff in progress.")
        
        # Need to send takeoff command
        else:
            self.takeoff_elapsed_time = 0.0 # time reset as a result of time_delta counting from beginning of launch
            if self.takeoff_type == 'horizontal':
                if not self.uav.is_vtol or not isinstance(self.uav, VTOL):
                    self.node.get_logger().error("Horizontal takeoff only valid for VTOL - cannot proceed.")
                    return
                self.takeoff_commanded = self.uav.fixed_wing_takeoff()
            else:
                # Vertical takeoff (multicopter or VTOL)
                self.log("Attempting vertical takeoff")
                self.uav.takeoff() # TODO: change to multicopter_takeoff()
                self.takeoff_commanded = True
            # TODO: change takeoff_type to enum
        
        # When in AUTO_LOITER, engage offboard mode after 1 second of elapsed time
        if self.takeoff_elapsed_time >= 1.0 and self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
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
