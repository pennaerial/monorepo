from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV

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

    def on_update(self, time_delta: float):
        """
        Update the mode
        """
        assert self.uav.is_vtol, "UAV is not a VTOL"
        # If vehicle_type is None, we haven't received VTOL status yet, so wait
        if self.uav.vehicle_type is None:
            self.log("Waiting for VTOL status...")
            return
        if self.uav.vehicle_type != self.to_mode:
            self.log(f"Transitioning from {self.uav.vehicle_type} to {self.to_mode}")
            self.uav.vtol_transition_to(self.to_mode)
        else:
            self.log(f"Already in {self.to_mode} mode")
    
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