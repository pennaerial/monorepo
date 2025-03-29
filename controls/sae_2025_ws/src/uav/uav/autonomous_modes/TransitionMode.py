from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV
from px4_msgs.msg import VehicleStatus

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
        if self.uav.vehicle_type != self.to_mode:
            self.uav.vtol_transition_to(self.to_mode)
    
    def check_status(self):
        """
        Check the status of the mode
        """
        if self.uav.vehicle_type != self.to_mode:
            return "continue"
        else:
            return "complete"