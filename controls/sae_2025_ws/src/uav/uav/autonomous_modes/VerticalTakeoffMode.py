from uav import Mode
from rclpy.node import Node
from uav import UAV

class VerticalTakeoffMode(Mode):
    """
    A mode for taking off vertically.
    """

    def __init__(self, node: Node, uav: UAV):
        """
        Initialize the VerticalTakeoffMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            altitude (float): The altitude to take off to.
        """
        super().__init__(node, uav, [])
        self.command_sent = False

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for taking off vertically.
        """
        if not self.command_sent:
            self.command_sent = True
            super().uav.takeoff()
    
    def check_status(self) -> str:
        """
        Check the status of the mode.

        Returns:
            str: The status of the mode.
        """
        if self.uav.check_takeoff_complete():
            return "complete"
        else:
            return "loop"