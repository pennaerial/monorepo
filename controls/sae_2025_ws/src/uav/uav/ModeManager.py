import rclpy
from rclpy.node import Node
from time import time
from uav import Mode

class ModeManager(Node):
    """
    A ROS 2 node for managing UAV modes and mission logic.
    """

    def __init__(self):
        super().__init__('mission_node')
        self.modes = {}
        self.active_mode = None
        self.last_update_time = time()

        self.get_logger().info("Mission Node has started!")

    def add_mode(self, mode_name: str, mode_instance: Mode) -> None:
        """
        Register a mode to the mission node.

        Args:
            mode_name (str): Name of the mode.
            mode_instance (Mode): An instance of the mode.
        """
        self.modes[mode_name] = mode_instance
        self.get_logger().info(f"Mode {mode_name} registered.")

    def switch_mode(self, mode_name: str) -> None:
        """
        Switch to a new mode.

        Args:
            mode_name (str): Name of the mode to activate.
        """
        if self.active_mode:
            self.active_mode.deactivate()

        if mode_name in self.modes:
            self.active_mode = self.modes[mode_name]
            self.active_mode.activate()
        else:
            self.get_logger().error(f"Mode {mode_name} not found.")

    def spin_once(self) -> None:
        """
        Execute one spin cycle of the node, updating the active mode.
        """
        current_time = time()
        time_delta = current_time - self.last_update_time
        self.last_update_time = current_time

        if self.active_mode:
            self.active_mode.update(time_delta)

    def spin(self):
        """
        Run the mission node loop.
        """
        try:
            while rclpy.ok():
                self.spin_once()
        except KeyboardInterrupt:
            self.get_logger().info("Mission Node shutting down.")
        finally:
            rclpy.shutdown()