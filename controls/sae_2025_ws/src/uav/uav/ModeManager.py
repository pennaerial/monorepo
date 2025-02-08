import rclpy
from rclpy.node import Node
from time import time
from uav import Mode
from uav import UAV
from typing import List
from vision_nodes import VisionNode
import yaml

#TODO: Think about how to encode the mission structure (when to switch `Modes`, etc.)
class ModeManager(Node):
    """
    A ROS 2 node for managing UAV modes and mission logic.
    """

    def __init__(self, mode_map: str, starting_mode: str):
        super().__init__('mission_node')
        self.modes = {}
        self.active_mode = None
        self.last_update_time = time()
        self.uav = UAV()
        self.get_logger().info("Mission Node has started!")
        self.mode_map = self.load_yaml_to_dict(mode_map)
        self.starting_mode = starting_mode
        
    def setup_vision(self, mode: Mode, vision_nodes: List[VisionNode]) -> None:
        """
        Setup the vision node for this mode.

        Args:
            mode (Mode): The mode to setup vision for.
            vision (VisionNode): The vision nodes to setup for this mode.
        """
        for vnode in vision_nodes:
            if vnode.node_name not in self.vision_nodes:
                self.vision_nodes[vnode.node_name] = vnode()
                
        mode.vision_nodes = {vnode.node_name: self.vision_nodes[vnode] for vnode in vision_nodes}

        for vision_node in self.vision_nodes.values():
            if vision_node.service_name not in self.vision_clients:
                client = self.node.create_client(vision_node.custom_service_type, vision_node.service_name)
                while not client.wait_for_service(timeout_sec=1.0):
                    self.node.get_logger().info(f"Service {vision_node.service_name} not available, waiting again...")
                self.vision_clients[vision_node.service_name] = client
            mode.vision_clients[vision_node.service_name] = self.vision_clients[vision_node.service_name]
            mode.vision_clients
    
    def on_enter(self) -> None:
        """
        Logic executed when this mode is activated.
        """
        self.switch_mode(self.starting_mode)

    def add_mode(self, mode_name: str, mode_instance: Mode) -> None:
        """
        Register a mode to the mission node.

        Args:
            mode_name (str): Name of the mode.
            mode_instance (Mode): An instance of the mode.
        """
        self.setup_vision(mode_instance, mode_instance._vision_nodes_list)
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
            
            state = self.active_mode.check_status()
            if state:
                self.switch_mode(self.mode_map[self.active_mode][state])

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
    def load_yaml_to_dict(filename):
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        return data