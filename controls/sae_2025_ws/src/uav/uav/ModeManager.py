import rclpy
from rclpy.node import Node
from time import time
import Mode
import UAV
from typing import List
from vision_nodes import VisionNode
import yaml
import importlib

#TODO: Think about how to encode the mission structure (when to switch `Modes`, etc.)
class ModeManager(Node):
    """
    A ROS 2 node for managing UAV modes and mission logic.
    """

    def __init__(self, mode_map: str):
        super().__init__('mission_node')
        self.modes = {}
        self.transitions = {}
        self.active_mode = None
        self.last_update_time = time()
        self.uav = UAV()
        self.get_logger().info("Mission Node has started!")

        self.setup_modes(mode_map)

    def on_enter(self) -> None:
        """
        Logic executed when this mode is activated.
        """
        self.switch_mode(self.starting_mode)
        
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
    
    def initialize_mode(self, mode_path: List[str], params: dict) -> Mode:
        """
        Initialize a mode instance.

        Args:
            mode_path (str): The path to the mode class.
            params (dict): Parameters to pass to the mode

        Returns:
            Mode: An instance of the mode.
        """
        module_name, class_name = mode_path.rsplit('.', 1)
        module = importlib.import_module(module_name)
        class_ = getattr(module, class_name)
        return class_(self, self.uav, params)
    
    def setup_modes(self, mode_map: str) -> None:
        """
        Setup the modes for the mission node.

        Args:
            mode_yaml (dict): A dictionary mapping mode names to instances of the mode.
        """
        mode_yaml = self.load_yaml_to_dict(mode_map)

        for mode_name in mode_yaml.keys():
            mode_path = mode_yaml[mode_name]['class']
            params = mode_yaml[mode_name].get('params', {})
            mode_instance = self.initialize_mode(mode_path, params)
            self.add_mode(mode_name, mode_instance)
            
            self.transitions[mode_name] = mode_yaml[mode_name].get('transitions', {})

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

    def transition(self, state: str) -> str:
        """
        Transition to the next mode based on the current state.

        Args:
            state (str): The current state of the mode.

        Returns:
            str: The name of the next mode to transition to.
        """
        return self.transitions[self.active_mode][state]

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
                self.switch_mode(self.transition(state))

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
        """
        Load a yaml file into a dictionary.

        Args:
            filename (str): The path to the yaml file.

        Returns:
            dict: The yaml file as a dictionary.
        """
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        return data
    

if __name__ == '__main__':
    mission_node = ModeManager('test_mode_manager')
    print(mission_node.modes)
    print(mission_node.transitions)
    print(mission_node.active_mode)
