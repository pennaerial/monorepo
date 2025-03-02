#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from time import time
from uav import Mode, UAV
from typing import List
from uav.vision_nodes import VisionNode
import yaml
import importlib
import inspect
import ast

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
        self.uav = UAV(self)
        self.get_logger().info("Mission Node has started!")
        self.vision_nodes = {}
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
    
    def initialize_mode(self, mode_path: str, params: dict) -> Mode:
        module_name, class_name = mode_path.rsplit('.', 1)
        module = importlib.import_module(module_name)
        mode_class = getattr(module, class_name)

        signature = inspect.signature(mode_class.__init__)
        args = {}

        for name, param in signature.parameters.items():
            if name == 'self':
                continue
            if name in params:
                param_value = params[name]
                if param.annotation in (str, inspect.Parameter.empty) or name in ('node', 'uav'):
                    args[name] = param_value
                else:
                    try:
                        args[name] = ast.literal_eval(param_value)
                    except (ValueError, SyntaxError):
                        raise ValueError(f"Parameter '{name}' must be a valid literal for mode '{mode_path}'. Received: {param_value}")
            elif param.default != inspect.Parameter.empty:
                args[name] = param.default
            else:
                raise ValueError(f"Missing required parameter '{name}' for mode '{mode_path}'")

        return mode_class(**args)
    
    def setup_modes(self, mode_map: str) -> None:
        """
        Setup the modes for the mission node.

        Args:
            mode_yaml (dict): A dictionary mapping mode names to instances of the mode.
        """
        mode_yaml = self.load_yaml_to_dict(mode_map)

        for mode_name in mode_yaml.keys():
            mode_info = mode_yaml[mode_name]

            mode_path = mode_info['class']

            params = mode_info.get('params', {}) | {'node': self, 'uav': self.uav}
            mode = self.initialize_mode(mode_path, params)
            self.add_mode(mode_name, mode)
            self.transitions[mode_name] = mode_info.get('transitions', {})
        
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
            
            if state == "continue":
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

    def load_yaml_to_dict(self, filename: str):
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
    # mission_node.spin()
    
    mission_node.transition('continue')
    print('transition continue')
    print(mission_node.modes)
    print(mission_node.transitions)
    print(mission_node.active_mode)
    
    mission_node.transition('continue')
    print('transition continu 2')
    print(mission_node.modes)
    print(mission_node.transitions)
    print(mission_node.active_mode)
     