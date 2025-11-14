#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from geometry_msgs.msg import Point
from typing import Optional, Tuple, Dict, Any
import threading
import time
import numpy as np
import yaml
import importlib
import inspect
import ast
from rclpy.executors import MultiThreadedExecutor

class SimOrchestrator(Node):
    """
    Base simulation class providing UAV position tracking and competition management.
    """
    
    def __init__(self, yml_path):
        super().__init__('sim_orchestrator')
        sim_params = self.load_yaml_to_dict(yml_path)
        world_params = sim_params['world']
        self.world_node = self.initialize_mode(world_params['class'], world_params['params'])
        scoring_params = sim_params['scoring']
        self.scoring_node = self.initialize_mode(scoring_params['class'], scoring_params['params'])

    
    def initialize_mode(self, node_path: str, params: dict) -> Mode:
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

        return node_class(**args)

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

    def spin(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.add_node(self.world_node)
        executor.add_node(self.scoring_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            # destroy nodes cleanly
            self.get_logger().info("Shutting down orchestrator and child nodes")
            self.destroy_node()
            self.world_node.destroy_node()
            self.scoring_node.destroy_node()



# def main(args=None):
#     """Test the SimBase class."""
#     rclpy.init(args=args)
    
#     sim_base = SimOrchestrator()
    
#     try:
#         rclpy.spin(sim_base)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         sim_base.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()