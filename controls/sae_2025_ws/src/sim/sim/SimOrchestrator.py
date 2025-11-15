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
from sim.world_gen import WorldNode
from sim.scoring import ScoringNode

class SimOrchestrator(Node):
    """
    Base simulation class providing UAV position tracking and competition management.
    """
    
    def __init__(self, yml_path):
        super().__init__('sim_orchestrator')
        print("Orhc NODE!!!!!!!!!!!!!")
        print(yml_path)
        sim_params = self.load_yaml_to_dict(yml_path)
        print(sim_params)
        world_params = sim_params['world']
        self.world_node = self.initialize_mode(world_params['class'], world_params['params'])
        scoring_params = sim_params['scoring']
        self.scoring_node = self.initialize_mode(scoring_params['class'], scoring_params['params'])

    
    def initialize_mode(self, node_path: str, params: dict) -> Node:
        print(params)
        module_name, class_name = node_path.rsplit('.', 1)
        module = importlib.import_module(module_name)
        node_class = getattr(module, class_name)

        signature = inspect.signature(node_class.__init__)
        args = {}
        print(signature.parameters.items())
        for name, param in signature.parameters.items():
            if name == 'self':
                continue

            # Get a value for this parameter, or default, or error
            if name in params:
                param_value = params[name]
            elif param.default != inspect.Parameter.empty:
                param_value = param.default
            else:
                raise ValueError(
                    f"Missing required parameter '{name}' for mode '{node_path}'"
                )

            # Only try literal_eval if:
            #  - the *value* is a string (coming from CLI/env/etc),
            #  - and the annotation says it's not a string,
            #  - and it's not one of your special cases.
            if (
                isinstance(param_value, str)
                and param.annotation not in (str, inspect.Parameter.empty)
                and name not in ('node')   # your custom bypasses
            ):
                try:
                    param_value = ast.literal_eval(param_value)
                except (ValueError, SyntaxError):
                    raise ValueError(
                        f"Parameter '{name}' must be a valid literal for mode "
                        f"'{node_path}'. Received: {param_value}"
                    )

            # Optional: convert lists to tuples for Tuple[...] annotations
            if (
                hasattr(param.annotation, '__origin__')
                and param.annotation.__origin__ is tuple
                and isinstance(param_value, list)
            ):
                param_value = tuple(param_value)

            args[name] = param_value

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