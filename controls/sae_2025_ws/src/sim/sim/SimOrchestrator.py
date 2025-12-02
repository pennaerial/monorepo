#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import yaml
import importlib
import inspect
import ast
from rclpy.executors import MultiThreadedExecutor


class SimOrchestrator(Node):
    """
    Base simulation class providing UAV position tracking and competition management.
    """

    def __init__(self, competition: str, competition_course: str, use_scoring: bool):
        super().__init__("sim_orchestrator")
        print("Orchestrator Node")
        print(f"Competition: {competition}")
        print(f"Competition course: {competition_course}")
        print(f"Use scoring: {use_scoring}")
        sim_params = self.load_yaml_to_dict(
            os.path.join(
                os.getcwd(), "src", "sim", "sim", "simulations", f"{competition}.yaml"
            )
        )
        world_params = sim_params["world"]
        self.world_node = self.initialize_mode(
            world_params["class"], world_params["params"]
        )
        
        self.scoring_node = None
        if use_scoring:
            scoring_params = sim_params["scoring"]
            self.scoring_node = self.initialize_mode(
                scoring_params["class"], scoring_params["params"]
            )

    def initialize_mode(self, node_path: str, params: dict) -> Node:
        print(f"SimOrchestrator Mode Parameters: {params}")
        module_name, class_name = node_path.rsplit(".", 1)
        module = importlib.import_module(module_name)
        node_class = getattr(module, class_name)

        signature = inspect.signature(node_class.__init__)
        args = {}
        print(f"SimOrchestrator Mode Signature: {signature.parameters.items()}")
        for name, param in signature.parameters.items():
            if name == "self":
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
                and name not in ("node")  # your custom bypasses
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
                hasattr(param.annotation, "__origin__")
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
        with open(filename, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file)
        return data

    def spin(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.add_node(self.world_node)

        if self.scoring_node is not None:
            executor.add_node(self.scoring_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            # destroy nodes cleanly
            self.get_logger().info("Shutting down orchestrator and child nodes")
            self.destroy_node()
            self.world_node.destroy_node()
            if self.scoring_node is not None:
                self.scoring_node.destroy_node()
