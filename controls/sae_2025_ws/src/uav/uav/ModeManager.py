#!/usr/bin/env python3
import inspect
from time import time
from typing import Any, get_type_hints
from rclpy.node import Node
import importlib

from uav.Vehicle import Vehicle
from uav.autonomous_modes.Mode import Mode
from uav.mission_spec import MissionSpec, load_mode_class

VISION_NODE_PATH = "uav.vision_nodes"


class ModeManager(Node):
    """Shared mission manager plumbing for a single bound vehicle."""

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.vehicle = None
        self.modes = {}
        self.transitions = {}
        self.active_mode = None
        self.last_update_time = time()
        self._vision_clients = {}

    def get_active_mode(self) -> Mode:
        return self.modes[self.active_mode]

    def setup_vision(self, vision_nodes: list[str]) -> None:
        nodes_to_setup = [node for node in vision_nodes if node]
        if not nodes_to_setup:
            return
        if self.vehicle is None or not getattr(self.vehicle, "has_camera", False):
            raise ValueError("Vision nodes require an active vehicle camera contract.")

        module = importlib.import_module(VISION_NODE_PATH)
        for vision_node in nodes_to_setup:
            vision_class = getattr(module, vision_node)
            key = vision_class.__name__
            if key in self._vision_clients:
                continue
            client, service_name = self._connect_vision_client(vision_class)
            self._vision_clients[key] = client
            self.get_logger().info(
                f"Registered vision client {vision_class.__name__} on {service_name}."
            )

    @property
    def vision_clients(self) -> dict:
        return self._vision_clients

    def _connect_vision_client(self, vision_class):
        service_name = self.vehicle.vision_service_name(vision_class)
        while True:
            client = self.create_client(vision_class.srv, service_name)
            if client.wait_for_service(timeout_sec=1.0):
                return client, service_name
            self.destroy_client(client)
            self.get_logger().info(
                f"Service {service_name} not available yet, waiting again..."
            )

    def get_vision_client(self, vision_node):
        key = vision_node.__name__
        if key not in self._vision_clients:
            raise KeyError(f"Vision client '{key}' is not registered.")
        return self._vision_clients[key]

    def initialize_mode(self, mode_path: str, params: dict) -> Mode:
        mode_class = load_mode_class(mode_path)
        signature = inspect.signature(mode_class.__init__)
        type_hints = get_type_hints(mode_class.__init__)
        args = {}
        consumed_params = set()

        for name, param in signature.parameters.items():
            if name == "self":
                continue
            if name == "node":
                args[name] = self
                continue
            if name == "vehicle":
                self._validate_vehicle_annotation(mode_path, type_hints.get(name))
                args[name] = self.vehicle
                continue
            if name in params:
                args[name] = params[name]
                consumed_params.add(name)
                continue
            if param.default != inspect.Parameter.empty:
                args[name] = param.default
                continue
            raise ValueError(
                f"Missing required parameter '{name}' for mode '{mode_path}'"
            )

        unexpected_params = sorted(set(params) - consumed_params)
        if unexpected_params:
            raise ValueError(
                f"Mode '{mode_path}' received unexpected parameter(s): {', '.join(unexpected_params)}"
            )

        return mode_class(**args)

    def _validate_vehicle_annotation(self, mode_path: str, annotation) -> None:
        if annotation in (None, inspect.Parameter.empty, Vehicle, Any):
            return
        if isinstance(annotation, type) and isinstance(self.vehicle, annotation):
            return
        actual = type(self.vehicle).__name__
        expected = getattr(annotation, "__name__", str(annotation))
        raise TypeError(
            f"Mode '{mode_path}' expects vehicle type '{expected}', got '{actual}'."
        )

    def setup_modes(self, mission_spec: MissionSpec) -> None:
        for mode_name, mode_info in mission_spec.modes.items():
            mode = self.initialize_mode(mode_info.class_path, mode_info.params)
            self.add_mode(mode_name, mode)
            self.transitions[mode_name] = mode_info.transitions

    def add_mode(self, mode_name: str, mode_instance: Mode) -> None:
        self.modes[mode_name] = mode_instance
        self.get_logger().info(f"Mode {mode_name} registered.")

    def transition(self, state: str) -> str:
        self.get_logger().info(
            f"Transitioning from {self.active_mode} based on state {state}."
        )
        return self.transitions[self.active_mode][state]

    def switch_mode(self, mode_name: str) -> None:
        if self.active_mode:
            self.get_active_mode().deactivate()

        if mode_name in self.modes:
            self.active_mode = mode_name
            self.get_active_mode().activate()
        else:
            self.get_logger().error(f"Mode {mode_name} not found.")

    def _run_active_mode(self, current_time: float) -> None:
        if not self.active_mode:
            return

        time_delta = current_time - self.last_update_time
        self.last_update_time = current_time
        try:
            self.get_active_mode().update(time_delta)
        except Exception as exc:
            self.get_logger().error(f"Error in mode {self.active_mode}: {exc}")
            self.handle_mode_state("error")
            return

        state = self.get_active_mode().check_status()
        self.handle_mode_state(state)

    def _stop_vehicle(self) -> None:
        if self.vehicle is None:
            return
        stop_method = getattr(self.vehicle, "stop", None)
        if callable(stop_method):
            stop_method()

    def handle_mode_state(self, state: str) -> None:
        if state == "error":
            self.get_logger().error(
                f"Error in mode {self.active_mode}. Switching to safe stop behavior."
            )
            self._stop_vehicle()
            self.destroy_node()
        elif state == "terminate":
            self.get_logger().info("Mission has completed.")
            self._stop_vehicle()
            self.destroy_node()
        elif state != "continue":
            self.switch_mode(self.transition(state))
