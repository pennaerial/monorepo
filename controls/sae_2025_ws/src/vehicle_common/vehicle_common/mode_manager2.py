import os
from abc import ABC, abstractmethod
from pathlib import Path
from time import time
from typing import cast

from pydantic import BaseModel
from rclpy.node import Node
from std_srvs.srv import Trigger

from vehicle_common.vehicle import Vehicle
from vehicle_common.mode import Mode
from vehicle_common.runtime.peer_connections import (
    declared_remote_peer_names,
    normalize_vehicle_name,
)
from vehicle_common.runtime.vision_loader import (
    canonical_vision_node_path,
    load_vision_class,
)
from vehicle_common.mode_loader import ModeRegistry
from vehicle_common.runtime.mission_loader import RuntimeMission


MISSION_STARTED_MARKER_ENV = "PENNAIR_MISSION_STARTED_MARKER_PATH"


class ModeManager(Node, ABC):
    """Shared mission manager plumbing for a single bound vehicle."""

    def __init__(
        self,
        node_name: str,
        *,
        vehicle_name: str = "",
        auto_launch: bool = True,
        peer_heartbeat_hz: float = 10.0,
        peer_stale_timeout_s: float = 0.5,
    ) -> None:
        super().__init__(node_name)
        self.vehicle: Vehicle | None = None
        self.modes: dict[str, Mode] = {}
        self.transitions: dict[str, dict[str, str]] = {}
        self.active_mode: str | None = None
        self.last_update_time = time()
        self._vision_clients = {}
        self.timer = None
        self.auto_launch = bool(auto_launch)
        self._auto_launch_timer = None
        self._runtime_vehicle_name = normalize_vehicle_name(vehicle_name)
        self.peer_heartbeat_hz = float(peer_heartbeat_hz)
        self.peer_stale_timeout_s = float(peer_stale_timeout_s)
        if self.peer_heartbeat_hz <= 0.0:
            raise ValueError(f"peer_heartbeat_hz must be positive, got {self.peer_heartbeat_hz!r}.")
        if self.peer_stale_timeout_s <= 0.0:
            raise ValueError(
                f"peer_stale_timeout_s must be positive, got {self.peer_stale_timeout_s!r}."
            )
        self._shared_mode_state = {}
        self._current_comm_builder = None
        self._runtime_closed = False
        self.start_mission_service = self.create_service(
            Trigger, "mode_manager/start_mission", self._start_mission_callback
        )
        self._clear_mission_started_marker()
        if self.auto_launch:
            self._auto_launch_timer = self.create_timer(0.1, self._maybe_auto_launch)

    @abstractmethod
    def spin_once(self) -> None:
        """Execute one iteration of the mode manager's control loop."""

    def get_active_mode(self) -> Mode:
        return self.modes[cast(str, self.active_mode)]

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _mission_started_marker_path(self) -> Path:
        explicit_path = os.environ.get(MISSION_STARTED_MARKER_ENV, "").strip()
        if explicit_path:
            return Path(explicit_path)

        deploy_root = os.environ.get("DEPLOY_ROOT", "").strip()
        if deploy_root:
            return Path(deploy_root) / "state" / "mission-started"

        runtime_vehicle_name = str(getattr(self, "_runtime_vehicle_name", "") or "").strip()
        if runtime_vehicle_name:
            return Path("/tmp/pennair") / runtime_vehicle_name / "mission-started"
        return Path("/tmp/pennair/mission-started")

    def _write_mission_started_marker(self) -> None:
        marker_path = self._mission_started_marker_path()
        vehicle_name = str(getattr(self, "_runtime_vehicle_name", "") or "").strip()
        marker_text = f"started_at={time():.6f}\nvehicle_name={vehicle_name}\n"
        try:
            marker_path.parent.mkdir(parents=True, exist_ok=True)
            marker_path.write_text(marker_text, encoding="utf-8")
        except OSError as exc:
            self.get_logger().warn(f"Failed to write mission-start marker {marker_path}: {exc}")

    def _clear_mission_started_marker(self) -> None:
        marker_path = self._mission_started_marker_path()
        try:
            marker_path.unlink(missing_ok=True)
        except OSError as exc:
            self.get_logger().warn(f"Failed to clear mission-start marker {marker_path}: {exc}")

    def setup_vision(self, vision_nodes: list[str]) -> None:
        nodes_to_setup = [node for node in vision_nodes if node]
        if not nodes_to_setup:
            return
        if self.vehicle is None or not getattr(self.vehicle, "has_camera", False):
            raise ValueError("Vision nodes require an active vehicle camera contract.")

        for vision_node in nodes_to_setup:
            vision_class = load_vision_class(vision_node)
            key = canonical_vision_node_path(vision_class)
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
        vehicle = self.vehicle
        if vehicle is None:
            raise ValueError("Vision nodes require an active vehicle camera contract.")
        service_name = vehicle.vision_service_name(vision_class)
        while True:
            client = super().create_client(vision_class.srv, service_name)
            if client.wait_for_service(timeout_sec=1.0):
                return client, service_name
            super().destroy_client(client)
            self.get_logger().info(f"Service {service_name} not available yet, waiting again...")

    def get_vision_client(self, vision_node):
        key = canonical_vision_node_path(vision_node)
        if key not in self._vision_clients:
            raise KeyError(f"Vision client '{key}' is not registered.")
        return self._vision_clients[key]

    def _mode_peer_names(self, mode_or_class: object) -> tuple[str, ...]:
        return declared_remote_peer_names(mode_or_class, self._runtime_vehicle_name)

    def initialize_mode(self, mode_id: str, params: BaseModel) -> Mode:
        registered_mode = ModeRegistry.get().get_registered_mode(mode_id)
        mode_class = registered_mode.mode_cls
        instance = mode_class()
        instance.initialize(self, self.vehicle, params)
        return instance

    def setup_modes(self, runtime_mission: RuntimeMission) -> None:
        for mode_name, runtime_mode in runtime_mission.modes.items():
            mode_instance = self.initialize_mode(runtime_mode.mode, runtime_mode._validated_params)
            self.modes[mode_name] = mode_instance
            self.get_logger().info(f"Mode {mode_name} registered.")
            self.transitions[mode_name] = runtime_mode.transitions

    def transition(self, state: str) -> str:
        self.get_logger().info(f"Transitioning from {self.active_mode} based on state {state}.")
        return self.transitions[self.active_mode][state]

    def switch_mode(self, mode_name: str) -> None:
        if self.active_mode:
            previous_mode = self.get_active_mode()
            previous_mode.deactivate()

        if mode_name in self.modes:
            self.active_mode = mode_name
            mode = self.get_active_mode()
            mode.activate()
        else:
            self.get_logger().error(f"Mode {mode_name} not found.")

    def run_active_mode(self, current_time: float) -> None:
        if not self.active_mode:
            return

        time_delta = current_time - self.last_update_time
        self.last_update_time = current_time
        mode = self.get_active_mode()
        try:
            mode.update(time_delta)
        except Exception as exc:
            self.get_logger().error(f"Error in mode {self.active_mode}: {exc}")
            self.handle_mode_state("error")
            return

        state = mode.check_status()
        self.handle_mode_state(state)

    def _deactivate_active_mode(self) -> None:
        if not self.active_mode:
            return

        mode_name = self.active_mode
        mode = self.modes.get(mode_name)
        self.active_mode = None
        if mode is None or not getattr(mode, "active", False):
            return

        try:
            mode.deactivate()
        except Exception as exc:
            self.get_logger().warn(f"Failed to deactivate mode {mode_name} during shutdown: {exc}")

    def _stop_vehicle(self) -> None:
        if self.vehicle is None:
            return
        stop_method = getattr(self.vehicle, "stop", None)
        if callable(stop_method):
            try:
                stop_method()
            except Exception as exc:
                self.get_logger().warn(f"Failed to stop vehicle during shutdown: {exc}")

    def _start_mission_callback(self, request, response):
        started = self.start_mission()
        response.success = True
        response.message = "Starting Mission!" if started else "Mission already started."
        return response

    def start_mission(self) -> bool:
        if self.timer is not None:
            return False
        self._cancel_auto_launch_timer()
        self.last_update_time = time()
        self.get_logger().info("MODE MANAGER | Starting Mission!")
        self.timer = self.create_timer(0.1, self.spin_once)
        self._write_mission_started_marker()
        return True

    def _cancel_auto_launch_timer(self) -> None:
        if self._auto_launch_timer is None:
            return
        cancel = getattr(self._auto_launch_timer, "cancel", None)
        if callable(cancel):
            cancel()
        self._auto_launch_timer = None

    def _auto_launch_ready(self) -> bool:
        return True

    def _maybe_auto_launch(self) -> None:
        if self.timer is not None:
            self._cancel_auto_launch_timer()
            return
        if not self._auto_launch_ready():
            return
        self.get_logger().info("MODE MANAGER | Auto-launch conditions satisfied.")
        self.start_mission()

    def handle_mode_state(self, state: str) -> None:
        if state == "error":
            self.get_logger().error(
                f"Error in mode {self.active_mode}. Switching to safe stop behavior."
            )
            self._clear_mission_started_marker()
            self._deactivate_active_mode()
            self._stop_vehicle()
            # self.destroy_node()
        elif state == "terminate":
            self.get_logger().info("Mission has completed.")
            self._clear_mission_started_marker()
            self._deactivate_active_mode()
            self._stop_vehicle()
            # self.destroy_node()
        elif state != "continue":
            next_mode = self.transition(state)
            if next_mode == "terminate":
                self.handle_mode_state("terminate")
            else:
                self.switch_mode(next_mode)
