#!/usr/bin/env python3
from contextlib import contextmanager
from dataclasses import dataclass
import inspect
import os
from pathlib import Path
from time import time
from typing import Any, Callable, Iterator, get_type_hints

from rclpy.node import Node
from std_srvs.srv import Trigger

from uav.vehicles.Vehicle import Vehicle
from uav.modes.Mode import Mode
from .comm_naming import ManagedCommKind, resolve_managed_target
from .comm_policy import ManagedCommLifetime, managed_creation_phase_error
from .managed_comms import ModeCommBuilder
from .managed_registry import ManagedCommRegistry
from .mode_paths import canonical_mode_path
from .mission_spec import MissionSpec, load_mode_class
from .schema import mode_entry_for_mode_id
from .peer_connections import (
    PeerConnectionTracker,
    declared_remote_peer_names,
    relevant_connection_status,
    normalize_vehicle_name,
)
from .vision_loader import canonical_vision_node_path, load_vision_class


@dataclass(frozen=True)
class _RawNodeApi:
    create_publisher: Callable[..., object]
    create_subscription: Callable[..., object]
    create_client: Callable[..., object]
    create_service: Callable[..., object]
    create_timer: Callable[..., object]
    destroy_publisher: Callable[[object], bool | None]
    destroy_subscription: Callable[[object], bool | None]
    destroy_client: Callable[[object], bool | None]
    destroy_timer: Callable[[object], bool | None] | None


MISSION_STARTED_MARKER_ENV = "PENNAIR_MISSION_STARTED_MARKER_PATH"


class ModeManager(Node):
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
        self.vehicle = None
        self.modes = {}
        self.transitions = {}
        self.active_mode = None
        self.last_update_time = time()
        self._vision_clients = {}
        self.timer = None
        self.auto_launch = bool(auto_launch)
        self._auto_launch_timer = None
        self._runtime_vehicle_name = normalize_vehicle_name(vehicle_name)
        self.peer_heartbeat_hz = float(peer_heartbeat_hz)
        self.peer_stale_timeout_s = float(peer_stale_timeout_s)
        if self.peer_heartbeat_hz <= 0.0:
            raise ValueError(
                f"peer_heartbeat_hz must be positive, got {self.peer_heartbeat_hz!r}."
            )
        if self.peer_stale_timeout_s <= 0.0:
            raise ValueError(
                "peer_stale_timeout_s must be positive, "
                f"got {self.peer_stale_timeout_s!r}."
            )
        self._shared_mode_state = {}
        self._current_comm_builder = None
        self._runtime_closed = False
        self._initialize_comms_runtime(vehicle_name=vehicle_name)
        self.start_mission_service = self.create_service(
            Trigger, "mode_manager/start_mission", self._start_mission_callback
        )
        self._clear_mission_started_marker()
        if self.auto_launch:
            self._auto_launch_timer = self.create_timer(0.1, self._maybe_auto_launch)

    def get_active_mode(self) -> Mode:
        return self.modes[self.active_mode]

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _mission_started_marker_path(self) -> Path:
        explicit_path = os.environ.get(MISSION_STARTED_MARKER_ENV, "").strip()
        if explicit_path:
            return Path(explicit_path)

        deploy_root = os.environ.get("DEPLOY_ROOT", "").strip()
        if deploy_root:
            return Path(deploy_root) / "state" / "mission-started"

        runtime_vehicle_name = str(
            getattr(self, "_runtime_vehicle_name", "") or ""
        ).strip()
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
            self.get_logger().warn(
                f"Failed to write mission-start marker {marker_path}: {exc}"
            )

    def _clear_mission_started_marker(self) -> None:
        marker_path = self._mission_started_marker_path()
        try:
            marker_path.unlink(missing_ok=True)
        except OSError as exc:
            self.get_logger().warn(
                f"Failed to clear mission-start marker {marker_path}: {exc}"
            )

    def shared_state_for(self, mode_or_class: object) -> dict[str, Any]:
        key = canonical_mode_path(mode_or_class)
        return self._shared_mode_state.setdefault(key, {})

    def _comm_debug_snapshot(self) -> dict[str, Any]:
        return {
            "peer_vehicle_names": tuple(self._peer_connections.peer_names),
            "connection_status": self._peer_connections.status(),
            "descriptors": self._managed_registry.debug_descriptors(),
        }

    def _initialize_comms_runtime(self, *, vehicle_name: str) -> None:
        self._runtime_vehicle_name = normalize_vehicle_name(vehicle_name)
        raw_node = super(ModeManager, self)
        self._raw_node_api = _RawNodeApi(
            create_publisher=raw_node.create_publisher,
            create_subscription=raw_node.create_subscription,
            create_client=raw_node.create_client,
            create_service=raw_node.create_service,
            create_timer=raw_node.create_timer,
            destroy_publisher=raw_node.destroy_publisher,
            destroy_subscription=raw_node.destroy_subscription,
            destroy_client=raw_node.destroy_client,
            destroy_timer=getattr(raw_node, "destroy_timer", None),
        )
        self._managed_registry = ManagedCommRegistry(
            connection_status=lambda: self._peer_connections.status(),
            raw_create_publisher=self._raw_node_api.create_publisher,
            raw_create_subscription=self._raw_node_api.create_subscription,
            raw_create_client=self._raw_node_api.create_client,
            raw_destroy_publisher=self._raw_node_api.destroy_publisher,
            raw_destroy_subscription=self._raw_node_api.destroy_subscription,
            raw_destroy_client=self._raw_node_api.destroy_client,
        )
        self._peer_connections = PeerConnectionTracker(
            runtime_vehicle_name=self._runtime_vehicle_name,
            peer_heartbeat_hz=self.peer_heartbeat_hz,
            peer_stale_timeout_s=self.peer_stale_timeout_s,
            now_seconds=self._now_seconds,
            on_connection_change=self._managed_registry.on_peer_connection_change,
            raw_create_publisher=self._raw_node_api.create_publisher,
            raw_create_subscription=self._raw_node_api.create_subscription,
            raw_destroy_publisher=self._raw_node_api.destroy_publisher,
            raw_destroy_subscription=self._raw_node_api.destroy_subscription,
            raw_create_timer=self._raw_node_api.create_timer,
            raw_destroy_timer=self._raw_node_api.destroy_timer,
        )

    def configure_peer_vehicle_names(
        self, peer_vehicle_names: tuple[str, ...] | list[str] | set[str]
    ) -> None:
        self._peer_connections.configure(peer_vehicle_names)

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
        service_name = self.vehicle.vision_service_name(vision_class)
        while True:
            client = super().create_client(vision_class.srv, service_name)
            if client.wait_for_service(timeout_sec=1.0):
                return client, service_name
            super().destroy_client(client)
            self.get_logger().info(
                f"Service {service_name} not available yet, waiting again..."
            )

    def get_vision_client(self, vision_node):
        key = canonical_vision_node_path(vision_node)
        if key not in self._vision_clients:
            raise KeyError(f"Vision client '{key}' is not registered.")
        return self._vision_clients[key]

    def _mode_peer_names(self, mode_or_class: object) -> tuple[str, ...]:
        return declared_remote_peer_names(mode_or_class, self._runtime_vehicle_name)

    def _mode_connection_status(self, mode: Mode) -> dict[str, bool]:
        return relevant_connection_status(
            self._peer_connections.status(),
            peer_vehicle_names=self._mode_peer_names(mode),
        )

    def _make_comm_builder(
        self,
        *,
        owner: object,
        owner_label: str,
        lifetime: ManagedCommLifetime,
        peer_vehicle_names: tuple[str, ...],
    ) -> ModeCommBuilder:
        return ModeCommBuilder(
            owner=owner,
            owner_label=owner_label,
            lifetime=lifetime,
            allowed_peer_names=peer_vehicle_names,
            registry=self._managed_registry,
        )

    @contextmanager
    def _use_comm_builder(self, builder: ModeCommBuilder) -> Iterator[None]:
        previous_builder = self._current_comm_builder
        self._current_comm_builder = builder
        try:
            yield
        finally:
            self._current_comm_builder = previous_builder

    def _raw_method(self, name: str):
        raw_node_api = getattr(self, "_raw_node_api", None)
        if raw_node_api is not None:
            return getattr(raw_node_api, name)
        raw_method = getattr(super(ModeManager, self), name, None)
        if raw_method is not None:
            return raw_method
        return getattr(Node, name)

    def _instantiate_mode(
        self,
        *,
        mode_class: type[Mode],
        args: dict[str, Any],
        mode_id: str,
        peer_vehicle_names: tuple[str, ...],
    ) -> Mode:
        mode_instance = mode_class.__new__(mode_class)
        if not isinstance(mode_instance, mode_class):
            raise TypeError(
                f"Mode '{mode_id}' returned unexpected instance from __new__()."
            )
        builder = self._make_comm_builder(
            owner=mode_instance,
            owner_label=mode_id,
            lifetime="persistent",
            peer_vehicle_names=peer_vehicle_names,
        )
        try:
            with self._use_comm_builder(builder):
                mode_class.__init__(mode_instance, **args)
        except Exception:
            self._managed_registry.destroy_for_owner(
                mode_instance, lifetime="persistent"
            )
            raise
        return mode_instance

    def _create_entity(
        self,
        *,
        kind: ManagedCommKind,
        interface_type: object,
        name: str,
        args: tuple[Any, ...],
        kwargs: dict[str, Any],
    ):
        if not hasattr(self, "_runtime_vehicle_name"):
            return self._raw_method(f"create_{kind}")(
                interface_type,
                name,
                *args,
                **kwargs,
            )
        target = resolve_managed_target(
            name=name,
            runtime_vehicle_name=self._runtime_vehicle_name,
        )
        if target is None:
            return self._raw_method(f"create_{kind}")(
                interface_type,
                name,
                *args,
                **kwargs,
            )
        builder = self._current_comm_builder
        if builder is None:
            raise managed_creation_phase_error(kind, name)
        return builder.create(
            kind=kind,
            interface_type=interface_type,
            name=name,
            target=target,
            args=args,
            kwargs=kwargs,
        )

    def _destroy_entity(self, *, kind: str, entity) -> bool:
        registry = getattr(self, "_managed_registry", None)
        if registry is None:
            return bool(self._raw_method(f"destroy_{kind}")(entity))
        return getattr(registry, f"destroy_{kind}")(entity)

    def initialize_mode(self, mode_id: str, params: dict) -> Mode:
        mode_entry = mode_entry_for_mode_id(mode_id)
        mode_class = load_mode_class(mode_entry.class_path)
        peer_vehicle_names = self._mode_peer_names(mode_class)
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
                self._validate_vehicle_annotation(mode_id, type_hints.get(name))
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
                f"Missing required parameter '{name}' for mode '{mode_id}'"
            )

        unexpected_params = sorted(set(params) - consumed_params)
        if unexpected_params:
            raise ValueError(
                f"Mode '{mode_id}' received unexpected parameter(s): {', '.join(unexpected_params)}"
            )

        return self._instantiate_mode(
            mode_class=mode_class,
            args=args,
            mode_id=mode_id,
            peer_vehicle_names=peer_vehicle_names,
        )

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
            mode = self.initialize_mode(mode_info.mode_id, mode_info.params)
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
            previous_mode = self.get_active_mode()
            previous_mode.deactivate()
            self._managed_registry.destroy_for_owner(previous_mode, lifetime="active")

        if mode_name in self.modes:
            self.active_mode = mode_name
            mode = self.get_active_mode()
            builder = self._make_comm_builder(
                owner=mode,
                owner_label=type(mode).__name__,
                lifetime="active",
                peer_vehicle_names=self._mode_peer_names(mode),
            )
            try:
                with self._use_comm_builder(builder):
                    mode.activate()
            except Exception:
                self._managed_registry.destroy_for_owner(mode, lifetime="active")
                raise
        else:
            self.get_logger().error(f"Mode {mode_name} not found.")

    def _run_active_mode(self, current_time: float) -> None:
        if not self.active_mode:
            return

        time_delta = current_time - self.last_update_time
        self.last_update_time = current_time
        mode = self.get_active_mode()
        connection_status = self._mode_connection_status(mode)
        try:
            if mode.connection_ready(connection_status):
                mode.update(time_delta)
            else:
                mode.disconnect(time_delta, connection_status)
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
            self._managed_registry.destroy_for_owner(mode, lifetime="active")
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to deactivate mode {mode_name} during shutdown: {exc}"
            )

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
        response.message = (
            "Starting Mission!" if started else "Mission already started."
        )
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
            self.destroy_node()
        elif state == "terminate":
            self.get_logger().info("Mission has completed.")
            self._clear_mission_started_marker()
            self._deactivate_active_mode()
            self._stop_vehicle()
            self.destroy_node()
        elif state != "continue":
            next_mode = self.transition(state)
            if next_mode == "terminate":
                self.handle_mode_state("terminate")
            else:
                self.switch_mode(next_mode)

    def create_publisher(self, msg_type, topic: str, *args, **kwargs):
        return self._create_entity(
            kind="publisher",
            interface_type=msg_type,
            name=topic,
            args=args,
            kwargs=kwargs,
        )

    def create_subscription(self, msg_type, topic: str, *args, **kwargs):
        return self._create_entity(
            kind="subscription",
            interface_type=msg_type,
            name=topic,
            args=args,
            kwargs=kwargs,
        )

    def create_client(self, srv_type, srv_name: str, *args, **kwargs):
        return self._create_entity(
            kind="client",
            interface_type=srv_type,
            name=srv_name,
            args=args,
            kwargs=kwargs,
        )

    def create_service(self, srv_type, srv_name: str, *args, **kwargs):
        return self._create_entity(
            kind="service",
            interface_type=srv_type,
            name=srv_name,
            args=args,
            kwargs=kwargs,
        )

    def destroy_publisher(self, publisher) -> bool:
        return self._destroy_entity(kind="publisher", entity=publisher)

    def destroy_subscription(self, subscription) -> bool:
        return self._destroy_entity(kind="subscription", entity=subscription)

    def destroy_client(self, client) -> bool:
        return self._destroy_entity(kind="client", entity=client)

    def _close_runtime_helpers(self) -> None:
        if getattr(self, "_runtime_closed", False):
            return
        self._runtime_closed = True
        managed_registry = getattr(self, "_managed_registry", None)
        if managed_registry is not None:
            managed_registry.close()
        peer_connections = getattr(self, "_peer_connections", None)
        if peer_connections is not None:
            peer_connections.close()

    def destroy_node(self) -> bool:
        self._clear_mission_started_marker()
        if getattr(self, "active_mode", None):
            self._deactivate_active_mode()
        self._stop_vehicle()
        self._close_runtime_helpers()
        return bool(super().destroy_node())
