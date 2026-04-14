#!/usr/bin/env python3
import inspect
from dataclasses import dataclass
from time import time
from typing import Any, Literal, get_type_hints

from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Empty

from uav.vehicles.Vehicle import Vehicle
from uav.modes.Mode import Mode
from .mission_spec import MissionSpec, load_mode_class
from .managed_entities import (
    ManagedClient,
    ManagedEntity,
    ManagedEntitySpec,
    ManagedPublisher,
    ManagedSubscription,
)
from .vision_loader import canonical_vision_node_path, load_vision_class

HEARTBEAT_TOPIC_SUFFIX = "mode_manager/heartbeat"
_SHARED_NAMESPACE_PREFIX = "/shared/"


@dataclass(frozen=True)
class _ModeEntityContext:
    owner: object
    owner_label: str
    phase: Literal["construct", "activate"]
    peer_vehicle_names: tuple[str, ...]


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
        self._runtime_vehicle_name = self._normalize_vehicle_name(vehicle_name)
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
        self._managed_entity_context = None
        self._managed_entities = {}
        self._peer_heartbeat_publisher = None
        self._peer_timer = None
        self._peer_heartbeat_subscriptions = {}
        self._peer_connected = {}
        self._peer_last_seen = {}
        self._mission_peer_names = ()
        self.start_mission_service = self.create_service(
            Trigger, "mode_manager/start_mission", self._start_mission_callback
        )
        if self._runtime_vehicle_name:
            self._peer_heartbeat_publisher = super().create_publisher(
                Empty,
                self._heartbeat_topic_for(self._runtime_vehicle_name),
                1,
            )
            self._peer_timer = super().create_timer(
                1.0 / self.peer_heartbeat_hz,
                self._peer_timer_callback,
            )
        if self.auto_launch:
            self._auto_launch_timer = self.create_timer(0.1, self._maybe_auto_launch)

    def get_active_mode(self) -> Mode:
        return self.modes[self.active_mode]

    @staticmethod
    def _normalize_vehicle_name(vehicle_name: str | None) -> str:
        return str(vehicle_name or "").strip().strip("/")

    @classmethod
    def _normalize_peer_vehicle_names(
        cls, peer_vehicle_names: tuple[str, ...] | list[str] | set[str] | None
    ) -> tuple[str, ...]:
        return tuple(
            sorted(
                {
                    cls._normalize_vehicle_name(peer_name)
                    for peer_name in tuple(peer_vehicle_names or ())
                    if cls._normalize_vehicle_name(peer_name)
                }
            )
        )

    def _mode_peer_vehicle_names(self, mode_or_class: object) -> tuple[str, ...]:
        peer_vehicle_names = self._normalize_peer_vehicle_names(
            getattr(mode_or_class, "peer_vehicle_names", ())
        )
        if not self._runtime_vehicle_name:
            return peer_vehicle_names
        return tuple(
            peer_name
            for peer_name in peer_vehicle_names
            if peer_name != self._runtime_vehicle_name
        )

    def _validate_mode_peer_contract(
        self, mode_class: type[Mode], mode_path: str
    ) -> tuple[str, ...]:
        peer_vehicle_names = self._mode_peer_vehicle_names(mode_class)
        if peer_vehicle_names and mode_class.on_disconnect is Mode.on_disconnect:
            raise ValueError(
                f"Mode '{mode_path}' must override on_disconnect() when "
                "peer_vehicle_names is non-empty."
            )
        return peer_vehicle_names

    def _heartbeat_topic_for(self, vehicle_name: str) -> str:
        return f"/{vehicle_name}/{HEARTBEAT_TOPIC_SUFFIX}"

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @property
    def mission_peer_vehicle_names(self) -> tuple[str, ...]:
        return self._mission_peer_names

    @property
    def peer_connection_status(self) -> dict[str, bool]:
        return dict(self._peer_connected)

    @property
    def managed_entity_descriptors(self) -> tuple:
        return tuple(
            wrapper.spec.descriptor() for wrapper in self._managed_entities.values()
        )

    def configure_peer_vehicle_names(
        self, peer_vehicle_names: tuple[str, ...] | list[str] | set[str]
    ) -> None:
        normalized_peer_names = [
            peer_name
            for peer_name in self._normalize_peer_vehicle_names(peer_vehicle_names)
            if peer_name != self._runtime_vehicle_name
        ]
        current_peer_names = set(self._mission_peer_names)
        next_peer_names = set(normalized_peer_names)

        for peer_name in current_peer_names - next_peer_names:
            subscription = self._peer_heartbeat_subscriptions.pop(peer_name, None)
            if subscription is not None:
                super().destroy_subscription(subscription)
            self._peer_connected.pop(peer_name, None)
            self._peer_last_seen.pop(peer_name, None)

        for peer_name in normalized_peer_names:
            if peer_name in current_peer_names:
                continue
            self._peer_connected[peer_name] = False
            self._peer_last_seen[peer_name] = None
            self._peer_heartbeat_subscriptions[peer_name] = super().create_subscription(
                Empty,
                self._heartbeat_topic_for(peer_name),
                lambda _msg, peer_name=peer_name: self._peer_heartbeat_callback(
                    peer_name
                ),
                1,
            )

        self._mission_peer_names = tuple(sorted(next_peer_names))

    def _peer_heartbeat_callback(self, peer_name: str) -> None:
        self._peer_last_seen[peer_name] = self._now_seconds()
        if self._peer_connected.get(peer_name, False):
            return
        self._peer_connected[peer_name] = True
        self._handle_peer_connection_change(peer_name, connected=True)

    def _peer_timer_callback(self) -> None:
        if self._peer_heartbeat_publisher is not None:
            self._peer_heartbeat_publisher.publish(Empty())

        current_time = self._now_seconds()
        for peer_name in self._mission_peer_names:
            last_seen = self._peer_last_seen.get(peer_name)
            if last_seen is None:
                continue
            if current_time - last_seen <= self.peer_stale_timeout_s:
                continue
            if not self._peer_connected.get(peer_name, False):
                continue
            self._peer_connected[peer_name] = False
            self._handle_peer_connection_change(peer_name, connected=False)

    def _handle_peer_connection_change(self, peer_name: str, *, connected: bool) -> None:
        for wrapper in list(self._managed_entities.values()):
            if wrapper.destroyed or peer_name not in wrapper.spec.peers:
                continue
            if wrapper.spec.scope == "peer":
                if connected:
                    self._ensure_managed_underlying(wrapper)
                else:
                    self._destroy_managed_underlying(wrapper)
                continue
            if connected and wrapper.spec.scope == "shared":
                self._refresh_managed_underlying(wrapper)

    def _bind_managed_entities(self, old_owner: object, new_owner: object) -> None:
        for wrapper in self._managed_entities.values():
            if wrapper.spec.owner is old_owner:
                wrapper.spec.owner = new_owner
                wrapper.spec.owner_label = type(new_owner).__name__

    def _destroy_managed_entities_for_owner(
        self,
        owner: object,
        *,
        phase: Literal["construct", "activate"] | None = None,
    ) -> None:
        for wrapper in list(self._managed_entities.values()):
            if wrapper.spec.owner is not owner:
                continue
            if phase is not None and wrapper.spec.phase != phase:
                continue
            self._destroy_managed_entity(wrapper)

    def _required_peers_for_mode(self, mode: Mode) -> tuple[str, ...]:
        return self._mode_peer_vehicle_names(mode)

    def _disconnected_peers_for_mode(self, mode: Mode) -> tuple[str, ...]:
        return tuple(
            peer_name
            for peer_name in self._required_peers_for_mode(mode)
            if not self._peer_connected.get(peer_name, False)
        )

    def _special_scope_for_name(
        self,
        kind: Literal["publisher", "subscription", "client", "service"],
        name: str,
    ) -> tuple[str | None, tuple[str, ...]]:
        normalized_name = str(name or "").strip()
        if not normalized_name:
            return None, ()

        context = self._managed_entity_context
        if normalized_name.startswith(_SHARED_NAMESPACE_PREFIX):
            if kind not in {"publisher", "subscription"}:
                raise ValueError(
                    f"{kind} cannot target shared namespace '{normalized_name}'."
                )
            if context is None:
                raise ValueError(
                    f"Shared {kind} '{normalized_name}' may only be created during "
                    "mode construction or activation."
                )
            return "shared", context.peer_vehicle_names

        if not normalized_name.startswith("/"):
            return None, ()

        prefix = normalized_name[1:].split("/", 1)[0]
        if not prefix:
            return None, ()
        if prefix == self._runtime_vehicle_name:
            return None, ()
        if context is None:
            raise ValueError(
                f"Peer {kind} '{normalized_name}' may only be created during mode "
                "construction or activation."
            )
        if prefix not in context.peer_vehicle_names:
            raise ValueError(
                f"Mode '{context.owner_label}' tried to create peer {kind} "
                f"for undeclared peer '{prefix}'."
            )
        if kind not in {"subscription", "client"}:
            raise ValueError(
                f"{kind} cannot target peer namespace '{normalized_name}'."
            )
        return "peer", (prefix,)

    def _make_managed_wrapper(self, spec: ManagedEntitySpec) -> ManagedEntity:
        wrapper_type = {
            "publisher": ManagedPublisher,
            "subscription": ManagedSubscription,
            "client": ManagedClient,
        }[spec.kind]
        return wrapper_type(spec)

    def _create_managed_underlying(self, wrapper: ManagedEntity):
        spec = wrapper.spec
        if spec.kind == "publisher":
            return super().create_publisher(
                spec.interface_type,
                spec.name,
                *spec.args,
                **spec.kwargs,
            )
        if spec.kind == "subscription":
            return super().create_subscription(
                spec.interface_type,
                spec.name,
                *spec.args,
                **spec.kwargs,
            )
        return super().create_client(
            spec.interface_type,
            spec.name,
            *spec.args,
            **spec.kwargs,
        )

    def _destroy_managed_underlying(self, wrapper: ManagedEntity) -> None:
        underlying = wrapper.detach()
        if underlying is None:
            return
        if wrapper.spec.kind == "publisher":
            super().destroy_publisher(underlying)
        elif wrapper.spec.kind == "subscription":
            super().destroy_subscription(underlying)
        else:
            super().destroy_client(underlying)

    def _should_have_managed_underlying(self, wrapper: ManagedEntity) -> bool:
        if wrapper.destroyed:
            return False
        if wrapper.spec.scope == "shared":
            return True
        return all(
            self._peer_connected.get(peer_name, False)
            for peer_name in wrapper.spec.peers
        )

    def _ensure_managed_underlying(self, wrapper: ManagedEntity) -> None:
        if not self._should_have_managed_underlying(wrapper):
            return
        if wrapper.get_underlying() is not None:
            return
        wrapper.attach(self._create_managed_underlying(wrapper))

    def _refresh_managed_underlying(self, wrapper: ManagedEntity) -> None:
        if wrapper.destroyed:
            return
        self._destroy_managed_underlying(wrapper)
        self._ensure_managed_underlying(wrapper)

    def _register_managed_entity(
        self,
        kind: Literal["publisher", "subscription", "client"],
        scope: Literal["peer", "shared"],
        interface_type,
        name: str,
        args: tuple[Any, ...],
        kwargs: dict[str, Any],
        peers: tuple[str, ...],
    ):
        context = self._managed_entity_context
        if context is None:
            raise ValueError(
                f"Managed {kind} '{name}' may only be created during mode "
                "construction or activation."
            )
        spec = ManagedEntitySpec(
            kind=kind,
            scope=scope,
            phase=context.phase,
            owner=context.owner,
            owner_label=context.owner_label,
            interface_type=interface_type,
            name=str(name),
            args=args,
            kwargs=dict(kwargs),
            peers=tuple(peers),
        )
        wrapper = self._make_managed_wrapper(spec)
        self._managed_entities[id(wrapper)] = wrapper
        self._ensure_managed_underlying(wrapper)
        return wrapper

    def _destroy_managed_entity(self, wrapper: ManagedEntity) -> bool:
        if id(wrapper) not in self._managed_entities:
            return False
        self._destroy_managed_underlying(wrapper)
        wrapper.mark_destroyed()
        self._managed_entities.pop(id(wrapper), None)
        return True

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

    def initialize_mode(self, mode_path: str, params: dict) -> Mode:
        mode_class = load_mode_class(mode_path)
        peer_vehicle_names = self._validate_mode_peer_contract(mode_class, mode_path)
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

        owner_token = object()
        previous_context = self._managed_entity_context
        self._managed_entity_context = _ModeEntityContext(
            owner=owner_token,
            owner_label=mode_path,
            phase="construct",
            peer_vehicle_names=peer_vehicle_names,
        )
        try:
            mode_instance = mode_class(**args)
        except Exception:
            self._destroy_managed_entities_for_owner(owner_token)
            raise
        finally:
            self._managed_entity_context = previous_context

        self._bind_managed_entities(owner_token, mode_instance)
        return mode_instance

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
            previous_mode = self.get_active_mode()
            previous_mode.deactivate()
            self._destroy_managed_entities_for_owner(previous_mode, phase="activate")

        if mode_name in self.modes:
            self.active_mode = mode_name
            mode = self.get_active_mode()
            previous_context = self._managed_entity_context
            self._managed_entity_context = _ModeEntityContext(
                owner=mode,
                owner_label=type(mode).__name__,
                phase="activate",
                peer_vehicle_names=self._required_peers_for_mode(mode),
            )
            try:
                mode.activate()
            except Exception:
                self._destroy_managed_entities_for_owner(mode, phase="activate")
                raise
            finally:
                self._managed_entity_context = previous_context
        else:
            self.get_logger().error(f"Mode {mode_name} not found.")

    def _run_active_mode(self, current_time: float) -> None:
        if not self.active_mode:
            return

        time_delta = current_time - self.last_update_time
        self.last_update_time = current_time
        mode = self.get_active_mode()
        disconnected_peers = self._disconnected_peers_for_mode(mode)
        try:
            if disconnected_peers:
                mode.disconnect(time_delta, disconnected_peers)
            else:
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
            self._destroy_managed_entities_for_owner(mode, phase="activate")
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
            self._deactivate_active_mode()
            self._stop_vehicle()
            self.destroy_node()
        elif state == "terminate":
            self.get_logger().info("Mission has completed.")
            self._deactivate_active_mode()
            self._stop_vehicle()
            self.destroy_node()
        elif state != "continue":
            self.switch_mode(self.transition(state))

    def create_publisher(self, msg_type, topic: str, *args, **kwargs):
        scope, peers = self._special_scope_for_name("publisher", topic)
        if scope is None:
            return super().create_publisher(msg_type, topic, *args, **kwargs)
        return self._register_managed_entity(
            "publisher",
            scope,
            msg_type,
            topic,
            args,
            kwargs,
            peers,
        )

    def create_subscription(self, msg_type, topic: str, *args, **kwargs):
        scope, peers = self._special_scope_for_name("subscription", topic)
        if scope is None:
            return super().create_subscription(msg_type, topic, *args, **kwargs)
        return self._register_managed_entity(
            "subscription",
            scope,
            msg_type,
            topic,
            args,
            kwargs,
            peers,
        )

    def create_client(self, srv_type, srv_name: str, *args, **kwargs):
        scope, peers = self._special_scope_for_name("client", srv_name)
        if scope is None:
            return super().create_client(srv_type, srv_name, *args, **kwargs)
        return self._register_managed_entity(
            "client",
            scope,
            srv_type,
            srv_name,
            args,
            kwargs,
            peers,
        )

    def create_service(self, srv_type, srv_name: str, *args, **kwargs):
        scope, _peers = self._special_scope_for_name("service", srv_name)
        if scope is None:
            return super().create_service(srv_type, srv_name, *args, **kwargs)
        raise ValueError(
            f"create_service() does not support managed scope for '{srv_name}'."
        )

    def destroy_publisher(self, publisher) -> bool:
        if isinstance(publisher, ManagedPublisher):
            return self._destroy_managed_entity(publisher)
        return super().destroy_publisher(publisher)

    def destroy_subscription(self, subscription) -> bool:
        if isinstance(subscription, ManagedSubscription):
            return self._destroy_managed_entity(subscription)
        return super().destroy_subscription(subscription)

    def destroy_client(self, client) -> bool:
        if isinstance(client, ManagedClient):
            return self._destroy_managed_entity(client)
        return super().destroy_client(client)
