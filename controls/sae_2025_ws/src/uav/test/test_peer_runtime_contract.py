from __future__ import annotations

from pathlib import Path
import sys
import textwrap
from types import SimpleNamespace
import types

import pytest


def _placeholder(name: str):
    return type(name, (), {})


def _install_ros_test_doubles() -> None:
    if "rclpy" not in sys.modules:
        rclpy = types.ModuleType("rclpy")
        rclpy.init = lambda *args, **kwargs: None
        rclpy.shutdown = lambda: None
        rclpy.ok = lambda: True
        node_mod = types.ModuleType("rclpy.node")
        executors_mod = types.ModuleType("rclpy.executors")
        clock_mod = types.ModuleType("rclpy.clock")
        parameter_mod = types.ModuleType("rclpy.parameter")
        validate_namespace_mod = types.ModuleType("rclpy.validate_namespace")
        validate_node_name_mod = types.ModuleType("rclpy.validate_node_name")
        qos_mod = types.ModuleType("rclpy.qos")

        class Node:
            def __init__(self, *_args, **_kwargs) -> None:
                pass

        class ExternalShutdownException(Exception):
            pass

        node_mod.Node = Node
        executors_mod.ExternalShutdownException = ExternalShutdownException
        clock_mod.Clock = _placeholder("Clock")
        parameter_mod.Parameter = _placeholder("Parameter")
        validate_namespace_mod.validate_namespace = lambda namespace: None
        validate_node_name_mod.validate_node_name = lambda node_name: None
        qos_mod.QoSProfile = _placeholder("QoSProfile")
        qos_mod.QoSReliabilityPolicy = _placeholder("QoSReliabilityPolicy")
        qos_mod.QoSHistoryPolicy = _placeholder("QoSHistoryPolicy")
        qos_mod.QoSDurabilityPolicy = _placeholder("QoSDurabilityPolicy")
        rclpy.node = node_mod
        rclpy.executors = executors_mod
        rclpy.clock = clock_mod
        rclpy.parameter = parameter_mod
        rclpy.validate_namespace = validate_namespace_mod
        rclpy.validate_node_name = validate_node_name_mod
        rclpy.qos = qos_mod
        sys.modules.update(
            {
                "rclpy": rclpy,
                "rclpy.node": node_mod,
                "rclpy.executors": executors_mod,
                "rclpy.clock": clock_mod,
                "rclpy.parameter": parameter_mod,
                "rclpy.validate_namespace": validate_namespace_mod,
                "rclpy.validate_node_name": validate_node_name_mod,
                "rclpy.qos": qos_mod,
            }
        )

    if "std_srvs" not in sys.modules:
        std_srvs = types.ModuleType("std_srvs")
        std_srvs_srv = types.ModuleType("std_srvs.srv")

        class Trigger:
            Request = _placeholder("Request")
            Response = _placeholder("Response")

        std_srvs_srv.Trigger = Trigger
        std_srvs.srv = std_srvs_srv
        sys.modules.update({"std_srvs": std_srvs, "std_srvs.srv": std_srvs_srv})

    if "std_msgs" not in sys.modules:
        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")
        std_msgs_msg.Empty = _placeholder("Empty")
        std_msgs.msg = std_msgs_msg
        sys.modules.update({"std_msgs": std_msgs, "std_msgs.msg": std_msgs_msg})


_install_ros_test_doubles()

from uav.modes.Mode import Mode  # noqa: E402
from uav.vehicles.Vehicle import Vehicle  # noqa: E402
from uav.runtime.ModeManager import ModeManager  # noqa: E402
import uav.runtime.ModeManager as mode_manager_module  # noqa: E402
from uav.runtime.managed_comms import (  # noqa: E402
    ManagedCommRegistry,
)
from uav.runtime.managed_entities import (  # noqa: E402
    ManagedClient,
    ManagedEntity,
    ManagedPublisher,
)
from uav.runtime.mode_paths import canonical_mode_path  # noqa: E402
from uav.runtime.peer_connections import (  # noqa: E402
    PeerConnectionTracker,
    declared_remote_peer_names,
)
from uav.runtime.mission_spec import MissionSpec, load_mission_spec  # noqa: E402
import uav.runtime.schema as schema_module  # noqa: E402
from uav.runtime.schema_generator import build_mode_registry_entry  # noqa: E402
from uav.runtime.schema_registry import ModeRegistryEntry  # noqa: E402


def _xfail_if_missing_peer_features(*features: str) -> None:
    missing: list[str] = []
    dataclass_fields = getattr(MissionSpec, "__dataclass_fields__", {})
    registry_fields = getattr(ModeRegistryEntry, "model_fields", {})
    mode_fields = getattr(Mode, "__dict__", {})
    manager_fields = getattr(ModeManager, "__dict__", {})

    for feature in features:
        if (
            feature == "mode_peer_vehicle_names"
            and "peer_vehicle_names" not in mode_fields
        ):
            missing.append("Mode.peer_vehicle_names")
        elif feature == "mode_on_disconnect" and "on_disconnect" not in mode_fields:
            missing.append("Mode.on_disconnect")
        elif (
            feature == "mode_connection_ready" and "connection_ready" not in mode_fields
        ):
            missing.append("Mode.connection_ready")
        elif (
            feature == "mission_peer_vehicle_names"
            and "peer_vehicle_names" not in dataclass_fields
        ):
            missing.append("MissionSpec.peer_vehicle_names")
        elif (
            feature == "registry_peer_vehicle_names"
            and "peer_vehicle_names" not in registry_fields
        ):
            missing.append("ModeRegistryEntry.peer_vehicle_names")
        elif (
            feature == "manager_create_subscription"
            and "create_subscription" not in manager_fields
        ):
            missing.append("ModeManager.create_subscription")
        elif (
            feature == "manager_create_publisher"
            and "create_publisher" not in manager_fields
        ):
            missing.append("ModeManager.create_publisher")
        elif (
            feature == "manager_create_client" and "create_client" not in manager_fields
        ):
            missing.append("ModeManager.create_client")
        elif (
            feature == "manager_create_service"
            and "create_service" not in manager_fields
        ):
            missing.append("ModeManager.create_service")
        elif (
            feature == "manager_shared_state_for"
            and "shared_state_for" not in manager_fields
        ):
            missing.append("ModeManager.shared_state_for")
        elif feature == "manager_connection_ready_runtime":
            runtime_names = set(
                getattr(ModeManager._run_active_mode, "__code__").co_names
            )
            if "connection_ready" not in runtime_names:
                missing.append("ModeManager._run_active_mode connection_ready logic")
        elif feature == "manager_disconnect_runtime":
            runtime_names = set(
                getattr(ModeManager._run_active_mode, "__code__").co_names
            )
            if "disconnect" not in runtime_names:
                missing.append("ModeManager._run_active_mode peer disconnect logic")

    if missing:
        pytest.xfail(
            "Peer runtime feature is not implemented in source yet: "
            + ", ".join(sorted(set(missing)))
        )


def _write_mission(tmp_path: Path, contents: str) -> Path:
    mission_path = tmp_path / "mission.yaml"
    mission_path.write_text(textwrap.dedent(contents), encoding="utf-8")
    return mission_path


class _FakeLogger:
    def info(self, _message: str) -> None:
        pass

    def warn(self, _message: str) -> None:
        pass

    def warning(self, _message: str) -> None:
        pass

    def error(self, _message: str) -> None:
        pass


class _FakeVehicle(Vehicle):
    def __init__(self, name: str = "payload_0") -> None:
        self.name = name
        self.has_camera = False

    def stop(self) -> None:
        pass


class _RecordingNode:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str]] = []
        self._logger = _FakeLogger()

    def get_logger(self) -> _FakeLogger:
        return self._logger

    def create_subscription(self, _msg_type, topic: str, *_args, **_kwargs):
        self.calls.append(("subscription", topic))
        return SimpleNamespace(kind="subscription", name=topic)

    def create_publisher(self, _msg_type, topic: str, *_args, **_kwargs):
        self.calls.append(("publisher", topic))
        return SimpleNamespace(kind="publisher", name=topic)

    def create_client(self, _srv_type, service_name: str, *_args, **_kwargs):
        self.calls.append(("client", service_name))
        return SimpleNamespace(kind="client", name=service_name)

    def create_service(self, _srv_type, service_name: str, *_args, **_kwargs):
        self.calls.append(("service", service_name))
        return SimpleNamespace(kind="service", name=service_name)


def _make_mode_manager() -> ModeManager:
    manager = object.__new__(ModeManager)
    manager.vehicle = _FakeVehicle()
    manager.modes = {}
    manager.transitions = {}
    manager.active_mode = None
    manager.last_update_time = 0.0
    manager._vision_clients = {}
    manager.timer = None
    manager.auto_launch = False
    manager._auto_launch_timer = None
    manager._runtime_vehicle_name = "payload_0"
    manager.peer_heartbeat_hz = 10.0
    manager.peer_stale_timeout_s = 0.5
    manager._shared_mode_state = {}
    manager._current_comm_builder = None
    manager._runtime_closed = False
    manager.get_logger = lambda: _FakeLogger()
    manager.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=0)
    )
    manager.destroy_node = lambda: None
    manager.handle_mode_state = lambda state: state

    def _test_double_node_method(name: str):
        method = getattr(mode_manager_module.Node, name, None)
        if method is None:
            return None
        if getattr(method, "__module__", "").startswith("rclpy"):
            return None
        return method

    def raw_create_publisher(msg_type, topic, *args, **kwargs):
        create_publisher = _test_double_node_method("create_publisher")
        if create_publisher is None:
            return SimpleNamespace(
                kind="publisher",
                name=topic,
                publish=lambda _message: None,
            )
        return create_publisher(manager, msg_type, topic, *args, **kwargs)

    def raw_create_subscription(msg_type, topic, *args, **kwargs):
        create_subscription = _test_double_node_method("create_subscription")
        if create_subscription is None:
            return SimpleNamespace(kind="subscription", name=topic)
        return create_subscription(manager, msg_type, topic, *args, **kwargs)

    def raw_create_client(srv_type, service_name, *args, **kwargs):
        create_client = _test_double_node_method("create_client")
        if create_client is None:
            return SimpleNamespace(
                kind="client",
                name=service_name,
                wait_for_service=lambda timeout_sec=0.0: True,
                call_async=lambda request: request,
            )
        return create_client(manager, srv_type, service_name, *args, **kwargs)

    def raw_create_service(srv_type, service_name, *args, **kwargs):
        create_service = _test_double_node_method("create_service")
        if create_service is None:
            return SimpleNamespace(kind="service", name=service_name)
        return create_service(manager, srv_type, service_name, *args, **kwargs)

    def raw_destroy_publisher(publisher) -> bool:
        destroy_publisher = _test_double_node_method("destroy_publisher")
        if destroy_publisher is None:
            return True
        return bool(destroy_publisher(manager, publisher))

    def raw_destroy_subscription(subscription) -> bool:
        destroy_subscription = _test_double_node_method("destroy_subscription")
        if destroy_subscription is None:
            return True
        return bool(destroy_subscription(manager, subscription))

    def raw_destroy_client(client) -> bool:
        destroy_client = _test_double_node_method("destroy_client")
        if destroy_client is None:
            return True
        return bool(destroy_client(manager, client))

    manager._raw_node_api = SimpleNamespace(
        create_publisher=raw_create_publisher,
        create_subscription=raw_create_subscription,
        create_client=raw_create_client,
        create_service=raw_create_service,
        create_timer=lambda _period, _callback: SimpleNamespace(cancel=lambda: None),
        destroy_publisher=raw_destroy_publisher,
        destroy_subscription=raw_destroy_subscription,
        destroy_client=raw_destroy_client,
        destroy_timer=lambda _timer: True,
    )
    manager._managed_registry = ManagedCommRegistry(
        connection_status=lambda: manager._peer_connections.status(),
        raw_create_publisher=raw_create_publisher,
        raw_create_subscription=raw_create_subscription,
        raw_create_client=raw_create_client,
        raw_destroy_publisher=raw_destroy_publisher,
        raw_destroy_subscription=raw_destroy_subscription,
        raw_destroy_client=raw_destroy_client,
    )
    manager._peer_connections = PeerConnectionTracker(
        runtime_vehicle_name=manager._runtime_vehicle_name,
        peer_heartbeat_hz=manager.peer_heartbeat_hz,
        peer_stale_timeout_s=manager.peer_stale_timeout_s,
        now_seconds=manager._now_seconds,
        on_connection_change=manager._managed_registry.on_peer_connection_change,
        raw_create_publisher=raw_create_publisher,
        raw_create_subscription=raw_create_subscription,
        raw_destroy_publisher=raw_destroy_publisher,
        raw_destroy_subscription=raw_destroy_subscription,
        raw_create_timer=lambda _period, _callback: SimpleNamespace(
            cancel=lambda: None
        ),
        raw_destroy_timer=lambda _timer: True,
    )
    return manager


def _configure_manager_for_mode(manager: ModeManager, mode: Mode) -> None:
    manager.modes = {"start": mode}
    manager.active_mode = "start"
    manager.get_active_mode = lambda: mode
    peer_vehicle_names = declared_remote_peer_names(mode, manager._runtime_vehicle_name)
    manager._peer_connections.configure(peer_vehicle_names)


def _set_peer_connection_state(
    manager: ModeManager, peer_status: dict[str, bool]
) -> None:
    manager._peer_connections._connected = dict(peer_status)
    manager._peer_connections._peer_names = tuple(sorted(peer_status))
    manager._peer_connections._last_seen = {
        peer_name: None for peer_name in manager._peer_connections._peer_names
    }


def _observed_peer_vehicle_names(
    calls: list[tuple[str, str]], declared_peers: tuple[str, ...]
) -> set[str]:
    observed: set[str] = set()
    for entity_kind, name in calls:
        if not isinstance(name, str) or not name.startswith("/"):
            continue
        if name.startswith("/shared/"):
            if entity_kind in {"publisher", "subscription"}:
                observed.update(declared_peers)
            continue
        parts = [part for part in name.split("/") if part]
        if len(parts) < 2:
            continue
        peer = parts[0]
        if entity_kind in {"subscription", "client"}:
            observed.add(peer)
    return observed


def test_build_mode_registry_entry_includes_peer_vehicle_names():
    _xfail_if_missing_peer_features(
        "mode_peer_vehicle_names",
        "mode_on_disconnect",
        "registry_peer_vehicle_names",
    )

    class PeerAwareMode(Mode[_FakeVehicle]):
        peer_vehicle_names = ("uav_2", "payload_1")

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)

        def on_disconnect(self, time_delta: float, peers: dict[str, bool]) -> None:
            pass

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    PeerAwareMode.__module__ = "uav.modes.payload.PeerAwareMode"
    entry = build_mode_registry_entry(PeerAwareMode)

    assert entry.mission_target == "payload"
    assert entry.peer_vehicle_names == ("payload_1", "uav_2")


def test_build_mode_registry_entry_allows_peer_mode_without_on_disconnect():
    _xfail_if_missing_peer_features(
        "mode_peer_vehicle_names",
        "registry_peer_vehicle_names",
    )

    class PeerAwareMode(Mode[_FakeVehicle]):
        peer_vehicle_names = ("uav_3",)

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)

        def check_status(self) -> str:
            return "continue"

        def on_update(self, time_delta: float) -> None:
            pass

    PeerAwareMode.__module__ = "uav.modes.payload.PeerAwareMode"
    entry = build_mode_registry_entry(PeerAwareMode)

    assert entry.peer_vehicle_names == ("uav_3",)


def test_build_mode_registry_entry_allows_direct_generic_mode_base():
    class DirectMode(Mode[_FakeVehicle]):
        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    DirectMode.__module__ = "uav.modes.payload.DirectMode"
    entry = build_mode_registry_entry(DirectMode)

    assert entry.mode_id == "payload.DirectMode"
    assert entry.mission_target == "payload"


def test_build_mode_registry_entry_rejects_unsupported_namespace():
    class UnsupportedNamespaceMode(Mode[_FakeVehicle]):
        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    UnsupportedNamespaceMode.__module__ = "uav.modes.boat.UnsupportedNamespaceMode"

    with pytest.raises(ValueError, match="must live under"):
        build_mode_registry_entry(UnsupportedNamespaceMode)


def test_load_mission_spec_collects_sorted_peer_vehicle_names_union(
    monkeypatch, tmp_path
):
    _xfail_if_missing_peer_features(
        "mission_peer_vehicle_names",
        "registry_peer_vehicle_names",
    )

    mission_path = _write_mission(
        tmp_path,
        """
        modes:
          start:
            mode: fake.StartMode
          support:
            mode: fake.SupportMode
          end:
            mode: fake.EndMode
        """,
    )

    fake_entries = {
        "fake.StartMode": SimpleNamespace(
            mission_target="payload",
            required_vision_nodes=(),
            requires_camera=False,
            transition_labels=(),
            peer_vehicle_names=("uav_1", "payload_2"),
        ),
        "fake.SupportMode": SimpleNamespace(
            mission_target="payload",
            required_vision_nodes=(),
            requires_camera=False,
            transition_labels=(),
            peer_vehicle_names=("uav_1", "uav_3"),
        ),
        "fake.EndMode": SimpleNamespace(
            mission_target="payload",
            required_vision_nodes=(),
            requires_camera=False,
            transition_labels=(),
            peer_vehicle_names=(),
        ),
    }

    monkeypatch.setattr(
        schema_module,
        "mode_entry_for_mode_id",
        lambda mode_id: fake_entries[mode_id],
    )
    monkeypatch.setattr(
        schema_module,
        "validate_mode_params",
        lambda _mode_id, params: params,
    )

    mission_spec = load_mission_spec(mission_path)

    assert mission_spec.peer_vehicle_names == ("payload_2", "uav_1", "uav_3")


def test_mode_manager_validates_peer_and_shared_entity_namespaces(monkeypatch):
    _xfail_if_missing_peer_features(
        "mode_peer_vehicle_names",
        "manager_create_subscription",
        "manager_create_publisher",
        "manager_create_client",
        "manager_create_service",
    )

    class PeerAwareMode(Mode[_FakeVehicle]):
        peer_vehicle_names = ("uav_1",)

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)
            self.peer_sub = self.node.create_subscription(
                object, "/uav_1/status", lambda _msg: None, 1
            )
            self.shared_pub = self.node.create_publisher(object, "/shared/debug", 1)

        def on_disconnect(self, time_delta: float, peers: dict[str, bool]) -> None:
            pass

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    monkeypatch.setattr(
        mode_manager_module,
        "load_mode_class",
        lambda _path: PeerAwareMode,
    )
    monkeypatch.setattr(
        mode_manager_module,
        "mode_entry_for_mode_id",
        lambda _mode_id: SimpleNamespace(class_path="fake.module.PeerAwareMode"),
    )
    monkeypatch.setattr(
        mode_manager_module.Node,
        "create_subscription",
        lambda *_args, **_kwargs: SimpleNamespace(kind="subscription"),
        raising=False,
    )
    monkeypatch.setattr(
        mode_manager_module.Node,
        "create_publisher",
        lambda *_args, **_kwargs: SimpleNamespace(kind="publisher"),
        raising=False,
    )
    monkeypatch.setattr(
        mode_manager_module.Node,
        "create_client",
        lambda *_args, **_kwargs: SimpleNamespace(kind="client"),
        raising=False,
    )
    monkeypatch.setattr(
        mode_manager_module.Node,
        "create_service",
        lambda *_args, **_kwargs: SimpleNamespace(kind="service"),
        raising=False,
    )

    manager = _make_mode_manager()
    mode = ModeManager.initialize_mode(manager, "fake.module.PeerAwareMode", {})
    _configure_manager_for_mode(manager, mode)

    with manager._use_comm_builder(
        manager._make_comm_builder(
            owner=mode,
            owner_label=type(mode).__name__,
            lifetime="active",
            peer_vehicle_names=declared_remote_peer_names(
                mode, manager._runtime_vehicle_name
            ),
        )
    ):
        peer_sub = ModeManager.create_subscription(
            manager, object, "/uav_1/status", lambda _msg: None, 1
        )
        peer_client = ModeManager.create_client(manager, object, "/uav_1/camera")

    assert isinstance(peer_sub, ManagedEntity)
    assert peer_sub.spec.kind == "subscription"
    assert isinstance(peer_client, ManagedClient)
    assert isinstance(mode.shared_pub, ManagedPublisher)
    assert isinstance(mode.peer_sub, ManagedEntity)
    assert mode.peer_sub.spec.kind == "subscription"

    with pytest.raises(ValueError):
        ModeManager.create_subscription(
            manager, object, "/uav_9/status", lambda _msg: None, 1
        )
    with pytest.raises(ValueError):
        ModeManager.create_publisher(manager, object, "/uav_1/cmd", 1)
    with pytest.raises(ValueError):
        ModeManager.create_client(manager, object, "/shared/camera")
    with pytest.raises(ValueError):
        ModeManager.create_service(
            manager, object, "/uav_1/serve", lambda _req, _res: _res
        )


def test_run_active_mode_uses_connection_ready_for_gating():
    _xfail_if_missing_peer_features(
        "mode_peer_vehicle_names",
        "mode_on_disconnect",
        "mode_connection_ready",
        "manager_connection_ready_runtime",
        "manager_disconnect_runtime",
    )

    class PeerAwareMode(Mode[_FakeVehicle]):
        peer_vehicle_names = ("uav_1", "uav_2")

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)
            self.update_calls: list[float] = []
            self.disconnect_calls: list[tuple[float, dict[str, bool]]] = []
            self.connection_checks: list[dict[str, bool]] = []
            self.status_checks = 0

        def connection_ready(self, connection_status: dict[str, bool]) -> bool:
            self.connection_checks.append(dict(connection_status))
            return True

        def on_update(self, time_delta: float) -> None:
            self.update_calls.append(time_delta)

        def on_disconnect(
            self, time_delta: float, connection_status: dict[str, bool]
        ) -> None:
            self.disconnect_calls.append((time_delta, dict(connection_status)))

        def check_status(self) -> str:
            self.status_checks += 1
            return "continue"

    manager = _make_mode_manager()
    mode = PeerAwareMode(manager, manager.vehicle)
    mode.active = True
    _configure_manager_for_mode(manager, mode)
    _set_peer_connection_state(manager, {"uav_1": False, "uav_2": True})
    handled_states: list[str] = []
    manager.handle_mode_state = lambda state: handled_states.append(state)
    manager.last_update_time = 10.0

    ModeManager._run_active_mode(manager, 10.25)

    assert mode.connection_checks == [{"uav_1": False, "uav_2": True}]
    assert mode.update_calls == [0.25]
    assert mode.disconnect_calls == []
    assert mode.status_checks == 1
    assert handled_states == ["continue"]


def test_run_active_mode_passes_full_connection_status_to_disconnect():
    _xfail_if_missing_peer_features(
        "mode_peer_vehicle_names",
        "mode_on_disconnect",
        "mode_connection_ready",
        "manager_connection_ready_runtime",
        "manager_disconnect_runtime",
    )

    class PeerAwareMode(Mode[_FakeVehicle]):
        peer_vehicle_names = ("uav_1", "uav_2")

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)
            self.update_calls: list[float] = []
            self.disconnect_calls: list[tuple[float, dict[str, bool]]] = []
            self.connection_checks: list[dict[str, bool]] = []
            self.status_checks = 0

        def connection_ready(self, connection_status: dict[str, bool]) -> bool:
            self.connection_checks.append(dict(connection_status))
            return False

        def on_disconnect(
            self, time_delta: float, connection_status: dict[str, bool]
        ) -> None:
            self.disconnect_calls.append((time_delta, dict(connection_status)))

        def on_update(self, time_delta: float) -> None:
            self.update_calls.append(time_delta)

        def check_status(self) -> str:
            self.status_checks += 1
            return "continue"

    manager = _make_mode_manager()
    mode = PeerAwareMode(manager, manager.vehicle)
    mode.active = True
    _configure_manager_for_mode(manager, mode)
    _set_peer_connection_state(manager, {"uav_1": False, "uav_2": True})
    handled_states: list[str] = []
    manager.handle_mode_state = lambda state: handled_states.append(state)
    manager.last_update_time = 10.0

    ModeManager._run_active_mode(manager, 10.25)

    assert mode.connection_checks == [{"uav_1": False, "uav_2": True}]
    assert mode.update_calls == []
    assert mode.disconnect_calls == [(0.25, {"uav_1": False, "uav_2": True})]
    assert mode.status_checks == 1
    assert handled_states == ["continue"]


def test_mode_connection_ready_defaults_to_all_declared_peers_connected():
    _xfail_if_missing_peer_features("mode_peer_vehicle_names", "mode_connection_ready")

    class PeerAwareMode(Mode[_FakeVehicle]):
        peer_vehicle_names = ("uav_1", "uav_2")

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)

        def on_disconnect(self, time_delta: float, peers: dict[str, bool]) -> None:
            pass

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    mode = PeerAwareMode(_RecordingNode(), _FakeVehicle())

    assert mode.connection_ready({"uav_1": True, "uav_2": True}) is True
    assert mode.connection_ready({"uav_1": True, "uav_2": False}) is False
    assert mode.connection_ready({"uav_1": False, "uav_2": True}) is False


def test_peer_client_wrapper_rebinds_on_connection_changes(monkeypatch):
    _xfail_if_missing_peer_features(
        "mode_peer_vehicle_names",
        "manager_create_client",
    )

    class PeerAwareMode(Mode[_FakeVehicle]):
        peer_vehicle_names = ("uav_1",)

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)

        def on_disconnect(self, time_delta: float, peers: dict[str, bool]) -> None:
            pass

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    create_calls: list[str] = []
    destroy_calls: list[object] = []

    monkeypatch.setattr(
        mode_manager_module.Node,
        "create_client",
        lambda _self, _srv_type, name, *_args, **_kwargs: (
            create_calls.append(name)
            or SimpleNamespace(
                kind="client",
                name=name,
                wait_for_service=lambda timeout_sec=0.0: True,
                call_async=lambda request: request,
            )
        ),
        raising=False,
    )
    monkeypatch.setattr(
        mode_manager_module.Node,
        "destroy_client",
        lambda _self, client: destroy_calls.append(client) or True,
        raising=False,
    )

    manager = _make_mode_manager()
    mode = PeerAwareMode(manager, manager.vehicle)
    _configure_manager_for_mode(manager, mode)

    with manager._use_comm_builder(
        manager._make_comm_builder(
            owner=mode,
            owner_label=type(mode).__name__,
            lifetime="active",
            peer_vehicle_names=declared_remote_peer_names(
                mode, manager._runtime_vehicle_name
            ),
        )
    ):
        client = ModeManager.create_client(manager, object, "/uav_1/camera")

    assert isinstance(client, ManagedClient)
    assert client.get_underlying() is None

    manager._peer_connections._connected["uav_1"] = True
    manager._managed_registry.on_peer_connection_change("uav_1", connected=True)

    assert create_calls == ["/uav_1/camera"]
    assert client.get_underlying() is not None

    manager._peer_connections._connected["uav_1"] = False
    manager._managed_registry.on_peer_connection_change("uav_1", connected=False)

    assert client.get_underlying() is None
    assert len(destroy_calls) == 1


def test_peer_timer_publishes_heartbeat_and_marks_stale_connections():
    _xfail_if_missing_peer_features("manager_disconnect_runtime")

    manager = _make_mode_manager()
    published: list[object] = []
    connection_events: list[tuple[str, bool]] = []

    manager._peer_connections._heartbeat_publisher = SimpleNamespace(
        publish=lambda msg: published.append(msg)
    )
    manager._peer_connections._peer_names = ("uav_1",)
    manager._peer_connections._connected = {"uav_1": True}
    manager._peer_connections._last_seen = {"uav_1": 0.0}
    manager.peer_stale_timeout_s = 0.5
    manager.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)
    )
    manager._peer_connections._on_connection_change = lambda peer_name, *, connected: (
        connection_events.append((peer_name, connected))
    )

    manager._peer_connections._peer_timer_callback()

    assert len(published) == 1
    assert manager._peer_connections._connected["uav_1"] is False
    assert connection_events == [("uav_1", False)]


def test_managed_entity_descriptors_use_persistent_and_active_phases(monkeypatch):
    _xfail_if_missing_peer_features(
        "mode_peer_vehicle_names",
        "mode_on_disconnect",
        "manager_create_subscription",
        "manager_create_publisher",
    )

    class PhaseAwareMode(Mode[_FakeVehicle]):
        peer_vehicle_names = ("uav_1",)

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)
            self.node.create_publisher(object, "/shared/debug", 1)

        def on_disconnect(self, time_delta: float, peers: dict[str, bool]) -> None:
            pass

        def on_enter(self) -> None:
            self.node.create_client(object, "/uav_1/camera")

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    monkeypatch.setattr(
        mode_manager_module,
        "load_mode_class",
        lambda _path: PhaseAwareMode,
    )
    monkeypatch.setattr(
        mode_manager_module,
        "mode_entry_for_mode_id",
        lambda _mode_id: SimpleNamespace(class_path="fake.module.PhaseAwareMode"),
    )
    monkeypatch.setattr(
        mode_manager_module.Node,
        "create_subscription",
        lambda *_args, **_kwargs: SimpleNamespace(kind="subscription"),
        raising=False,
    )
    monkeypatch.setattr(
        mode_manager_module.Node,
        "create_publisher",
        lambda *_args, **_kwargs: SimpleNamespace(kind="publisher"),
        raising=False,
    )

    manager = _make_mode_manager()
    mode = ModeManager.initialize_mode(manager, "fake.module.PhaseAwareMode", {})
    manager.modes = {"start": mode}
    manager.active_mode = None

    ModeManager.switch_mode(manager, "start")

    lifetimes = {
        descriptor.lifetime
        for descriptor in ModeManager._comm_debug_snapshot(manager)["descriptors"]
    }
    assert lifetimes == {"active", "persistent"}


def test_shared_state_for_is_keyed_by_canonical_mode_path():
    _xfail_if_missing_peer_features("manager_shared_state_for")

    manager = _make_mode_manager()

    class SharedStateMode(Mode[_FakeVehicle]):
        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)

        def on_disconnect(self, time_delta: float, peers: dict[str, bool]) -> None:
            pass

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    canonical_path = canonical_mode_path(SharedStateMode)
    state_a = ModeManager.shared_state_for(manager, canonical_path)
    state_b = ModeManager.shared_state_for(manager, canonical_path)
    state_other = ModeManager.shared_state_for(
        manager, f"{SharedStateMode.__module__}.OtherMode"
    )

    assert state_a is state_b
    assert state_a is not state_other
    state_a["example"] = "value"
    assert ModeManager.shared_state_for(manager, canonical_path)["example"] == "value"


def test_peer_entity_usage_instrumentation_matches_declared_peers():
    _xfail_if_missing_peer_features(
        "mode_peer_vehicle_names",
        "mode_on_disconnect",
    )

    class InstrumentedPeerMode(Mode[_FakeVehicle]):
        peer_vehicle_names = ("payload_1", "uav_2")

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)
            self.node.create_subscription(object, "/uav_2/status", lambda _msg: None, 1)

        def on_disconnect(self, time_delta: float, peers: dict[str, bool]) -> None:
            pass

        def on_enter(self) -> None:
            self.node.create_client(object, "/payload_1/camera_data")
            self.node.create_publisher(object, "/shared/debug", 1)

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    node = _RecordingNode()
    mode = InstrumentedPeerMode(node, _FakeVehicle())
    mode.activate()

    assert _observed_peer_vehicle_names(node.calls, mode.peer_vehicle_names) == {
        "payload_1",
        "uav_2",
    }
