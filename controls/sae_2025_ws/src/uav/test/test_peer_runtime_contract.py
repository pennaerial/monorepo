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
from uav.runtime.ModeManager import ModeManager  # noqa: E402
import uav.runtime.ModeManager as mode_manager_module  # noqa: E402
from uav.runtime.managed_entities import (  # noqa: E402
    ManagedClient,
    ManagedPublisher,
    ManagedSubscription,
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
        if feature == "mode_peer_vehicle_names" and "peer_vehicle_names" not in mode_fields:
            missing.append("Mode.peer_vehicle_names")
        elif feature == "mode_on_disconnect" and "on_disconnect" not in mode_fields:
            missing.append("Mode.on_disconnect")
        elif feature == "mission_peer_vehicle_names" and "peer_vehicle_names" not in dataclass_fields:
            missing.append("MissionSpec.peer_vehicle_names")
        elif feature == "registry_peer_vehicle_names" and "peer_vehicle_names" not in registry_fields:
            missing.append("ModeRegistryEntry.peer_vehicle_names")
        elif feature == "manager_create_subscription" and "create_subscription" not in manager_fields:
            missing.append("ModeManager.create_subscription")
        elif feature == "manager_create_publisher" and "create_publisher" not in manager_fields:
            missing.append("ModeManager.create_publisher")
        elif feature == "manager_create_client" and "create_client" not in manager_fields:
            missing.append("ModeManager.create_client")
        elif feature == "manager_create_service" and "create_service" not in manager_fields:
            missing.append("ModeManager.create_service")
        elif feature == "manager_disconnect_runtime":
            runtime_names = set(getattr(ModeManager._run_active_mode, "__code__").co_names)
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


class _FakeVehicle:
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
    manager._managed_entity_context = None
    manager._managed_entities = {}
    manager._peer_heartbeat_publisher = None
    manager._peer_timer = None
    manager._peer_heartbeat_subscriptions = {}
    manager._peer_connected = {}
    manager._peer_last_seen = {}
    manager._mission_peer_names = ()
    manager.get_logger = lambda: _FakeLogger()
    manager.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=0)
    )
    manager.destroy_node = lambda: None
    manager.handle_mode_state = lambda state: state
    return manager


def _configure_manager_for_mode(manager: ModeManager, mode: Mode) -> None:
    manager.modes = {"start": mode}
    manager.active_mode = "start"
    manager.get_active_mode = lambda: mode
    peer_vehicle_names = manager._mode_peer_vehicle_names(mode)
    manager._mission_peer_names = peer_vehicle_names
    for peer_name in peer_vehicle_names:
        manager._peer_connected.setdefault(peer_name, False)
        manager._peer_last_seen.setdefault(peer_name, None)
    manager._managed_entity_context = mode_manager_module._ModeEntityContext(
        owner=mode,
        owner_label=type(mode).__name__,
        phase="activate",
        peer_vehicle_names=peer_vehicle_names,
    )


def _set_peer_connection_state(manager: ModeManager, peer_status: dict[str, bool]) -> None:
    manager._peer_connected = dict(peer_status)
    manager._mission_peer_names = tuple(sorted(peer_status))


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

    class PeerAwareMode(Mode):
        mission_target = "payload"
        peer_vehicle_names = ("uav_2", "payload_1")

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)

        def on_disconnect(self, time_delta: float, peers: tuple[str, ...]) -> None:
            pass

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    entry = build_mode_registry_entry(PeerAwareMode)

    assert entry.peer_vehicle_names == ("payload_1", "uav_2")


def test_build_mode_registry_entry_rejects_peer_mode_without_on_disconnect():
    _xfail_if_missing_peer_features(
        "mode_peer_vehicle_names",
        "mode_on_disconnect",
        "registry_peer_vehicle_names",
    )

    class InvalidPeerAwareMode(Mode):
        mission_target = "payload"
        peer_vehicle_names = ("uav_3",)

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    with pytest.raises((TypeError, ValueError), match="on_disconnect"):
        build_mode_registry_entry(InvalidPeerAwareMode)


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
            class: fake.StartMode
          support:
            class: fake.SupportMode
          end:
            class: fake.EndMode
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
        "mode_entry_for_class_path",
        lambda class_path: fake_entries[class_path],
    )
    monkeypatch.setattr(
        schema_module,
        "validate_mode_params",
        lambda _class_path, params: params,
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

    class PeerAwareMode(Mode):
        mission_target = "payload"
        peer_vehicle_names = ("uav_1",)

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)

        def on_disconnect(self, time_delta: float, peers: tuple[str, ...]) -> None:
            pass

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

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
    mode = PeerAwareMode(manager, manager.vehicle)
    _configure_manager_for_mode(manager, mode)

    peer_sub = ModeManager.create_subscription(
        manager, object, "/uav_1/status", lambda _msg: None, 1
    )
    shared_pub = ModeManager.create_publisher(manager, object, "/shared/debug", 1)

    assert isinstance(peer_sub, ManagedSubscription)
    assert isinstance(shared_pub, ManagedPublisher)

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


def test_run_active_mode_calls_disconnect_handler_for_missing_required_peers():
    _xfail_if_missing_peer_features(
        "mode_peer_vehicle_names",
        "mode_on_disconnect",
        "manager_disconnect_runtime",
    )

    class PeerAwareMode(Mode):
        mission_target = "payload"
        peer_vehicle_names = ("uav_1", "uav_2")

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)
            self.update_calls: list[float] = []
            self.disconnect_calls: list[tuple[float, tuple[str, ...]]] = []
            self.status_checks = 0

        def on_disconnect(self, time_delta: float, peers: tuple[str, ...]) -> None:
            self.disconnect_calls.append((time_delta, peers))

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

    assert mode.update_calls == []
    assert mode.disconnect_calls == [(0.25, ("uav_1",))]
    assert mode.status_checks == 1
    assert handled_states == ["continue"]


def test_peer_client_wrapper_rebinds_on_connection_changes(monkeypatch):
    _xfail_if_missing_peer_features(
        "mode_peer_vehicle_names",
        "manager_create_client",
    )

    class PeerAwareMode(Mode):
        mission_target = "payload"
        peer_vehicle_names = ("uav_1",)

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)

        def on_disconnect(self, time_delta: float, peers: tuple[str, ...]) -> None:
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

    client = ModeManager.create_client(manager, object, "/uav_1/camera")

    assert isinstance(client, ManagedClient)
    assert client.get_underlying() is None

    manager._peer_connected["uav_1"] = True
    ModeManager._handle_peer_connection_change(manager, "uav_1", connected=True)

    assert create_calls == ["/uav_1/camera"]
    assert client.get_underlying() is not None

    manager._peer_connected["uav_1"] = False
    ModeManager._handle_peer_connection_change(manager, "uav_1", connected=False)

    assert client.get_underlying() is None
    assert len(destroy_calls) == 1


def test_peer_timer_publishes_heartbeat_and_marks_stale_connections():
    _xfail_if_missing_peer_features("manager_disconnect_runtime")

    manager = _make_mode_manager()
    published: list[object] = []
    connection_events: list[tuple[str, bool]] = []

    manager._peer_heartbeat_publisher = SimpleNamespace(
        publish=lambda msg: published.append(msg)
    )
    manager._mission_peer_names = ("uav_1",)
    manager._peer_connected = {"uav_1": True}
    manager._peer_last_seen = {"uav_1": 0.0}
    manager.peer_stale_timeout_s = 0.5
    manager.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)
    )
    manager._handle_peer_connection_change = (
        lambda peer_name, *, connected: connection_events.append(
            (peer_name, connected)
        )
    )

    ModeManager._peer_timer_callback(manager)

    assert len(published) == 1
    assert manager._peer_connected["uav_1"] is False
    assert connection_events == [("uav_1", False)]


def test_peer_entity_usage_instrumentation_matches_declared_peers():
    _xfail_if_missing_peer_features(
        "mode_peer_vehicle_names",
        "mode_on_disconnect",
    )

    class InstrumentedPeerMode(Mode):
        mission_target = "payload"
        peer_vehicle_names = ("payload_1", "uav_2")

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)
            self.node.create_subscription(object, "/uav_2/status", lambda _msg: None, 1)

        def on_disconnect(self, time_delta: float, peers: tuple[str, ...]) -> None:
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
