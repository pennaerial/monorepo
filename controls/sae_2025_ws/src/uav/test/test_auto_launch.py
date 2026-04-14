from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import types

import pytest

if "std_msgs" not in sys.modules:
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Empty = type("Empty", (), {})
    std_msgs.msg = std_msgs_msg
    sys.modules.update({"std_msgs": std_msgs, "std_msgs.msg": std_msgs_msg})

if "ament_index_python" not in sys.modules:
    ament_index_python = types.ModuleType("ament_index_python")
    ament_index_packages = types.ModuleType("ament_index_python.packages")
    ament_index_packages.get_package_share_directory = lambda _name: str(
        Path(__file__).resolve().parents[1]
    )
    ament_index_python.packages = ament_index_packages
    sys.modules.update(
        {
            "ament_index_python": ament_index_python,
            "ament_index_python.packages": ament_index_packages,
        }
    )

launch_module = sys.modules.get("launch")
if launch_module is None:
    launch_module = types.ModuleType("launch")
    sys.modules["launch"] = launch_module
if not hasattr(launch_module, "LaunchDescription"):
    launch_module.LaunchDescription = type("LaunchDescription", (), {})

launch_actions = sys.modules.get("launch.actions")
if launch_actions is None:
    launch_actions = types.ModuleType("launch.actions")
    sys.modules["launch.actions"] = launch_actions
for name in ("DeclareLaunchArgument", "IncludeLaunchDescription", "OpaqueFunction"):
    if not hasattr(launch_actions, name):
        setattr(launch_actions, name, type(name, (), {}))

launch_sources = sys.modules.get("launch.launch_description_sources")
if launch_sources is None:
    launch_sources = types.ModuleType("launch.launch_description_sources")
    sys.modules["launch.launch_description_sources"] = launch_sources
if not hasattr(launch_sources, "PythonLaunchDescriptionSource"):
    launch_sources.PythonLaunchDescriptionSource = type(
        "PythonLaunchDescriptionSource", (), {}
    )

launch_logging = sys.modules.get("launch.logging")
if launch_logging is None:
    launch_logging = types.ModuleType("launch.logging")
    sys.modules["launch.logging"] = launch_logging
if not hasattr(launch_logging, "get_logger"):
    launch_logging.get_logger = lambda *_args, **_kwargs: SimpleNamespace(
        warning=lambda *_a, **_k: None,
        warn=lambda *_a, **_k: None,
        info=lambda *_a, **_k: None,
    )

launch_substitutions = sys.modules.get("launch.substitutions")
if launch_substitutions is None:
    launch_substitutions = types.ModuleType("launch.substitutions")
    sys.modules["launch.substitutions"] = launch_substitutions
if not hasattr(launch_substitutions, "LaunchConfiguration"):
    launch_substitutions.LaunchConfiguration = type("LaunchConfiguration", (), {})

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
    clock_mod.Clock = type("Clock", (), {})
    parameter_mod.Parameter = type("Parameter", (), {})
    validate_namespace_mod.validate_namespace = lambda namespace: None
    validate_node_name_mod.validate_node_name = lambda node_name: None
    qos_mod.QoSProfile = type("QoSProfile", (), {})
    qos_mod.QoSReliabilityPolicy = type("QoSReliabilityPolicy", (), {})
    qos_mod.QoSHistoryPolicy = type("QoSHistoryPolicy", (), {})
    qos_mod.QoSDurabilityPolicy = type("QoSDurabilityPolicy", (), {})
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

from uav.runtime.ModeManager import ModeManager  # noqa: E402

try:
    import uav.runtime.uav_mission as uav_mission_module
    import uav.runtime.UAVModeManager as uav_manager_module
    from uav.runtime.UAVModeManager import UAVModeManager
except ModuleNotFoundError as exc:
    if exc.name != "px4_msgs":
        raise
    uav_mission_module = None
    uav_manager_module = None
    UAVModeManager = None

try:
    import uav.runtime.payload_mission as payload_mission_module
    import uav.runtime.PayloadModeManager as payload_manager_module
except ModuleNotFoundError as exc:
    if exc.name != "payload_interfaces":
        raise
    payload_mission_module = None
    payload_manager_module = None


class _FakeLogger:
    def __init__(self) -> None:
        self.messages = []

    def info(self, message: str) -> None:
        self.messages.append(("info", message))

    def warn(self, message: str) -> None:
        self.messages.append(("warn", message))

    def error(self, message: str) -> None:
        self.messages.append(("error", message))


class _FakeTimer:
    def __init__(self, callback) -> None:
        self.callback = callback
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True


def _make_mode_manager(*, ready: bool) -> ModeManager:
    manager = object.__new__(ModeManager)
    manager.vehicle = None
    manager.modes = {}
    manager.transitions = {}
    manager.active_mode = None
    manager.last_update_time = 0.0
    manager._vision_clients = {}
    manager.timer = None
    manager.auto_launch = True
    manager._auto_launch_timer = _FakeTimer(None)
    manager._ready = ready
    manager._logger = _FakeLogger()
    manager.spin_once = lambda: None
    manager._auto_launch_ready = lambda: manager._ready
    manager.create_timer = lambda period, callback: _FakeTimer(callback)
    manager.get_logger = lambda: manager._logger
    return manager


class _FakeParameter:
    def __init__(self, value) -> None:
        self.value = value


class _FakeServiceClient:
    def __init__(self, *, ready: bool) -> None:
        self.ready = ready
        self.calls = []

    def wait_for_service(self, timeout_sec: float) -> bool:
        self.calls.append(timeout_sec)
        return self.ready


def _stub_mode_manager_init(
    self,
    node_name: str,
    *,
    vehicle_name: str = "",
    auto_launch: bool = True,
    peer_heartbeat_hz: float = 10.0,
    peer_stale_timeout_s: float = 0.5,
) -> None:
    self.vehicle = None
    self.modes = {}
    self.transitions = {}
    self.active_mode = None
    self.last_update_time = 0.0
    self._vision_clients = {}
    self.timer = None
    self.auto_launch = bool(auto_launch)
    self._runtime_vehicle_name = str(vehicle_name).strip().strip("/")
    self.peer_heartbeat_hz = float(peer_heartbeat_hz)
    self.peer_stale_timeout_s = float(peer_stale_timeout_s)
    self._managed_entity_context = None
    self._managed_entities = {}
    self._peer_heartbeat_publisher = None
    self._peer_timer = None
    self._peer_heartbeat_subscriptions = {}
    self._peer_connected = {}
    self._peer_last_seen = {}
    self._mission_peer_names = ()
    self._logger = _FakeLogger()
    self.create_timer = lambda period, callback: _FakeTimer(callback)
    self.create_service = lambda *args, **kwargs: object()
    self.configure_peer_vehicle_names = lambda peer_names: setattr(
        self, "_mission_peer_names", tuple(peer_names)
    )
    self.get_logger = lambda: self._logger
    self.start_mission_service = object()
    self._auto_launch_timer = None
    if self.auto_launch:
        self._auto_launch_timer = self.create_timer(0.1, self._maybe_auto_launch)


def _load_main_launch_module():
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "main.launch.py"
    spec = importlib.util.spec_from_file_location("uav_main_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_start_mission_is_idempotent():
    manager = _make_mode_manager(ready=False)

    assert ModeManager.start_mission(manager) is True
    first_timer = manager.timer

    assert first_timer is not None
    assert manager._auto_launch_timer is None
    assert ModeManager.start_mission(manager) is False
    assert manager.timer is first_timer


def test_auto_launch_starts_once_when_ready():
    manager = _make_mode_manager(ready=False)
    auto_launch_timer = manager._auto_launch_timer

    ModeManager._maybe_auto_launch(manager)
    assert manager.timer is None
    assert auto_launch_timer is manager._auto_launch_timer

    manager._ready = True
    ModeManager._maybe_auto_launch(manager)

    assert manager.timer is not None
    assert manager._auto_launch_timer is None
    assert auto_launch_timer.cancelled is True


def test_uav_auto_launch_readiness_requires_all_inputs():
    if UAVModeManager is None:
        pytest.skip("px4_msgs Python module is not available in this test environment")

    manager = object.__new__(UAVModeManager)
    manager.vehicle = SimpleNamespace(
        vehicle_status=None,
        vehicle_attitude=None,
        yaw=None,
        local_position=None,
        global_position=None,
        flight_check=False,
    )

    assert UAVModeManager._auto_launch_ready(manager) is False

    manager.vehicle.vehicle_status = object()
    manager.vehicle.vehicle_attitude = object()
    manager.vehicle.yaw = 0.0
    manager.vehicle.local_position = object()
    manager.vehicle.global_position = object()

    assert UAVModeManager._auto_launch_ready(manager) is False

    manager.vehicle.flight_check = True

    assert UAVModeManager._auto_launch_ready(manager) is True


def test_payload_manager_waits_for_service_when_auto_launch_false(monkeypatch):
    if payload_mission_module is None:
        pytest.skip(
            "payload_interfaces Python module is not available in this test environment"
        )

    monkeypatch.setattr(ModeManager, "__init__", _stub_mode_manager_init)
    monkeypatch.setattr(ModeManager, "setup_vision", lambda self, _: None)
    monkeypatch.setattr(ModeManager, "setup_modes", lambda self, _: None)
    monkeypatch.setattr(
        payload_manager_module,
        "Payload",
        lambda node, name: SimpleNamespace(
            name=name,
            timed_drive_client=_FakeServiceClient(ready=False),
        ),
    )

    manager = payload_manager_module.PayloadModeManager(
        mission_spec=SimpleNamespace(is_payload=True, vision_nodes=[]),
        vehicle_name="payload_0",
        auto_launch=False,
    )

    assert manager.timer is None
    assert manager._auto_launch_timer is None

    response = SimpleNamespace(success=None, message=None)
    ModeManager._start_mission_callback(manager, None, response)

    assert manager.timer is not None
    assert response.success is True
    assert response.message == "Starting Mission!"


def test_payload_auto_launch_readiness_requires_timed_drive_service():
    if payload_mission_module is None:
        pytest.skip(
            "payload_interfaces Python module is not available in this test environment"
        )

    manager = object.__new__(payload_manager_module.PayloadModeManager)
    manager.vehicle = SimpleNamespace(
        timed_drive_client=_FakeServiceClient(ready=False),
    )

    assert (
        payload_manager_module.PayloadModeManager._auto_launch_ready(manager) is False
    )

    manager.vehicle.timed_drive_client.ready = True

    assert payload_manager_module.PayloadModeManager._auto_launch_ready(manager) is True
    assert manager.vehicle.timed_drive_client.calls == [0.0, 0.0]


def test_payload_manager_auto_launches_when_service_is_ready(monkeypatch):
    if payload_mission_module is None:
        pytest.skip(
            "payload_interfaces Python module is not available in this test environment"
        )

    monkeypatch.setattr(ModeManager, "__init__", _stub_mode_manager_init)
    monkeypatch.setattr(ModeManager, "setup_vision", lambda self, _: None)
    monkeypatch.setattr(ModeManager, "setup_modes", lambda self, _: None)
    timed_drive_client = _FakeServiceClient(ready=False)
    monkeypatch.setattr(
        payload_manager_module,
        "Payload",
        lambda node, name: SimpleNamespace(
            name=name,
            timed_drive_client=timed_drive_client,
        ),
    )

    manager = payload_manager_module.PayloadModeManager(
        mission_spec=SimpleNamespace(is_payload=True, vision_nodes=[]),
        vehicle_name="payload_0",
        auto_launch=True,
    )

    assert manager.timer is None
    assert manager._auto_launch_timer is not None

    ModeManager._maybe_auto_launch(manager)

    assert manager.timer is None
    assert manager._auto_launch_timer is not None
    assert timed_drive_client.calls == [0.0]

    timed_drive_client.ready = True

    ModeManager._maybe_auto_launch(manager)

    assert manager.timer is not None
    assert manager._auto_launch_timer is None
    assert timed_drive_client.calls == [0.0, 0.0]


def test_uav_manager_waits_for_service_when_auto_launch_false(monkeypatch):
    if uav_mission_module is None:
        pytest.skip("px4_msgs Python module is not available in this test environment")

    monkeypatch.setattr(ModeManager, "__init__", _stub_mode_manager_init)
    monkeypatch.setattr(ModeManager, "setup_vision", lambda self, _: None)
    monkeypatch.setattr(ModeManager, "setup_modes", lambda self, _: None)
    monkeypatch.setattr(
        uav_manager_module,
        "Multicopter",
        lambda node, DEBUG=False, camera_offsets=None, vehicle_name="uav": (
            SimpleNamespace(
                vehicle_status=None,
                vehicle_attitude=None,
                yaw=None,
                local_position=None,
                global_position=None,
                flight_check=False,
                failsafe=False,
                emergency_landing=False,
                nav_state=None,
                arm_state=None,
            )
        ),
    )
    monkeypatch.setattr(uav_manager_module.AirframeClass, "parse", lambda value: value)

    manager = uav_manager_module.UAVModeManager(
        mission_spec=SimpleNamespace(is_uav=True, vision_nodes=[]),
        auto_launch=False,
        vehicle_class=uav_manager_module.AirframeClass.MULTICOPTER,
    )

    assert manager.timer is None
    assert manager._auto_launch_timer is None

    response = SimpleNamespace(success=None, message=None)
    ModeManager._start_mission_callback(manager, None, response)

    assert manager.timer is not None
    assert response.success is True
    assert response.message == "Starting Mission!"


def test_uav_manager_auto_launches_only_after_readiness(monkeypatch):
    if uav_mission_module is None:
        pytest.skip("px4_msgs Python module is not available in this test environment")

    monkeypatch.setattr(ModeManager, "__init__", _stub_mode_manager_init)
    monkeypatch.setattr(ModeManager, "setup_vision", lambda self, _: None)
    monkeypatch.setattr(ModeManager, "setup_modes", lambda self, _: None)
    monkeypatch.setattr(
        uav_manager_module,
        "Multicopter",
        lambda node, DEBUG=False, camera_offsets=None, vehicle_name="uav": (
            SimpleNamespace(
                vehicle_status=None,
                vehicle_attitude=None,
                yaw=None,
                local_position=None,
                global_position=None,
                flight_check=False,
                failsafe=False,
                emergency_landing=False,
                nav_state=None,
                arm_state=None,
            )
        ),
    )
    monkeypatch.setattr(uav_manager_module.AirframeClass, "parse", lambda value: value)

    manager = uav_manager_module.UAVModeManager(
        mission_spec=SimpleNamespace(is_uav=True, vision_nodes=[]),
        auto_launch=True,
        vehicle_class=uav_manager_module.AirframeClass.MULTICOPTER,
    )

    assert manager.timer is None
    assert manager._auto_launch_timer is not None

    ModeManager._maybe_auto_launch(manager)
    assert manager.timer is None

    manager.vehicle.vehicle_status = object()
    manager.vehicle.vehicle_attitude = object()
    manager.vehicle.yaw = 0.0
    manager.vehicle.local_position = object()
    manager.vehicle.global_position = object()
    manager.vehicle.flight_check = True

    ModeManager._maybe_auto_launch(manager)

    assert manager.timer is not None
    assert manager._auto_launch_timer is None


@pytest.mark.parametrize("auto_launch", [True, False])
def test_uav_bootstrap_forwards_auto_launch(monkeypatch, auto_launch):
    if uav_mission_module is None:
        pytest.skip("px4_msgs Python module is not available in this test environment")

    fake_spec = SimpleNamespace(is_uav=True)
    bootstrap = object.__new__(uav_mission_module.UAVMissionBootstrap)
    parameters = {
        "mode_map": "/tmp/test_uav_mission.yaml",
        "auto_launch": auto_launch,
        "debug": False,
        "servo_only": False,
        "vehicle_name": "uav_2",
        "vehicle_class": "MULTICOPTER",
        "camera_mount_offsets": [0.0, 0.0, 0.0],
        "peer_heartbeat_hz": 8.0,
        "peer_stale_timeout_s": 1.25,
    }
    bootstrap.get_parameter = lambda name: _FakeParameter(parameters[name])

    monkeypatch.setattr(uav_mission_module.MissionSpec, "load", lambda _: fake_spec)
    monkeypatch.setattr(uav_mission_module.AirframeClass, "parse", lambda value: value)

    manager_kwargs = bootstrap.manager_kwargs()

    assert manager_kwargs["auto_launch"] is auto_launch
    assert manager_kwargs["vehicle_name"] == "uav_2"
    assert manager_kwargs["peer_heartbeat_hz"] == 8.0
    assert manager_kwargs["peer_stale_timeout_s"] == 1.25


@pytest.mark.parametrize("auto_launch", [True, False])
def test_payload_bootstrap_forwards_auto_launch(monkeypatch, auto_launch):
    if payload_mission_module is None:
        pytest.skip(
            "payload_interfaces Python module is not available in this test environment"
        )

    fake_spec = SimpleNamespace(is_payload=True)
    bootstrap = object.__new__(payload_mission_module.PayloadMissionBootstrap)
    parameters = {
        "mode_map": "/tmp/test_payload_mission.yaml",
        "auto_launch": auto_launch,
        "vehicle_name": "payload_0",
        "peer_heartbeat_hz": 12.0,
        "peer_stale_timeout_s": 0.75,
    }
    bootstrap.get_parameter = lambda name: _FakeParameter(parameters[name])

    monkeypatch.setattr(payload_mission_module.MissionSpec, "load", lambda _: fake_spec)

    manager_kwargs = bootstrap.manager_kwargs()

    assert manager_kwargs["auto_launch"] is auto_launch
    assert manager_kwargs["peer_heartbeat_hz"] == 12.0
    assert manager_kwargs["peer_stale_timeout_s"] == 0.75


def test_launch_auto_launch_yaml_requires_bool():
    module = _load_main_launch_module()

    assert module._yaml_bool_value(True, name="auto_launch", default=False) is True
    assert module._yaml_bool_value(None, name="auto_launch", default=True) is True

    with pytest.raises(ValueError, match="auto_launch"):
        module._yaml_bool_value("false", name="auto_launch", default=True)
