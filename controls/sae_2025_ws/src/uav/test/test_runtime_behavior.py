from __future__ import annotations

import sys
import types
from types import SimpleNamespace

import pytest


def _placeholder(name: str):
    return type(name, (), {})


def _install_ros_test_doubles() -> None:
    if "rclpy" not in sys.modules:
        rclpy = types.ModuleType("rclpy")
        rclpy.ok = lambda: True

        node_mod = types.ModuleType("rclpy.node")

        class Node:
            pass

        node_mod.Node = Node

        executors_mod = types.ModuleType("rclpy.executors")

        class ExternalShutdownException(Exception):
            pass

        executors_mod.ExternalShutdownException = ExternalShutdownException

        clock_mod = types.ModuleType("rclpy.clock")
        clock_mod.Clock = _placeholder("Clock")

        parameter_mod = types.ModuleType("rclpy.parameter")
        parameter_mod.Parameter = _placeholder("Parameter")

        validate_namespace_mod = types.ModuleType("rclpy.validate_namespace")
        validate_namespace_mod.validate_namespace = lambda namespace: None

        validate_node_name_mod = types.ModuleType("rclpy.validate_node_name")
        validate_node_name_mod.validate_node_name = lambda node_name: None

        qos_mod = types.ModuleType("rclpy.qos")
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

    if "px4_msgs" not in sys.modules:
        px4_msgs = types.ModuleType("px4_msgs")
        px4_msgs_msg = types.ModuleType("px4_msgs.msg")

        class VehicleStatus:
            NAVIGATION_STATE_AUTO_LOITER = 1
            NAVIGATION_STATE_AUTO_LAND = 2
            ARMING_STATE_ARMED = 3

        class VtolVehicleStatus:
            VEHICLE_VTOL_STATE_MC = 1
            VEHICLE_VTOL_STATE_FW = 2
            VEHICLE_VTOL_STATE_TRANSITION_TO_FW = 3
            VEHICLE_VTOL_STATE_TRANSITION_TO_MC = 4

        px4_msgs_msg.VehicleStatus = VehicleStatus
        px4_msgs_msg.OffboardControlMode = _placeholder("OffboardControlMode")
        px4_msgs_msg.TrajectorySetpoint = _placeholder("TrajectorySetpoint")
        px4_msgs_msg.VehicleCommand = _placeholder("VehicleCommand")
        px4_msgs_msg.VehicleAttitude = _placeholder("VehicleAttitude")
        px4_msgs_msg.VehicleGlobalPosition = _placeholder("VehicleGlobalPosition")
        px4_msgs_msg.VehicleLocalPosition = _placeholder("VehicleLocalPosition")
        px4_msgs_msg.SensorGps = _placeholder("SensorGps")
        px4_msgs_msg.VtolVehicleStatus = VtolVehicleStatus
        px4_msgs.msg = px4_msgs_msg
        sys.modules.update({"px4_msgs": px4_msgs, "px4_msgs.msg": px4_msgs_msg})

    if "payload_interfaces" not in sys.modules:
        payload_interfaces = types.ModuleType("payload_interfaces")
        payload_interfaces_msg = types.ModuleType("payload_interfaces.msg")
        payload_interfaces_srv = types.ModuleType("payload_interfaces.srv")
        payload_interfaces_msg.DriveCommand = _placeholder("DriveCommand")
        payload_interfaces_msg.ServoCommand = _placeholder("ServoCommand")

        class TimedDrive:
            Request = _placeholder("Request")

        class DeadReckon:
            Request = _placeholder("Request")

        payload_interfaces_srv.TimedDrive = TimedDrive
        payload_interfaces_srv.DeadReckon = DeadReckon
        payload_interfaces.msg = payload_interfaces_msg
        payload_interfaces.srv = payload_interfaces_srv
        sys.modules.update(
            {
                "payload_interfaces": payload_interfaces,
                "payload_interfaces.msg": payload_interfaces_msg,
                "payload_interfaces.srv": payload_interfaces_srv,
            }
        )


_install_ros_test_doubles()

from uav.modes.Mode import Mode  # noqa: E402
from uav.runtime.ModeManager import ModeManager  # noqa: E402
import uav.runtime.ModeManager as mode_manager_module  # noqa: E402
import uav.runtime.uav_mission as uav_mission_module  # noqa: E402
from uav.runtime.UAVModeManager import UAVModeManager  # noqa: E402
import uav.runtime.payload_mission as payload_mission_module  # noqa: E402
import uav.runtime.PayloadModeManager as payload_manager_module  # noqa: E402


class _ExpectedVehicle:
    pass


class _OtherVehicle:
    pass


class _FakeLogger:
    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def info(self, message: str) -> None:
        self.messages.append(("info", message))

    def warn(self, message: str) -> None:
        self.messages.append(("warn", message))

    def warning(self, message: str) -> None:
        self.messages.append(("warning", message))

    def error(self, message: str) -> None:
        self.messages.append(("error", message))


class _FakeTimer:
    def __init__(self, callback=None) -> None:
        self.callback = callback
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True


class _TrackingMode:
    def __init__(
        self, label: str, *, status: str = "continue", raises: Exception | None = None
    ) -> None:
        self.label = label
        self.status = status
        self.raises = raises
        self.activated = 0
        self.deactivated = 0
        self.updates: list[float] = []

    def activate(self) -> None:
        self.activated += 1

    def deactivate(self) -> None:
        self.deactivated += 1

    def update(self, time_delta: float) -> None:
        self.updates.append(time_delta)
        if self.raises is not None:
            raise self.raises

    def check_status(self) -> str:
        return self.status


def _make_mode_manager(*, vehicle=None, auto_launch: bool = False) -> ModeManager:
    manager = object.__new__(ModeManager)
    manager.vehicle = vehicle
    manager.modes = {}
    manager.transitions = {}
    manager.active_mode = None
    manager.last_update_time = 0.0
    manager._vision_clients = {}
    manager.timer = None
    manager.auto_launch = auto_launch
    manager._auto_launch_timer = None
    manager._logger = _FakeLogger()
    manager.destroyed = False
    manager.stopped = False
    manager.get_logger = lambda: manager._logger
    manager.create_timer = lambda period, callback: _FakeTimer(callback)
    manager.create_client = lambda *args, **kwargs: None
    manager.destroy_client = lambda client: None
    manager.destroy_node = lambda: setattr(manager, "destroyed", True)
    manager._stop_vehicle = lambda: setattr(manager, "stopped", True)
    return manager


def _stub_mode_manager_init(self, node_name: str, *, auto_launch: bool = True) -> None:
    self.vehicle = None
    self.modes = {}
    self.transitions = {}
    self.active_mode = None
    self.last_update_time = 0.0
    self._vision_clients = {}
    self.timer = None
    self.auto_launch = bool(auto_launch)
    self._auto_launch_timer = None
    self._logger = _FakeLogger()
    self.create_timer = lambda period, callback: _FakeTimer(callback)
    self.create_service = lambda *args, **kwargs: object()
    self.get_logger = lambda: self._logger
    self.destroy_node = lambda: None


def _make_bootstrap(module_cls, params: dict[str, object]):
    bootstrap = object.__new__(module_cls)
    bootstrap.get_parameter = lambda name: SimpleNamespace(value=params[name])
    return bootstrap


def _fake_mission_spec(*, target: str, is_uav: bool, is_payload: bool, vision_nodes=()):
    return SimpleNamespace(
        target=target,
        is_uav=is_uav,
        is_payload=is_payload,
        vision_nodes=tuple(vision_nodes),
    )


def _require_runtime_support() -> None:
    if Mode is None or ModeManager is None or mode_manager_module is None:
        pytest.skip(
            "ROS runtime Python modules are not available in this test environment"
        )


def test_initialize_mode_happy_path(monkeypatch):
    _require_runtime_support()

    class FakeMode(Mode):
        mission_target = "uav"

        def __init__(
            self,
            node,
            vehicle: _ExpectedVehicle,
            required: int,
            optional: str = "default",
        ) -> None:
            super().__init__(node, vehicle)
            self.required = required
            self.optional = optional

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    manager = _make_mode_manager(vehicle=_ExpectedVehicle())
    monkeypatch.setattr(
        mode_manager_module,
        "load_mode_class",
        lambda _path: FakeMode,
    )

    mode = ModeManager.initialize_mode(
        manager, "fake.module.FakeMode", {"required": 7, "optional": "value"}
    )

    assert isinstance(mode, FakeMode)
    assert mode.node is manager
    assert mode.vehicle is manager.vehicle
    assert mode.required == 7
    assert mode.optional == "value"


def test_initialize_mode_missing_required_parameter(monkeypatch):
    _require_runtime_support()

    class FakeMode(Mode):
        mission_target = "uav"

        def __init__(self, node, vehicle: _ExpectedVehicle, required: int) -> None:
            super().__init__(node, vehicle)
            self.required = required

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    manager = _make_mode_manager(vehicle=_ExpectedVehicle())
    monkeypatch.setattr(
        mode_manager_module,
        "load_mode_class",
        lambda _path: FakeMode,
    )

    with pytest.raises(ValueError, match="Missing required parameter 'required'"):
        ModeManager.initialize_mode(manager, "fake.module.FakeMode", {})


def test_initialize_mode_rejects_unexpected_parameter(monkeypatch):
    _require_runtime_support()

    class FakeMode(Mode):
        mission_target = "uav"

        def __init__(self, node, vehicle: _ExpectedVehicle, required: int) -> None:
            super().__init__(node, vehicle)
            self.required = required

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    manager = _make_mode_manager(vehicle=_ExpectedVehicle())
    monkeypatch.setattr(
        mode_manager_module,
        "load_mode_class",
        lambda _path: FakeMode,
    )

    with pytest.raises(ValueError, match="unexpected parameter"):
        ModeManager.initialize_mode(
            manager, "fake.module.FakeMode", {"required": 7, "extra": 1}
        )


def test_initialize_mode_rejects_vehicle_type_mismatch(monkeypatch):
    _require_runtime_support()

    class FakeMode(Mode):
        mission_target = "uav"

        def __init__(self, node, vehicle: _ExpectedVehicle, required: int) -> None:
            super().__init__(node, vehicle)
            self.required = required

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    manager = _make_mode_manager(vehicle=_OtherVehicle())
    monkeypatch.setattr(
        mode_manager_module,
        "load_mode_class",
        lambda _path: FakeMode,
    )

    with pytest.raises(TypeError, match="expects vehicle type"):
        ModeManager.initialize_mode(manager, "fake.module.FakeMode", {"required": 7})


def test_setup_vision_deduplicates_clients(monkeypatch):
    _require_runtime_support()

    class FakeVisionNode:
        srv = SimpleNamespace(Response=type("Response", (), {}))

    vision_module = SimpleNamespace(FakeVisionNode=FakeVisionNode)
    client = SimpleNamespace(wait_for_service=lambda timeout_sec: True)
    manager = _make_mode_manager(
        vehicle=SimpleNamespace(
            has_camera=True,
            vision_service_name=lambda vision_class: f"/vision/{vision_class.__name__}",
        )
    )
    created_clients: list[tuple[object, str]] = []

    monkeypatch.setattr(
        mode_manager_module.importlib,
        "import_module",
        lambda _path: vision_module,
    )
    monkeypatch.setattr(
        manager,
        "create_client",
        lambda service, service_name: (
            created_clients.append((service, service_name)) or client
        ),
    )

    ModeManager.setup_vision(manager, ["FakeVisionNode", "FakeVisionNode"])

    assert list(manager.vision_clients) == ["FakeVisionNode"]
    assert created_clients == [(FakeVisionNode.srv, "/vision/FakeVisionNode")]


def test_setup_vision_rejects_vehicle_without_camera():
    _require_runtime_support()

    manager = _make_mode_manager(vehicle=SimpleNamespace(has_camera=False))

    with pytest.raises(
        ValueError, match="Vision nodes require an active vehicle camera contract"
    ):
        ModeManager.setup_vision(manager, ["FakeVisionNode"])


def test_switch_mode_deactivates_previous_mode_and_activates_new_mode():
    _require_runtime_support()

    start_mode = _TrackingMode("start")
    next_mode = _TrackingMode("next")
    manager = _make_mode_manager()
    manager.modes = {"start": start_mode, "next": next_mode}
    manager.active_mode = "start"

    ModeManager.switch_mode(manager, "next")

    assert manager.active_mode == "next"
    assert start_mode.deactivated == 1
    assert next_mode.activated == 1


def test_switch_mode_missing_mode_logs_error_and_keeps_current_mode():
    _require_runtime_support()

    start_mode = _TrackingMode("start")
    manager = _make_mode_manager()
    manager.modes = {"start": start_mode}
    manager.active_mode = "start"

    ModeManager.switch_mode(manager, "missing")

    assert manager.active_mode == "start"
    assert any(
        level == "error" and "Mode missing not found." in msg
        for level, msg in manager._logger.messages
    )


def test_run_active_mode_transitions_on_status():
    _require_runtime_support()

    start_mode = _TrackingMode("start", status="complete")
    next_mode = _TrackingMode("next")
    manager = _make_mode_manager()
    manager.modes = {"start": start_mode, "next": next_mode}
    manager.transitions = {"start": {"complete": "next"}}
    manager.active_mode = "start"

    ModeManager._run_active_mode(manager, 2.5)

    assert start_mode.updates == [2.5]
    assert manager.last_update_time == 2.5
    assert manager.active_mode == "next"
    assert start_mode.deactivated == 1
    assert next_mode.activated == 1


def test_run_active_mode_handles_update_error():
    _require_runtime_support()

    start_mode = _TrackingMode("start", raises=RuntimeError("boom"))
    manager = _make_mode_manager()
    manager.modes = {"start": start_mode}
    manager.active_mode = "start"
    handled: list[str] = []
    manager.handle_mode_state = lambda state: handled.append(state)

    ModeManager._run_active_mode(manager, 9.0)

    assert start_mode.updates == [9.0]
    assert handled == ["error"]


def test_handle_mode_state_requires_exact_transition_label():
    _require_runtime_support()

    start_mode = _TrackingMode("start", status="complete")
    next_mode = _TrackingMode("next")
    manager = _make_mode_manager()
    manager.modes = {"start": start_mode, "next": next_mode}
    manager.transitions = {"start": {"complete": "next"}}
    manager.active_mode = "start"

    ModeManager.handle_mode_state(manager, "complete")

    assert manager.active_mode == "next"
    assert start_mode.deactivated == 1
    assert next_mode.activated == 1

    manager = _make_mode_manager()
    manager.modes = {"start": _TrackingMode("start"), "next": _TrackingMode("next")}
    manager.transitions = {"start": {"complete": "next"}}
    manager.active_mode = "start"

    with pytest.raises(KeyError):
        ModeManager.handle_mode_state(manager, "Complete")
    assert manager.active_mode == "start"


def test_start_mission_callback_reports_already_started():
    _require_runtime_support()

    manager = _make_mode_manager()
    manager.timer = object()
    response = SimpleNamespace(success=None, message=None)

    ModeManager._start_mission_callback(manager, None, response)

    assert response.success is True
    assert response.message == "Mission already started."


def test_uav_mode_manager_rejects_payload_spec(monkeypatch):
    _require_runtime_support()

    if UAVModeManager is None:
        pytest.skip("px4_msgs Python module is not available in this test environment")

    monkeypatch.setattr(ModeManager, "__init__", _stub_mode_manager_init)

    with pytest.raises(ValueError, match="requires a UAV mission spec"):
        UAVModeManager(
            mission_spec=_fake_mission_spec(
                target="payload", is_uav=False, is_payload=True
            )
        )


def test_payload_mode_manager_rejects_uav_spec(monkeypatch):
    _require_runtime_support()

    if payload_mission_module is None:
        pytest.skip(
            "payload_interfaces Python module is not available in this test environment"
        )

    monkeypatch.setattr(ModeManager, "__init__", _stub_mode_manager_init)

    with pytest.raises(ValueError, match="requires a payload mission spec"):
        payload_manager_module.PayloadModeManager(
            mission_spec=_fake_mission_spec(
                target="uav", is_uav=True, is_payload=False
            ),
            vehicle_name="payload_0",
        )


def test_uav_bootstrap_bool_parameter_and_manager_validation(monkeypatch):
    _require_runtime_support()

    if uav_mission_module is None:
        pytest.skip("px4_msgs Python module is not available in this test environment")

    bootstrap = _make_bootstrap(
        uav_mission_module.UAVMissionBootstrap, {"auto_launch": "true"}
    )

    with pytest.raises(ValueError, match="requires boolean parameter 'auto_launch'"):
        uav_mission_module.UAVMissionBootstrap._bool_parameter(bootstrap, "auto_launch")

    monkeypatch.setattr(
        uav_mission_module.MissionSpec,
        "load",
        lambda _path: _fake_mission_spec(
            target="payload", is_uav=False, is_payload=True
        ),
    )
    bootstrap = _make_bootstrap(
        uav_mission_module.UAVMissionBootstrap,
        {
            "mode_map": "mission.yaml",
            "auto_launch": True,
            "debug": False,
            "servo_only": False,
            "vehicle_name": "uav_5",
            "vehicle_class": "MULTICOPTER",
            "camera_mount_offsets": [0.0, 0.0, 0.0],
        },
    )

    with pytest.raises(ValueError, match="requires a UAV mission spec"):
        uav_mission_module.UAVMissionBootstrap.manager_kwargs(bootstrap)


def test_payload_bootstrap_bool_parameter_and_manager_validation(monkeypatch):
    _require_runtime_support()

    if payload_mission_module is None:
        pytest.skip(
            "payload_interfaces Python module is not available in this test environment"
        )

    bootstrap = _make_bootstrap(
        payload_mission_module.PayloadMissionBootstrap, {"auto_launch": 1}
    )

    with pytest.raises(ValueError, match="requires boolean parameter 'auto_launch'"):
        payload_mission_module.PayloadMissionBootstrap._bool_parameter(
            bootstrap, "auto_launch"
        )

    monkeypatch.setattr(
        payload_mission_module.MissionSpec,
        "load",
        lambda _path: _fake_mission_spec(target="uav", is_uav=True, is_payload=False),
    )
    bootstrap = _make_bootstrap(
        payload_mission_module.PayloadMissionBootstrap,
        {
            "mode_map": "mission.yaml",
            "auto_launch": True,
            "vehicle_name": "payload_0",
        },
    )

    with pytest.raises(ValueError, match="requires a payload mission spec"):
        payload_mission_module.PayloadMissionBootstrap.manager_kwargs(bootstrap)


def test_mode_manager_stop_vehicle_without_rclpy_guard():
    _require_runtime_support()

    stop_calls: list[str] = []
    manager = object.__new__(ModeManager)
    manager.vehicle = SimpleNamespace(stop=lambda: stop_calls.append("stop"))
    manager.get_logger = lambda: _FakeLogger()

    ModeManager._stop_vehicle(manager)

    assert stop_calls == ["stop"]


def test_mode_manager_terminate_deactivates_mode_and_stops_vehicle():
    _require_runtime_support()

    events: list[str] = []
    logger = _FakeLogger()

    class _FakeMode:
        def __init__(self) -> None:
            self.active = True

        def deactivate(self) -> None:
            events.append("deactivate")
            self.active = False

    manager = object.__new__(ModeManager)
    manager.vehicle = SimpleNamespace(stop=lambda: events.append("stop"))
    manager.modes = {"start": _FakeMode()}
    manager.transitions = {}
    manager.active_mode = "start"
    manager.get_logger = lambda: logger
    manager.destroy_node = lambda: events.append("destroy")

    ModeManager.handle_mode_state(manager, "terminate")

    assert events == ["deactivate", "stop", "destroy"]
    assert manager.active_mode is None


def test_payload_mission_main_stops_vehicle_during_external_shutdown(monkeypatch):
    _require_runtime_support()

    if payload_mission_module is None:
        pytest.skip(
            "payload_interfaces Python module is not available in this test environment"
        )

    events: list[str] = []

    class _Bootstrap:
        def destroy_node(self) -> None:
            events.append("bootstrap_destroy")

        def manager_kwargs(self) -> dict:
            events.append("manager_kwargs")
            return {}

    class _MissionNode:
        def _deactivate_active_mode(self) -> None:
            events.append("deactivate")

        def _stop_vehicle(self) -> None:
            events.append("stop")

        def destroy_node(self) -> None:
            events.append("destroy")

    mission_node = _MissionNode()

    monkeypatch.setattr(
        payload_mission_module.rclpy,
        "init",
        lambda args=None: events.append("init"),
        raising=False,
    )
    monkeypatch.setattr(
        payload_mission_module.rclpy,
        "shutdown",
        lambda: events.append("shutdown"),
        raising=False,
    )
    monkeypatch.setattr(payload_mission_module.rclpy, "ok", lambda: False)
    monkeypatch.setattr(payload_mission_module, "PayloadMissionBootstrap", _Bootstrap)
    monkeypatch.setattr(
        payload_mission_module,
        "PayloadModeManager",
        lambda **kwargs: events.append(f"manager:{kwargs}") or mission_node,
    )
    monkeypatch.setattr(
        payload_mission_module.rclpy,
        "spin",
        lambda _node: (_ for _ in ()).throw(
            payload_mission_module.ExternalShutdownException()
        ),
        raising=False,
    )

    payload_mission_module.main()

    assert events == [
        "init",
        "manager_kwargs",
        "bootstrap_destroy",
        "manager:{}",
        "deactivate",
        "stop",
        "destroy",
    ]
