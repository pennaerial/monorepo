from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from uav.runtime.ModeManager import ModeManager  # noqa: E402


def _load_fleet_module():
    package_root = PACKAGE_ROOT
    sim_package_root = Path(__file__).resolve().parents[2] / "sim"
    if str(sim_package_root) not in sys.path:
        sys.path.insert(0, str(sim_package_root))
    launch_path = package_root / "launch" / "fleet.launch.py"
    spec = importlib.util.spec_from_file_location(
        "uav_fleet_launch_schema_tests", launch_path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _TrackingMode:
    def __init__(self, label: str, *, status: str = "continue") -> None:
        self.label = label
        self.status = status
        self.deactivated = 0
        self.activated = 0

    def activate(self) -> None:
        self.activated += 1

    def deactivate(self) -> None:
        self.deactivated += 1

    def update(self, time_delta: float) -> None:
        return None

    def check_status(self) -> str:
        return self.status


def _make_mode_manager():
    manager = object.__new__(ModeManager)
    manager.vehicle = None
    manager.modes = {}
    manager.transitions = {}
    manager.active_mode = None
    manager.last_update_time = 0.0
    manager._vision_clients = {}
    manager.timer = None
    manager.auto_launch = False
    manager._auto_launch_timer = None
    manager._logger = SimpleNamespace(messages=[], info=lambda msg: None)
    manager.get_logger = lambda: manager._logger
    manager.create_timer = lambda period, callback: SimpleNamespace(cancel=lambda: None)
    manager.create_client = lambda *args, **kwargs: None
    manager.destroy_client = lambda client: None
    manager.destroy_node = lambda: None
    manager._stop_vehicle = lambda: None
    manager._runtime_vehicle_name = "schema-test"
    manager._current_comm_builder = None
    manager._managed_registry = SimpleNamespace(
        destroy_for_owner=lambda *args, **kwargs: None
    )
    return manager


def test_checked_in_fleet_specs_load_against_mission_schema(monkeypatch):
    fleet_launch_module = _load_fleet_module()
    fleets_dir = Path(__file__).resolve().parent.parent / "uav" / "fleets"
    missions_dir = Path(__file__).resolve().parent.parent / "uav" / "missions"
    fleet_paths = sorted(fleets_dir.glob("*.yaml"))
    assert fleet_paths, "Expected checked-in fleet specs under src/uav/uav/fleets"

    monkeypatch.setattr(
        fleet_launch_module,
        "mission_path_for_name",
        lambda mission_name: str((missions_dir / f"{mission_name}.yaml").resolve()),
    )

    for fleet_path in fleet_paths:
        fleet = yaml.safe_load(fleet_path.read_text(encoding="utf-8")) or {}
        backend, vehicles = fleet_launch_module._vehicle_stack_configs(fleet)

        assert backend["kind"] in {"sim", "hardware"}
        assert vehicles, f"{fleet_path.name} did not resolve any vehicles"

        for vehicle in vehicles:
            assert vehicle["kind"] in {"uav", "payload"}
            assert "mission_path" in vehicle
            assert Path(vehicle["mission_path"]).suffix == ".yaml"
            if vehicle["kind"] == "uav":
                assert isinstance(vehicle["px4_airframe_id"], int)
            else:
                assert vehicle["payload_controller"]


def test_fleet_vehicle_transition_labels_are_case_sensitive():
    manager = _make_mode_manager()
    start_mode = _TrackingMode("start", status="complete")
    next_mode = _TrackingMode("next")
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
