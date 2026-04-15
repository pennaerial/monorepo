from __future__ import annotations

import builtins
import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


_EXCLUDED_KEYS = {
    "airframe",
    "custom_airframe_model",
    "model",
    "sim",
    "px4_namespace",
    "px4_instance",
    "payload_controller",
}


def _load_fleet_module():
    package_root = Path(__file__).resolve().parents[1]
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
    sim_package_root = Path(__file__).resolve().parents[2] / "sim"
    if str(sim_package_root) not in sys.path:
        sys.path.insert(0, str(sim_package_root))
    launch_path = package_root / "launch" / "fleet.launch.py"
    spec = importlib.util.spec_from_file_location(
        "uav_fleet_launch_helpers", launch_path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def fleet_launch_module():
    return _load_fleet_module()


def _example_fleet():
    return {
        "backend": {
            "kind": "sim",
            "px4_path": "~/PX4-Autopilot",
            "world_name": "sae",
            "mission_stage": "payload_retreat",
            "world_overrides": {
                "params": {
                    "controllables": {
                        "uav_model_0": {
                            "kind": "uav",
                            "px4_airframe_id": 4010,
                            "model": "gz_x500_mono_cam",
                            "path_to_sdf": "~/.simulation-gazebo/models/x500_mono_cam/model.sdf",
                            "position": [5.0, 0.0, 0.1],
                            "rpy": [0.0, 0.0, 0.0],
                        }
                    }
                }
            },
        },
        "defaults": {
            "auto_launch": True,
            "debug": False,
            "vision_debug": False,
            "force_camera": False,
            "save_vision_milliseconds": 0,
            "servo_only": False,
            "camera_mount_offsets": [0.0, 0.0, 0.0],
            "camera_input_transport": "raw",
            "camera_rotate_degrees": 0.0,
            "camera_preprocess_hook": "",
        },
        "vehicles": [
            {
                "name": "uav_alpha",
                "controllable": "uav_model_0",
                "mission": "hover",
                "camera_mount_offsets": [0.2, 0.0, 0.0],
            },
            {
                "name": "payload_alpha",
                "controllable": "payload_0",
                "mission": "payload_retreat",
                "vision_debug": True,
                "force_camera": True,
            },
        ],
    }


@pytest.fixture
def fake_mission_loader(monkeypatch, fleet_launch_module):
    def _load(path: str):
        if "hover" in path:
            return SimpleNamespace(target="uav")
        if "payload_retreat" in path:
            return SimpleNamespace(target="payload")
        raise AssertionError(f"Unexpected mission path requested in test: {path}")

    monkeypatch.setattr(fleet_launch_module.MissionSpec, "load", staticmethod(_load))
    return _load


def test_backend_config_rejects_unsupported_kind(fleet_launch_module):
    fleet = _example_fleet()
    fleet["backend"]["kind"] = "unknown"
    with pytest.raises(ValueError, match="Unsupported backend kind"):
        fleet_launch_module._backend_config(fleet)


def test_backend_config_accepts_hardware_backend(fleet_launch_module):
    fleet = {
        "backend": {"kind": "hardware", "px4_path": "~/PX4-Autopilot"},
        "vehicles": [
            {
                "name": "uav_alpha",
                "mission_path": "/tmp/hover.yaml",
                "kind": "uav",
                "px4_airframe_id": 4004,
            }
        ],
    }

    backend = fleet_launch_module._backend_config(fleet)

    assert backend["kind"] == "hardware"
    assert backend["px4_path"] == "~/PX4-Autopilot"


def test_backend_config_accepts_real_backend(fleet_launch_module):
    fleet = {
        "backend": {"kind": "real", "px4_path": "~/PX4-Autopilot"},
        "vehicles": [
            {
                "name": "uav_alpha",
                "mission_path": "/tmp/hover.yaml",
                "kind": "uav",
                "px4_airframe_id": 4004,
            }
        ],
    }

    backend = fleet_launch_module._backend_config(fleet)

    assert backend["kind"] == "hardware"
    assert backend["px4_path"] == "~/PX4-Autopilot"


def test_real_backend_does_not_import_sim(monkeypatch):
    package_root = Path(__file__).resolve().parents[1]
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
    launch_path = package_root / "launch" / "fleet.launch.py"
    spec = importlib.util.spec_from_file_location(
        "uav_fleet_launch_hardware_only", launch_path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None

    real_import = builtins.__import__

    def _import(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "sim" or name.startswith("sim."):
            raise ModuleNotFoundError("No module named 'sim'")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", _import)
    spec.loader.exec_module(module)

    backend = module._backend_config(
        {
            "backend": {"kind": "real"},
            "vehicles": [
                {
                    "name": "uav_alpha",
                    "mission_path": "/tmp/hover.yaml",
                    "kind": "uav",
                    "px4_airframe_id": 4004,
                }
            ],
        }
    )

    assert backend["kind"] == "hardware"


def test_backend_config_rejects_embedded_world(fleet_launch_module):
    fleet = _example_fleet()
    fleet["backend"]["world"] = {"name": "SAEWorldNode", "params": {}}
    with pytest.raises(ValueError, match="no longer support 'backend.world'"):
        fleet_launch_module._backend_config(fleet)


def test_backend_config_defaults_missing_stage_to_base(fleet_launch_module):
    fleet = _example_fleet()
    fleet["backend"].pop("mission_stage", None)
    fleet["backend"]["world_overrides"] = {}

    backend = fleet_launch_module._backend_config(fleet)

    assert backend["world_name"] == "sae"
    assert backend["mission_stage"] == "base"
    assert "payload_0" in backend["_resolved_world"]["params"]["controllables"]
    assert "payload_1" in backend["_resolved_world"]["params"]["controllables"]


def test_backend_config_requires_controllables(fleet_launch_module):
    fleet = _example_fleet()
    fleet["backend"]["world_name"] = "custom"
    fleet["backend"].pop("mission_stage", None)
    fleet["backend"].pop("world_overrides", None)
    with pytest.raises(ValueError, match="non-empty world.params.controllables"):
        fleet_launch_module._backend_config(fleet)


def test_vehicle_stack_configs_defaults_namespaces_and_instances(
    fleet_launch_module, fake_mission_loader
):
    backend, vehicles = fleet_launch_module._vehicle_stack_configs(_example_fleet())

    assert backend["world_name"] == "sae"
    assert backend["mission_stage"] == "payload_retreat"
    assert vehicles[0]["vehicle_name"] == "uav_alpha"
    assert vehicles[0]["sim_entity_name"] == "uav_model_0"
    assert vehicles[0]["px4_instance"] == 0
    assert vehicles[0]["px4_airframe_id"] == 4010
    assert vehicles[0]["launch_middleware"] is False
    assert vehicles[0]["launch_px4_sitl"] is True
    assert vehicles[1]["vehicle_name"] == "payload_alpha"
    assert vehicles[1]["sim_entity_name"] == "payload_0"
    assert vehicles[1]["launch_payload_backend"] is True
    assert vehicles[0]["auto_launch"] is True
    assert vehicles[0]["debug"] is False
    assert vehicles[0]["vision_debug"] is False
    assert vehicles[0]["force_camera"] is False
    assert vehicles[0]["save_vision_milliseconds"] == 0
    assert vehicles[0]["servo_only"] is False
    assert vehicles[0]["camera_mount_offsets"] == [0.2, 0.0, 0.0]
    assert vehicles[0]["camera_input_transport"] == "raw"
    assert vehicles[0]["camera_rotate_degrees"] == 0.0
    assert vehicles[0]["camera_preprocess_hook"] == ""
    assert vehicles[1]["auto_launch"] is True
    assert vehicles[1]["debug"] is False
    assert vehicles[1]["vision_debug"] is True
    assert vehicles[1]["force_camera"] is True
    assert vehicles[1]["save_vision_milliseconds"] == 0
    assert vehicles[1]["servo_only"] is False
    assert vehicles[1]["camera_mount_offsets"] == [0.0, 0.0, 0.0]
    assert vehicles[1]["camera_input_transport"] == "raw"
    assert vehicles[1]["camera_rotate_degrees"] == 0.0
    assert vehicles[1]["camera_preprocess_hook"] == ""


@pytest.mark.parametrize("key", sorted(_EXCLUDED_KEYS))
def test_fleet_defaults_reject_excluded_keys(fleet_launch_module, key):
    fleet = _example_fleet()
    fleet["defaults"][key] = "forbidden"

    with pytest.raises(ValueError, match=rf"do not support '{key}'"):
        fleet_launch_module._defaults_config(fleet)


@pytest.mark.parametrize("key", sorted(_EXCLUDED_KEYS))
def test_fleet_vehicle_entries_reject_excluded_keys(
    fleet_launch_module, fake_mission_loader, key
):
    fleet = _example_fleet()
    fleet["vehicles"][0][key] = "forbidden"

    with pytest.raises(ValueError, match=rf"does not support '{key}'"):
        fleet_launch_module._vehicle_stack_configs(fleet)


def test_fleet_defaults_reject_unknown_keys(fleet_launch_module):
    fleet = _example_fleet()
    fleet["defaults"]["unknown_knob"] = True

    with pytest.raises(ValueError, match="field 'unknown_knob' is not supported"):
        fleet_launch_module._defaults_config(fleet)


def test_fleet_defaults_must_be_mapping(fleet_launch_module):
    fleet = _example_fleet()
    fleet["defaults"] = ["auto_launch"]

    with pytest.raises(ValueError, match="defaults section must be a mapping"):
        fleet_launch_module._defaults_config(fleet)


def test_vehicle_stack_configs_rejects_unknown_controllable(
    fleet_launch_module, fake_mission_loader
):
    fleet = _example_fleet()
    fleet["vehicles"][0]["controllable"] = "missing"
    with pytest.raises(ValueError, match="unknown controllable 'missing'"):
        fleet_launch_module._vehicle_stack_configs(fleet)


def test_vehicle_stack_configs_rejects_declared_kind_mismatch(
    fleet_launch_module, fake_mission_loader
):
    fleet = _example_fleet()
    fleet["vehicles"][0]["kind"] = "payload"
    with pytest.raises(ValueError, match="declared kind 'payload'"):
        fleet_launch_module._vehicle_stack_configs(fleet)


def test_vehicle_stack_configs_rejects_missing_backend_airframe_id(
    fleet_launch_module, fake_mission_loader
):
    fleet = _example_fleet()
    del fleet["backend"]["world_overrides"]["params"]["controllables"]["uav_model_0"][
        "px4_airframe_id"
    ]
    with pytest.raises(ValueError, match="define px4_airframe_id"):
        fleet_launch_module._vehicle_stack_configs(fleet)


def test_vehicle_stack_configs_support_hardware_uav(
    fleet_launch_module, fake_mission_loader
):
    fleet = {
        "backend": {"kind": "hardware", "px4_path": "~/PX4-Autopilot"},
        "vehicles": [
            {
                "name": "uav_alpha",
                "mission_path": "/tmp/hover.yaml",
                "kind": "uav",
                "px4_airframe_id": 4004,
                "auto_launch": False,
            }
        ],
    }

    backend, vehicles = fleet_launch_module._vehicle_stack_configs(fleet)

    assert backend["kind"] == "hardware"
    assert vehicles[0]["sim"] is False
    assert vehicles[0]["launch_px4_sitl"] is False
    assert vehicles[0]["launch_middleware"] is True
    assert vehicles[0]["px4_airframe_id"] == 4004
    assert vehicles[0]["auto_launch"] is False


def test_vehicle_stack_configs_support_real_uav_alias(
    fleet_launch_module, fake_mission_loader
):
    fleet = {
        "backend": {"kind": "real", "px4_path": "~/PX4-Autopilot"},
        "vehicles": [
            {
                "name": "uav_alpha",
                "mission_path": "/tmp/hover.yaml",
                "kind": "uav",
                "px4_airframe_id": 4004,
                "auto_launch": False,
            }
        ],
    }

    backend, vehicles = fleet_launch_module._vehicle_stack_configs(fleet)

    assert backend["kind"] == "hardware"
    assert vehicles[0]["sim"] is False
    assert vehicles[0]["launch_px4_sitl"] is False
    assert vehicles[0]["launch_middleware"] is True
    assert vehicles[0]["px4_airframe_id"] == 4004
    assert vehicles[0]["auto_launch"] is False


def test_vehicle_stack_configs_support_hardware_payload(
    fleet_launch_module, fake_mission_loader
):
    fleet = {
        "backend": {"kind": "hardware"},
        "vehicles": [
            {
                "name": "payload_alpha",
                "mission_path": "/tmp/payload_retreat.yaml",
                "kind": "payload",
                "payload_controller": "GPIOController",
            }
        ],
    }

    _, vehicles = fleet_launch_module._vehicle_stack_configs(fleet)

    assert vehicles[0]["sim"] is False
    assert vehicles[0]["launch_payload_backend"] is True
    assert vehicles[0]["payload_controller"] == "GPIOController"
