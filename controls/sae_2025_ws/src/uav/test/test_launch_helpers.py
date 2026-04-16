from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


def _load_launch_module(filename: str, module_name: str):
    package_root = Path(__file__).resolve().parents[1]
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
    sim_package_root = Path(__file__).resolve().parents[2] / "sim"
    if str(sim_package_root) not in sys.path:
        sys.path.insert(0, str(sim_package_root))
    # test_runtime_behavior installs lightweight rclpy stubs into sys.modules.
    # launch_ros needs the real ROS Python packages here.
    fake_rclpy = sys.modules.get("rclpy")
    if fake_rclpy is not None and not hasattr(fake_rclpy, "__path__"):
        for module_name_key in list(sys.modules):
            if module_name_key == "rclpy" or module_name_key.startswith("rclpy."):
                del sys.modules[module_name_key]
    launch_path = package_root / "launch" / filename
    spec = importlib.util.spec_from_file_location(module_name, launch_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def main_launch_module():
    return _load_launch_module("main.launch.py", "uav_main_launch_helpers")


@pytest.fixture(scope="module")
def stack_launch_module():
    return _load_launch_module("vehicle_stack.launch.py", "uav_vehicle_stack_helpers")


def test_yaml_or_launch_string_prefers_launch_override(monkeypatch, main_launch_module):
    monkeypatch.setattr(
        main_launch_module,
        "LaunchConfiguration",
        lambda name: SimpleNamespace(perform=lambda context: "  override-value  "),
    )

    assert (
        main_launch_module._yaml_or_launch_string(
            SimpleNamespace(), "mission_name", "basic"
        )
        == "override-value"
    )


def test_yaml_or_launch_string_falls_back_to_yaml_value(
    monkeypatch, main_launch_module
):
    monkeypatch.setattr(
        main_launch_module,
        "LaunchConfiguration",
        lambda name: SimpleNamespace(perform=lambda context: "   "),
    )

    assert (
        main_launch_module._yaml_or_launch_string(SimpleNamespace(), "vehicle_name", 7)
        == "7"
    )
    assert (
        main_launch_module._yaml_or_launch_string(
            SimpleNamespace(), "vehicle_name", None
        )
        == ""
    )


def test_yaml_bool_value_accepts_boolean_and_default(main_launch_module):
    assert (
        main_launch_module._yaml_bool_value(True, name="auto_launch", default=False)
        is True
    )
    assert (
        main_launch_module._yaml_bool_value(False, name="auto_launch", default=True)
        is False
    )
    assert (
        main_launch_module._yaml_bool_value(None, name="auto_launch", default=True)
        is True
    )
    assert (
        main_launch_module._yaml_bool_value(None, name="auto_launch", default=False)
        is False
    )


def test_yaml_bool_value_rejects_non_boolean(main_launch_module):
    with pytest.raises(ValueError, match="must be a YAML boolean"):
        main_launch_module._yaml_bool_value("false", name="auto_launch", default=True)


def test_main_launch_force_camera_prefers_new_key(main_launch_module):
    assert (
        main_launch_module._resolve_force_camera(
            {"force_camera": True, "use_camera": False}
        )
        is True
    )


def test_main_launch_force_camera_accepts_legacy_alias(main_launch_module):
    assert main_launch_module._resolve_force_camera({"use_camera": True}) is True


def test_single_vehicle_config_carries_camera_calibration_file(
    monkeypatch, main_launch_module
):
    monkeypatch.setattr(
        main_launch_module, "mission_path_for_name", lambda name: f"/tmp/{name}.yaml"
    )
    monkeypatch.setattr(
        main_launch_module.MissionSpec,
        "load",
        staticmethod(
            lambda _path: SimpleNamespace(is_uav=False, is_payload=True, target="payload")
        ),
    )
    monkeypatch.setattr(
        main_launch_module,
        "find_folder_with_heuristic",
        lambda *_args, **_kwargs: "/tmp/PX4-Autopilot",
    )

    class FakeLaunchConfiguration:
        def __init__(self, name: str):
            self.name = name

        def perform(self, _context):
            return {"px4_path": "~/PX4-Autopilot"}.get(self.name, "")

    monkeypatch.setattr(
        main_launch_module, "LaunchConfiguration", FakeLaunchConfiguration
    )

    vehicle_config, backend = main_launch_module._single_vehicle_config(
        SimpleNamespace(),
        {
            "mission_name": "payload_retreat",
            "vehicle_name": "payload_7",
            "sim": False,
            "camera_calibration_file": "payload_7_camera_info.yaml",
        },
    )

    assert backend is None
    assert vehicle_config["camera_calibration_file"] == "payload_7_camera_info.yaml"


@pytest.mark.parametrize(
    ("mission_spec", "expected"),
    [
        (SimpleNamespace(is_uav=True, is_payload=False, target="uav"), "uav_mission"),
        (
            SimpleNamespace(is_uav=False, is_payload=True, target="payload"),
            "payload_mission",
        ),
    ],
)
def test_runtime_executable_for_supported_targets(
    stack_launch_module, mission_spec, expected
):
    assert stack_launch_module._runtime_executable_for(mission_spec) == expected


def test_runtime_executable_for_rejects_unsupported_target(stack_launch_module):
    mission_spec = SimpleNamespace(
        is_uav=False, is_payload=False, target="experimental"
    )
    with pytest.raises(ValueError, match="Unsupported mission target 'experimental'"):
        stack_launch_module._runtime_executable_for(mission_spec)


@pytest.mark.parametrize(
    ("value", "default", "expected"),
    [
        (True, False, True),
        (False, True, False),
        ("true", False, True),
        ("no", True, False),
        (None, True, True),
    ],
)
def test_resolve_bool(stack_launch_module, value, default, expected):
    assert (
        stack_launch_module._resolve_bool({"flag": value}, "flag", default) is expected
    )


def test_resolve_bool_rejects_invalid_value(stack_launch_module):
    with pytest.raises(ValueError, match="expected boolean 'flag'"):
        stack_launch_module._resolve_bool({"flag": "maybe"}, "flag", True)


def test_vehicle_stack_force_camera_prefers_new_key(stack_launch_module):
    assert (
        stack_launch_module._resolve_force_camera(
            {"force_camera": True, "use_camera": False}
        )
        is True
    )


def test_vehicle_stack_force_camera_accepts_legacy_alias(stack_launch_module):
    assert stack_launch_module._resolve_force_camera({"use_camera": "yes"}) is True


def test_launch_setup_starts_camera_actions_for_camera_only_mission(
    monkeypatch, stack_launch_module
):
    monkeypatch.setenv("ROS_LOG_DIR", "/tmp/ros_logs")
    mission_spec = SimpleNamespace(
        is_uav=False,
        is_payload=True,
        target="payload",
        vision_nodes=(),
        requires_camera=True,
    )
    recorded = {}

    monkeypatch.setattr(
        stack_launch_module,
        "_load_vehicle_config",
        lambda _context: {
            "vehicle_name": "payload_0",
            "sim": False,
            "auto_launch": True,
            "debug": False,
            "vision_debug": False,
            "launch_payload_backend": False,
        },
    )
    monkeypatch.setattr(
        stack_launch_module,
        "_resolve_mission_path",
        lambda _config: "/tmp/mission.yaml",
    )
    monkeypatch.setattr(
        stack_launch_module.MissionSpec,
        "load",
        staticmethod(lambda _path: mission_spec),
    )
    monkeypatch.setattr(
        stack_launch_module,
        "_runtime_executable_for",
        lambda _mission_spec: "payload_mission",
    )
    monkeypatch.setattr(
        stack_launch_module,
        "_resolve_camera_input_transport",
        lambda **_kwargs: "compressed",
    )
    monkeypatch.setattr(
        stack_launch_module,
        "_camera_contract_for",
        lambda *args, **kwargs: {
            "vehicle_name": "payload_0",
            "camera_info_url": "",
            "mission_target": "payload",
        },
    )

    def _record_camera_actions(**kwargs):
        recorded["camera_actions"] = kwargs
        return ["camera"]

    monkeypatch.setattr(
        stack_launch_module,
        "_build_camera_actions",
        _record_camera_actions,
    )
    monkeypatch.setattr(
        stack_launch_module,
        "_build_runtime_parameters",
        lambda **_kwargs: {},
    )
    monkeypatch.setattr(
        stack_launch_module,
        "_payload_launch_action",
        lambda **_kwargs: "payload",
    )
    monkeypatch.setattr(
        stack_launch_module,
        "Node",
        lambda **kwargs: ("node", kwargs),
    )

    actions = stack_launch_module.launch_setup(SimpleNamespace())

    assert "camera_actions" in recorded
    assert recorded["camera_actions"]["vision_nodes"] == []
    assert "camera" in actions


def test_launch_setup_validates_uav_camera_sensor_for_camera_only_mission(
    monkeypatch, stack_launch_module
):
    monkeypatch.setenv("ROS_LOG_DIR", "/tmp/ros_logs")
    mission_spec = SimpleNamespace(
        is_uav=True,
        is_payload=False,
        target="uav",
        vision_nodes=(),
        requires_camera=True,
    )

    monkeypatch.setattr(
        stack_launch_module,
        "_load_vehicle_config",
        lambda _context: {
            "vehicle_name": "uav_0",
            "sim": False,
            "auto_launch": True,
            "debug": False,
            "vision_debug": False,
            "launch_middleware": False,
            "launch_px4_sitl": False,
            "px4_path": "/tmp/PX4-Autopilot",
        },
    )
    monkeypatch.setattr(
        stack_launch_module,
        "_resolve_mission_path",
        lambda _config: "/tmp/mission.yaml",
    )
    monkeypatch.setattr(
        stack_launch_module.MissionSpec,
        "load",
        staticmethod(lambda _path: mission_spec),
    )
    monkeypatch.setattr(
        stack_launch_module,
        "_runtime_executable_for",
        lambda _mission_spec: "uav_mission",
    )
    monkeypatch.setattr(
        stack_launch_module,
        "find_folder_with_heuristic",
        lambda *_args, **_kwargs: "/tmp/PX4-Autopilot",
    )
    monkeypatch.setattr(
        stack_launch_module,
        "_resolve_uav_airframe",
        lambda _config, _px4_path: (
            SimpleNamespace(name="MULTICOPTER"),
            4004,
            "no_camera",
        ),
    )
    monkeypatch.setattr(stack_launch_module, "vehicle_camera_map", {})

    with pytest.raises(ValueError, match="does not have a camera sensor configured"):
        stack_launch_module.launch_setup(SimpleNamespace())


def test_vehicle_namespace_for_rejects_empty(stack_launch_module):
    with pytest.raises(ValueError, match="non-empty vehicle_name"):
        stack_launch_module._vehicle_namespace_for(" / ")


def test_camera_contract_for_uav_and_payload(stack_launch_module):
    uav_contract = stack_launch_module._camera_contract_for(
        SimpleNamespace(is_uav=True, is_payload=False, target="uav"),
        vehicle_name="uav_7",
    )
    assert uav_contract == {
        "vehicle_name": "uav_7",
        "camera_namespace": "uav_7",
        "image_topic": "camera",
        "camera_info_topic": "camera_info",
        "camera_service_name": "camera_data",
        "input_transport": "",
        "input_raw_topic": "camera_source",
        "input_compressed_topic": "camera_source/compressed",
        "input_camera_info_topic": "camera_info_source",
        "rotate_degrees": 0.0,
        "preprocess_hook": "",
        "camera_info_url": "",
        "mission_target": "uav",
    }

    payload_contract = stack_launch_module._camera_contract_for(
        SimpleNamespace(is_uav=False, is_payload=True, target="payload"),
        vehicle_name="payload_2",
        input_transport="compressed",
        rotate_degrees=180.0,
        preprocess_hook="rotate_then_crop",
        camera_info_url="file:///tmp/payload.yaml",
    )
    assert payload_contract == {
        "vehicle_name": "payload_2",
        "camera_namespace": "payload_2",
        "image_topic": "camera",
        "camera_info_topic": "camera_info",
        "camera_service_name": "camera_data",
        "input_transport": "compressed",
        "input_raw_topic": "camera_source",
        "input_compressed_topic": "camera_source/compressed",
        "input_camera_info_topic": "camera_info_source",
        "rotate_degrees": 180.0,
        "preprocess_hook": "rotate_then_crop",
        "camera_info_url": "file:///tmp/payload.yaml",
        "mission_target": "payload",
    }


def test_resolve_camera_input_transport_prefers_raw_for_payload_preprocessing(
    stack_launch_module,
):
    mission_spec = SimpleNamespace(is_uav=False, is_payload=True, target="payload")

    assert (
        stack_launch_module._resolve_camera_input_transport(
            mission_spec=mission_spec,
            sim=False,
            configured_transport="compressed",
            rotate_degrees=180.0,
            preprocess_hook="",
        )
        == "raw"
    )
    assert (
        stack_launch_module._resolve_camera_input_transport(
            mission_spec=mission_spec,
            sim=False,
            configured_transport="compressed",
            rotate_degrees=0.0,
            preprocess_hook="pkg.module:hook",
        )
        == "raw"
    )
    assert (
        stack_launch_module._resolve_camera_input_transport(
            mission_spec=mission_spec,
            sim=False,
            configured_transport="compressed",
            rotate_degrees=0.0,
            preprocess_hook="",
        )
        == "compressed"
    )
    assert (
        stack_launch_module._resolve_camera_input_transport(
            mission_spec=mission_spec,
            sim=False,
            configured_transport="raw",
            rotate_degrees=180.0,
            preprocess_hook="",
        )
        == "raw"
    )


def test_payload_camera_info_url_for_prefers_named_file(
    monkeypatch, tmp_path, stack_launch_module
):
    uav_share = tmp_path / "uav_share"
    calibration_dir = uav_share / "config" / "camera_calibrations"
    calibration_dir.mkdir(parents=True)
    preferred = calibration_dir / "payload_2_camera_info.yaml"
    fallback = calibration_dir / "payload_0_camera_info.yaml"
    fallback.write_text("fallback", encoding="utf-8")
    preferred.write_text("preferred", encoding="utf-8")

    monkeypatch.setattr(
        stack_launch_module,
        "get_package_share_directory",
        lambda package_name: str(uav_share),
    )

    expected = f"file://{preferred}"
    assert stack_launch_module._payload_camera_info_url_for("payload_2") == expected

def test_payload_camera_info_url_for_uses_explicit_filename(
    monkeypatch, tmp_path, stack_launch_module
):
    uav_share = tmp_path / "uav_share"
    calibration_dir = uav_share / "config" / "camera_calibrations"
    calibration_dir.mkdir(parents=True)
    requested = calibration_dir / "warehouse_cam.yaml"
    requested.write_text("warehouse", encoding="utf-8")

    monkeypatch.setattr(
        stack_launch_module,
        "get_package_share_directory",
        lambda package_name: str(uav_share),
    )

    expected = f"file://{requested}"
    assert (
        stack_launch_module._payload_camera_info_url_for(
            "payload_2", camera_calibration_file="warehouse_cam.yaml"
        )
        == expected
    )



def test_payload_camera_info_url_for_falls_back_to_payload_0(
    monkeypatch, tmp_path, stack_launch_module
):
    uav_share = tmp_path / "uav_share"
    calibration_dir = uav_share / "config" / "camera_calibrations"
    calibration_dir.mkdir(parents=True)
    fallback = calibration_dir / "payload_0_camera_info.yaml"
    fallback.write_text("fallback", encoding="utf-8")

    monkeypatch.setattr(
        stack_launch_module,
        "get_package_share_directory",
        lambda package_name: str(uav_share),
    )

    expected = f"file://{fallback}"
    assert stack_launch_module._payload_camera_info_url_for("payload_2") == expected


def test_payload_camera_info_url_for_rejects_missing_explicit_file(
    monkeypatch, tmp_path, stack_launch_module
):
    uav_share = tmp_path / "uav_share"
    calibration_dir = uav_share / "config" / "camera_calibrations"
    calibration_dir.mkdir(parents=True)

    monkeypatch.setattr(
        stack_launch_module,
        "get_package_share_directory",
        lambda package_name: str(uav_share),
    )

    with pytest.raises(ValueError, match="requested camera calibration file"):
        stack_launch_module._payload_camera_info_url_for(
            "payload_2", camera_calibration_file="missing.yaml"
        )


def test_build_runtime_parameters_for_uav(stack_launch_module):
    parameters = stack_launch_module._build_runtime_parameters(
        mission_path="/tmp/mission.yaml",
        mission_spec=SimpleNamespace(is_payload=False, is_uav=True),
        vehicle_name="uav_3",
        auto_launch=True,
        debug=1,
        servo_only=0,
        vehicle_class_name="MULTICOPTER",
        camera_mount_offsets=[0.1, 0.2, 0.3],
    )

    assert parameters == {
        "mode_map": "/tmp/mission.yaml",
        "auto_launch": True,
        "vehicle_name": "uav_3",
        "debug": True,
        "servo_only": False,
        "vehicle_class": "MULTICOPTER",
        "camera_mount_offsets": [0.1, 0.2, 0.3],
    }


def test_build_runtime_parameters_for_payload(stack_launch_module):
    parameters = stack_launch_module._build_runtime_parameters(
        mission_path="/tmp/payload.yaml",
        mission_spec=SimpleNamespace(is_payload=True, is_uav=False),
        vehicle_name="payload_0",
        auto_launch=False,
        debug=True,
        servo_only=True,
        vehicle_class_name=None,
        camera_mount_offsets=[],
    )

    assert parameters == {
        "mode_map": "/tmp/payload.yaml",
        "auto_launch": False,
        "vehicle_name": "payload_0",
    }


def test_build_runtime_parameters_rejects_missing_vehicle_class(stack_launch_module):
    with pytest.raises(ValueError, match="UAV missions require a vehicle class"):
        stack_launch_module._build_runtime_parameters(
            mission_path="/tmp/mission.yaml",
            mission_spec=SimpleNamespace(is_payload=False, is_uav=True),
            vehicle_name="uav",
            auto_launch=True,
            debug=False,
            servo_only=False,
            vehicle_class_name=None,
            camera_mount_offsets=[0.0, 0.0, 0.0],
        )


def test_build_runtime_parameters_rejects_bad_camera_offsets(stack_launch_module):
    with pytest.raises(
        ValueError, match="camera_mount_offsets must have exactly 3 values"
    ):
        stack_launch_module._build_runtime_parameters(
            mission_path="/tmp/mission.yaml",
            mission_spec=SimpleNamespace(is_payload=False, is_uav=True),
            vehicle_name="uav",
            auto_launch=True,
            debug=False,
            servo_only=False,
            vehicle_class_name="MULTICOPTER",
            camera_mount_offsets=[0.0, 0.0],
        )


def test_legacy_backend_override_uses_stage_contract_for_uav(
    monkeypatch, main_launch_module
):
    monkeypatch.setattr(
        main_launch_module,
        "resolve_stage_world",
        lambda **kwargs: {
            "mission_stage": "payload_retreat",
            "sim_stage_params": {
                "world": {"params": {"vehicle_pose": [1.0, 2.0, 3.0, 0.0, 0.0, 0.5]}}
            },
            "world": {
                "params": {
                    "controllables": {"payload_0": {"kind": "payload"}},
                }
            },
        },
    )

    backend, world_name = main_launch_module._legacy_backend_override(
        mission_spec=SimpleNamespace(is_uav=True, is_payload=False),
        sim_params={"competition": 3, "mission_stage": "payload_retreat"},
        vehicle_name="uav_alpha",
        model="gz_x500_mono_cam",
        px4_airframe_id=4010,
    )

    assert world_name == "sae"
    assert backend == {
        "kind": "sim",
        "world_name": "sae",
        "mission_stage": "payload_retreat",
        "world_overrides": {
            "params": {
                "controllables": {
                    "uav_alpha": {
                        "kind": "uav",
                        "px4_airframe_id": 4010,
                        "path_to_sdf": "~/.simulation-gazebo/models/x500_mono_cam/model.sdf",
                        "model": "gz_x500_mono_cam",
                        "position": [1.0, 2.0, 3.0],
                        "rpy": [0.0, 0.0, 0.5],
                    }
                }
            }
        },
        "scoring": False,
    }


def test_legacy_backend_override_rejects_unknown_payload(
    monkeypatch, main_launch_module
):
    monkeypatch.setattr(
        main_launch_module,
        "resolve_stage_world",
        lambda **kwargs: {
            "mission_stage": "base",
            "sim_stage_params": {"world": {"params": {}}},
            "world": {"params": {"controllables": {"payload_1": {"kind": "payload"}}}},
        },
    )

    with pytest.raises(ValueError, match="Configured vehicle_name 'payload_0'"):
        main_launch_module._legacy_backend_override(
            mission_spec=SimpleNamespace(is_uav=False, is_payload=True),
            sim_params={"competition": 3},
            vehicle_name="payload_0",
            model="",
        )


def test_sim_camera_bridge_actions_use_entity_name_for_payload(
    monkeypatch, stack_launch_module
):
    class FakeNode:
        def __init__(self, **kwargs):
            self.arguments = kwargs.get("arguments", [])
            self.remappings = kwargs.get("remappings", [])
            self.namespace = kwargs.get("namespace", "")

    monkeypatch.setattr(stack_launch_module, "Node", FakeNode)

    actions = stack_launch_module._sim_camera_bridge_actions(
        world_name="sae",
        vehicle_name="payload_ns",
        sim_entity_name="payload_model_7",
        mission_target="payload",
    )
    image_bridge = actions[0]
    info_bridge = actions[1]
    assert (
        "/world/sae/model/payload_model_7/link/camera_link/sensor/camera/image"
        in image_bridge.arguments[0]
    )
    assert image_bridge.remappings == [
        (
            "/world/sae/model/payload_model_7/link/camera_link/sensor/camera/image",
            "camera_source",
        )
    ]
    assert image_bridge.namespace == "payload_ns"
    assert info_bridge.remappings == [
        (
            "/world/sae/model/payload_model_7/link/camera_link/sensor/camera/camera_info",
            "camera_info_source",
        )
    ]
    assert info_bridge.namespace == "payload_ns"


def test_legacy_backend_override_returns_stage_based_backend_for_uav(
    monkeypatch, main_launch_module
):
    monkeypatch.setattr(
        main_launch_module, "_resolved_sim_world_name", lambda params: "sae"
    )
    monkeypatch.setattr(
        main_launch_module,
        "resolve_stage_world",
        lambda **kwargs: {
            "mission_stage": "payload_retreat",
            "sim_stage_params": {
                "world": {"params": {"vehicle_pose": [5.0, 0.0, 0.1, 0.0, 0.0, 0.0]}}
            },
            "world": {"params": {"controllables": {"payload_0": {"kind": "payload"}}}},
        },
    )

    backend, world_name = main_launch_module._legacy_backend_override(
        mission_spec=SimpleNamespace(is_uav=True, is_payload=False),
        sim_params={"mission_stage": "payload_retreat"},
        vehicle_name="uav_0",
        model="gz_x500_mono_cam",
        px4_airframe_id=4010,
    )

    assert world_name == "sae"
    assert backend == {
        "kind": "sim",
        "world_name": "sae",
        "mission_stage": "payload_retreat",
        "world_overrides": {
            "params": {
                "controllables": {
                    "uav_0": {
                        "kind": "uav",
                        "px4_airframe_id": 4010,
                        "path_to_sdf": "~/.simulation-gazebo/models/x500_mono_cam/model.sdf",
                        "model": "gz_x500_mono_cam",
                        "position": [5.0, 0.0, 0.1],
                        "rpy": [0.0, 0.0, 0.0],
                    }
                }
            }
        },
        "scoring": False,
    }
