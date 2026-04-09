from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


def _load_launch_module():
    package_root = Path(__file__).resolve().parents[1]
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "main.launch.py"
    spec = importlib.util.spec_from_file_location("uav_main_launch_helpers", launch_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def launch_module():
    return _load_launch_module()


def test_yaml_or_launch_string_prefers_launch_override(monkeypatch, launch_module):
    monkeypatch.setattr(
        launch_module,
        "LaunchConfiguration",
        lambda name: SimpleNamespace(perform=lambda context: "  override-value  "),
    )

    assert (
        launch_module._yaml_or_launch_string(SimpleNamespace(), "mission_name", "basic")
        == "override-value"
    )


def test_yaml_or_launch_string_falls_back_to_yaml_value(monkeypatch, launch_module):
    monkeypatch.setattr(
        launch_module,
        "LaunchConfiguration",
        lambda name: SimpleNamespace(perform=lambda context: "   "),
    )

    assert launch_module._yaml_or_launch_string(SimpleNamespace(), "payload_name", 7) == "7"
    assert (
        launch_module._yaml_or_launch_string(SimpleNamespace(), "payload_name", None)
        == ""
    )


def test_yaml_bool_value_accepts_boolean_and_default(launch_module):
    assert launch_module._yaml_bool_value(True, name="auto_launch", default=False) is True
    assert launch_module._yaml_bool_value(False, name="auto_launch", default=True) is False
    assert launch_module._yaml_bool_value(None, name="auto_launch", default=True) is True
    assert launch_module._yaml_bool_value(None, name="auto_launch", default=False) is False


def test_yaml_bool_value_rejects_non_boolean(launch_module):
    with pytest.raises(ValueError, match="must be a YAML boolean"):
        launch_module._yaml_bool_value("false", name="auto_launch", default=True)


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
def test_runtime_executable_for_supported_targets(launch_module, mission_spec, expected):
    assert launch_module._runtime_executable_for(mission_spec) == expected


def test_runtime_executable_for_rejects_unsupported_target(launch_module):
    mission_spec = SimpleNamespace(is_uav=False, is_payload=False, target="experimental")

    with pytest.raises(ValueError, match="Unsupported mission target 'experimental'"):
        launch_module._runtime_executable_for(mission_spec)


@pytest.mark.parametrize(
    ("mission_spec", "sim", "expected"),
    [
        (SimpleNamespace(is_uav=True, is_payload=False), True, True),
        (SimpleNamespace(is_uav=True, is_payload=False), False, False),
        (SimpleNamespace(is_uav=False, is_payload=True), True, False),
        (SimpleNamespace(is_uav=False, is_payload=True), False, False),
    ],
)
def test_sim_requires_px4_sitl(launch_module, mission_spec, sim, expected):
    assert launch_module._sim_requires_px4_sitl(mission_spec, sim=sim) is expected


@pytest.mark.parametrize(
    ("model", "expected"),
    [
        ("gz_multicopter", "multicopter_0"),
        ("payload", "payload_0"),
        ("gz_payload_0", "payload_0_0"),
    ],
)
def test_gz_entity_name_for_model(launch_module, model, expected):
    assert launch_module._gz_entity_name_for_model(model) == expected


def test_camera_contract_for_uav_and_payload(launch_module):
    uav_contract = launch_module._camera_contract_for(
        SimpleNamespace(is_uav=True, is_payload=False),
        payload_name="",
        uav_name="uav_7",
    )
    assert uav_contract == {
        "vehicle_name": "uav_7",
        "camera_namespace": "/uav_7",
        "image_topic": "/uav_7/camera",
        "camera_info_topic": "/uav_7/camera_info",
        "camera_service_name": "/uav_7/camera_data",
        "input_transport": "",
        "input_raw_topic": "/uav_7/camera_source",
        "input_compressed_topic": "/uav_7/camera_source/compressed",
        "input_camera_info_topic": "/uav_7/camera_info_source",
        "rotate_degrees": 0.0,
        "preprocess_hook": "",
        "camera_info_url": "",
    }

    payload_contract = launch_module._camera_contract_for(
        SimpleNamespace(is_uav=False, is_payload=True),
        payload_name="payload_2",
        input_transport="compressed",
        rotate_degrees=180.0,
        preprocess_hook="rotate_then_crop",
        camera_info_url="file:///tmp/payload.yaml",
    )
    assert payload_contract == {
        "vehicle_name": "payload_2",
        "camera_namespace": "/payload_2",
        "image_topic": "/payload_2/camera",
        "camera_info_topic": "/payload_2/camera_info",
        "camera_service_name": "/payload_2/camera_data",
        "input_transport": "compressed",
        "input_raw_topic": "/payload_2/camera_source",
        "input_compressed_topic": "/payload_2/camera_source/compressed",
        "input_camera_info_topic": "/payload_2/camera_info_source",
        "rotate_degrees": 180.0,
        "preprocess_hook": "rotate_then_crop",
        "camera_info_url": "file:///tmp/payload.yaml",
    }


def test_camera_contract_for_requires_payload_name(launch_module):
    with pytest.raises(ValueError, match="Payload missions require an explicit payload_name"):
        launch_module._camera_contract_for(
            SimpleNamespace(is_uav=False, is_payload=True),
            payload_name="",
        )


def test_payload_camera_info_url_for_prefers_named_file(monkeypatch, tmp_path, launch_module):
    payload_share = tmp_path / "payload_share"
    (payload_share / "config").mkdir(parents=True)
    preferred = payload_share / "config" / "payload_2_camera_info.yaml"
    fallback = payload_share / "config" / "payload_0_camera_info.yaml"
    fallback.write_text("fallback", encoding="utf-8")
    preferred.write_text("preferred", encoding="utf-8")

    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda package_name: str(payload_share),
    )

    expected = f"file://{preferred}"
    assert launch_module._payload_camera_info_url_for("payload_2") == expected


def test_payload_camera_info_url_for_falls_back_to_payload_0(monkeypatch, tmp_path, launch_module):
    payload_share = tmp_path / "payload_share"
    (payload_share / "config").mkdir(parents=True)
    fallback = payload_share / "config" / "payload_0_camera_info.yaml"
    fallback.write_text("fallback", encoding="utf-8")

    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda package_name: str(payload_share),
    )

    expected = f"file://{fallback}"
    assert launch_module._payload_camera_info_url_for("payload_2") == expected


def test_payload_camera_info_url_for_missing_files_returns_empty(monkeypatch, tmp_path, launch_module):
    payload_share = tmp_path / "payload_share"
    (payload_share / "config").mkdir(parents=True)

    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda package_name: str(payload_share),
    )

    assert launch_module._payload_camera_info_url_for("payload_2") == ""


def test_build_runtime_parameters_for_uav(launch_module):
    parameters = launch_module._build_runtime_parameters(
        "/tmp/mission.yaml",
        SimpleNamespace(is_payload=False, is_uav=True),
        auto_launch=True,
        debug=1,
        servo_only=0,
        vehicle_class_name="MULTICOPTER",
        payload_name="",
        uav_name="uav_3",
        uav_camera_offsets=[0.1, 0.2, 0.3],
    )

    assert parameters == {
        "mode_map": "/tmp/mission.yaml",
        "auto_launch": True,
        "vehicle_name": "uav_3",
        "debug": True,
        "servo_only": False,
        "vehicle_class": "MULTICOPTER",
        "uav_camera_offsets": [0.1, 0.2, 0.3],
    }


def test_build_runtime_parameters_for_payload(launch_module):
    parameters = launch_module._build_runtime_parameters(
        "/tmp/payload.yaml",
        SimpleNamespace(is_payload=True, is_uav=False),
        auto_launch=False,
        debug=True,
        servo_only=True,
        vehicle_class_name=None,
        payload_name="payload_0",
        uav_name="uav_ignored",
        uav_camera_offsets=[],
    )

    assert parameters == {
        "mode_map": "/tmp/payload.yaml",
        "auto_launch": False,
        "payload_name": "payload_0",
    }


def test_build_runtime_parameters_rejects_missing_vehicle_class(launch_module):
    with pytest.raises(ValueError, match="UAV missions require a vehicle class"):
        launch_module._build_runtime_parameters(
            "/tmp/mission.yaml",
            SimpleNamespace(is_payload=False, is_uav=True),
            auto_launch=True,
            debug=False,
            servo_only=False,
            vehicle_class_name=None,
            payload_name="",
            uav_name="uav",
            uav_camera_offsets=[0.0, 0.0, 0.0],
        )


def test_build_runtime_parameters_rejects_bad_camera_offsets(launch_module):
    with pytest.raises(ValueError, match="uav_camera_offsets must have exactly 3 values"):
        launch_module._build_runtime_parameters(
            "/tmp/mission.yaml",
            SimpleNamespace(is_payload=False, is_uav=True),
            auto_launch=True,
            debug=False,
            servo_only=False,
            vehicle_class_name="MULTICOPTER",
            payload_name="",
            uav_name="uav",
            uav_camera_offsets=[0.0, 0.0],
        )
