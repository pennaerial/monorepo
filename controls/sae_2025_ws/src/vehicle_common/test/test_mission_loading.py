import pytest
from pydantic import ValidationError

from vehicle_common.runtime.mission_loader import RuntimeMode, RuntimeMission
from vehicle_common.mode_loader import ModeRegistry
import yaml

from mock_classes import MockMode, MockVehicle, MockVisionNode, MockVerticalTakeoffMode


mode_registry = ModeRegistry.get()

mock_mode_yaml = """
mode: mock
params: {}
transitions: {}
"""


def test_mode_validate():
    mode_dict = yaml.safe_load(mock_mode_yaml)
    runtime_mode = RuntimeMode.model_validate(mode_dict)
    assert runtime_mode.mode == "mock"
    assert runtime_mode.transitions == {}
    assert runtime_mode.params == {}
    assert runtime_mode._validated_params == MockMode.Params()
    assert runtime_mode._registered == mode_registry.get_registered_mode("mock")


mock_mission_yaml = """
modes:
  takeoff:
    mode: mock.VerticalTakeoffMode
    params:
      takeoff_height: 10.0
      takeoff_method: OFFBOARD
    transitions:
      complete: loiter
  loiter:
    mode: mock.loiter
    params: {}
    transitions: {}
"""


def test_runtime_mission_parses_modes():
    mission = RuntimeMission.model_validate(yaml.safe_load(mock_mission_yaml))

    assert set(mission.modes) == {"takeoff", "loiter"}
    takeoff = mission.modes["takeoff"]
    assert takeoff.mode == "mock.VerticalTakeoffMode"
    assert takeoff.transitions == {"complete": "loiter"}
    assert takeoff._registered == mode_registry.get_registered_mode(
        "mock.VerticalTakeoffMode"
    )
    assert takeoff._validated_params == MockVerticalTakeoffMode.Params(
        takeoff_height=10.0, takeoff_method="OFFBOARD"
    )

    loiter = mission.modes["loiter"]
    assert loiter.mode == "mock.loiter"
    assert loiter.transitions == {}
    assert loiter._registered == mode_registry.get_registered_mode("mock.loiter")
    assert mission._targets == {MockVehicle}
    assert mission._vision_nodes == {MockVisionNode}
    assert mission._peer_vehicle_names == {"peer1"}
    assert mission._requires_camera is True


mock_mission_yaml_2 = """
modes:
  takeoff:
    mode: mock.VerticalTakeoffMode
    params: {}
    transitions: {}
  bare:
    mode: mock
    params: {}
    transitions: {}
"""


def test_runtime_mission_vision_nodes_empty_when_not_shared_by_all_modes():
    mission = RuntimeMission.model_validate(yaml.safe_load(mock_mission_yaml_2))
    # "mock" has no vision node requirement, so the intersection collapses to empty
    assert mission._vision_nodes == set()


mission_3 = """
modes:
  bare:
    mode: mock
    params: {}
    transitions: {}
"""


def test_runtime_mission_requires_camera_false_if_no_mode_requires_it():
    mission = RuntimeMission.model_validate(yaml.safe_load(mission_3))
    assert mission._requires_camera is False


def test_runtime_mission_forbids_extra_fields():
    with pytest.raises(ValidationError):
        RuntimeMission.model_validate({"modes": {}, "unexpected": "field"})


def test_runtime_mode_forbids_extra_fields():
    with pytest.raises(ValidationError):
        RuntimeMode.model_validate({"mode": "mock", "unexpected": "field"})


bad_mission = """
modes:
  bad:
    mode: does.not.exist
    params: {}
    transitions: {}
"""


def test_runtime_mission_unknown_mode_id_raises():
    with pytest.raises(KeyError):
        RuntimeMission.model_validate(yaml.safe_load(bad_mission))


bad_params = """
modes:
  takeoff:
    mode: mock.VerticalTakeoffMode
    params:
      takeoff_height: not-a-number
    transitions: {}
"""


def test_runtime_mission_invalid_params_raises():
    with pytest.raises(ValidationError):
        RuntimeMission.model_validate(yaml.safe_load(bad_params))


def test_runtime_mission_requires_at_least_one_mode():
    with pytest.raises(TypeError):
        RuntimeMission.model_validate({"modes": {}})
