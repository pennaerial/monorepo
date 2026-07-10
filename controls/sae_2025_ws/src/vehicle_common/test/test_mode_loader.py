import pytest

from vehicle_common.mode_loader import (
    serialize_type,
    deserialize_type,
    RegisteredMode,
    register_mode,
    get_registered_mode,
    ModeRegistry
)


from vehicle_common import Mode, Vehicle
from uav.vision_nodes import VisionNode
import json

class MockVehicle(Vehicle):
    pass


class MockVisionNode(VisionNode):
    pass


@register_mode(id="mock", targets=[MockVehicle])
class MockMode(Mode):
    pass

def test_serialize_types():
    result = serialize_type(MockMode)
    assert result == (f"{MockMode.__module__}:{MockMode.__qualname__}")

def test_deserialize_type():
    path = serialize_type(MockMode)
    result = deserialize_type(path, Mode)
    assert result is MockMode

def test_deserialize_raises_wrong_base():
    path = serialize_type(MockVehicle)
    with pytest.raises(ValueError):
        deserialize_type(path, VisionNode)

def test_registered_mode_creation():
    mode = RegisteredMode(
        id="test",
        mode_cls=MockMode,
        targets=[MockVehicle],
        required_vision_nodes=[MockVisionNode],
    )

    assert mode.id == "test"
    assert mode.mode_cls is MockMode
    assert mode.targets == [MockVehicle]
    assert mode.required_vision_nodes == [MockVisionNode]

def test_get_registered_mode():
    m = get_registered_mode("mock")
    assert m.id == "mock"
    assert m.mode_cls == MockMode
    assert m.required_vision_nodes == []
    assert m.targets == [MockVehicle]
    assert m.peer_vehicle_names == []
    assert not m.requires_camera


def test_registered_mode_to_json():
    mode = RegisteredMode(
        id="test",
        mode_cls=MockMode,
        targets=[MockVehicle],
        required_vision_nodes=[MockVisionNode],
    )

    json_str = mode.model_dump_json(indent=4)
    data = json.loads(json_str)

    assert data["id"] == "test"

    assert data["mode_cls"] == serialize_type(MockMode)

    assert data["targets"] == [
        serialize_type(MockVehicle)
    ]

    assert data["required_vision_nodes"] == [
        serialize_type(MockVisionNode)
    ]

    assert data["peer_vehicle_names"] == []
    assert data["requires_camera"] is False
    assert data["transition_labels"] == []


def test_registered_mode_from_json():
    mode = RegisteredMode(
        id="test",
        mode_cls=MockMode,
        targets=[MockVehicle],
        required_vision_nodes=[MockVisionNode],
    )

    json_str = mode.model_dump_json()

    loaded = RegisteredMode.model_validate_json(json_str)

    assert loaded.id == mode.id
    assert loaded.mode_cls is MockMode
    assert loaded.targets == [MockVehicle]
    assert loaded.required_vision_nodes == [MockVisionNode]


def test_registered_mode_json_round_trip():
    original = RegisteredMode(
        id="round_trip",
        mode_cls=MockMode,
        targets=[MockVehicle],
        required_vision_nodes=[MockVisionNode],
        peer_vehicle_names=["vehicle1"],
        requires_camera=True,
        transition_labels=["done"],
    )

    loaded = RegisteredMode.model_validate_json(
        original.model_dump_json()
    )

    assert loaded.model_dump() == original.model_dump()


def test_registered_mode_from_json_invalid_mode_type():
    data = {
        "id": "bad",
        "mode_cls": serialize_type(MockVehicle),
        "targets": [],
        "required_vision_nodes": [],
    }

    with pytest.raises(ValueError):
        RegisteredMode.model_validate(data)


def test_registered_mode_from_json_invalid_path():
    data = {
        "id": "bad",
        "mode_cls": "not.a.real.module:FakeClass",
        "targets": [],
        "required_vision_nodes": [],
    }

    with pytest.raises((ValueError, ModuleNotFoundError)):
        RegisteredMode.model_validate(data)

def test_mode_registry_json_round_trip():
    original_json = """
    {
        "modes": {
            "mock": {
                "id": "mock",
                "mode_cls": "test_mode_loader:MockMode",
                "targets": [
                    "test_mode_loader:MockVehicle"
                ],
                "required_vision_nodes": [],
                "peer_vehicle_names": [],
                "requires_camera": false,
                "transition_labels": []
            }
        }
    }
    """

    registry = ModeRegistry.model_validate_json(original_json)

    assert "mock" in registry.modes

    mode = registry.modes["mock"]

    assert mode.mode_cls is MockMode
    assert mode.targets == [MockVehicle]

    output_json = registry.model_dump_json(indent=4)

    # Compare parsed JSON, not raw strings
    assert json.loads(output_json) == json.loads(original_json)
