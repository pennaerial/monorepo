from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
UAV_PACKAGE_ROOT = PACKAGE_ROOT.parent / "uav"

for path in (PACKAGE_ROOT, UAV_PACKAGE_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

for module_name in [
    "pydantic",
    "rclpy",
    "px4_msgs",
    "sensor_msgs",
    "cv_bridge",
    "payload_interfaces",
    "uav_interfaces",
    "cv2",
]:
    pytest.importorskip(module_name)

if "pydantic" not in sys.modules:

    class _FieldInfo:
        def __init__(self, *, default=..., default_factory=None, alias=None):
            self.default = default
            self.default_factory = default_factory
            self.alias = alias

    class _FakeBaseModel:
        def __init__(self, **data):
            annotations = {}
            for base in reversed(self.__class__.__mro__):
                annotations.update(getattr(base, "__annotations__", {}))
            for name, annotation in annotations.items():
                if name.startswith("_"):
                    continue
                if name in data:
                    value = data.pop(name)
                else:
                    class_value = getattr(self.__class__, name, _FieldInfo())
                    if isinstance(class_value, _FieldInfo):
                        if class_value.default_factory is not None:
                            value = class_value.default_factory()
                        elif class_value.default is not ...:
                            value = class_value.default
                        else:
                            raise TypeError(f"Missing required field '{name}'.")
                    else:
                        value = class_value
                setattr(self, name, value)
            for name, value in data.items():
                setattr(self, name, value)

        @classmethod
        def model_validate(cls, value):
            if isinstance(value, cls):
                return value
            if isinstance(value, dict):
                return cls(**value)
            if hasattr(value, "__dict__"):
                return cls(**value.__dict__)
            raise TypeError(f"Cannot validate {value!r} as {cls.__name__}.")

        def model_dump(self, mode: str = "python"):
            def _dump(value):
                if isinstance(value, _FakeBaseModel):
                    return value.model_dump(mode=mode)
                if isinstance(value, list):
                    return [_dump(item) for item in value]
                if isinstance(value, tuple):
                    return [_dump(item) for item in value]
                if isinstance(value, dict):
                    return {key: _dump(item) for key, item in value.items()}
                return value

            return {
                key: _dump(value)
                for key, value in self.__dict__.items()
                if not key.startswith("_")
            }

        @classmethod
        def model_json_schema(cls):
            return {"title": cls.__name__, "type": "object"}

    def _fake_field(*, default=..., default_factory=None, alias=None):
        return _FieldInfo(default=default, default_factory=default_factory, alias=alias)

    def _fake_config_dict(**kwargs):
        return dict(kwargs)

    def _fake_create_model(name, __config__=None, __module__=None, **field_defs):
        namespace = {"__module__": __module__ or __name__}
        annotations = {}
        for field_name, (annotation, default) in field_defs.items():
            annotations[field_name] = annotation
            namespace[field_name] = _FieldInfo(default=default)
        namespace["__annotations__"] = annotations
        return type(name, (_FakeBaseModel,), namespace)

    pydantic_stub = types.ModuleType("pydantic")
    pydantic_stub.BaseModel = _FakeBaseModel
    pydantic_stub.ConfigDict = _fake_config_dict
    pydantic_stub.Field = _fake_field
    pydantic_stub.create_model = _fake_create_model
    sys.modules["pydantic"] = pydantic_stub

backend_pkg = types.ModuleType("backend")
backend_pkg.__path__ = [str(PACKAGE_ROOT / "backend")]
sys.modules.setdefault("backend", backend_pkg)

from backend.schema import (  # noqa: E402
    fleet_schema,
    mission_catalog,
    mission_schema_for_name,
    mode_registry,
    schema_index,
)


def test_schema_index_exposes_missions_fleet_and_modes():
    payload = schema_index().model_dump()

    assert payload["success"] is True
    assert "hover" in payload["missions"]["available_missions"]
    assert "payload_retreat" in payload["missions"]["available_missions"]
    assert "example_fleet" in payload["fleet"]["available_fleets"]
    assert "sae_payload_pickup" in payload["fleet"]["available_fleets"]
    assert set(payload["modes"]["targets"]) == {"uav", "payload"}

    backend_section = next(
        section
        for section in payload["fleet"]["sections"]
        if section["name"] == "backend"
    )
    assert backend_section["applies_to"] == ["sim", "hardware"]

    vehicle_sim = next(
        section
        for section in payload["fleet"]["sections"]
        if section["name"] == "vehicle.sim"
    )
    vehicle_hardware = next(
        section
        for section in payload["fleet"]["sections"]
        if section["name"] == "vehicle.hardware"
    )
    assert {field["name"] for field in vehicle_sim["fields"]} >= {
        "controllable",
        "px4_instance",
    }
    assert {field["name"] for field in vehicle_hardware["fields"]} >= {
        "px4_airframe_id",
        "px4_namespace",
        "payload_controller",
    }


def test_schema_mission_exposes_constructor_metadata():
    payload = mission_schema_for_name("hover").model_dump()

    assert payload["success"] is True
    assert payload["name"] == "hover"
    assert payload["target"] == "uav"
    assert payload["start_mode"] == "start"
    start_mode = next(mode for mode in payload["modes"] if mode["name"] == "start")
    takeoff_type = next(
        field
        for field in start_mode["metadata"]["params"]
        if field["name"] == "takeoff_type"
    )
    assert takeoff_type["schema_type"] == "enum"
    assert takeoff_type["default"] == "vertical"
    assert set(takeoff_type["choices"]) == {"vertical", "horizontal"}
    assert start_mode["metadata"]["transition_labels"] == ["complete"]
    assert payload["document_schema"]["type"] == "object"

    gps_mode = next(mode for mode in payload["modes"] if mode["name"] == "GPS")
    coordinates = next(
        field
        for field in gps_mode["metadata"]["params"]
        if field["name"] == "coordinates"
    )
    assert coordinates["schema_type"] == "list"
    assert coordinates["required"] is True


def test_schema_mode_registry_can_filter_by_target():
    payload = mode_registry(target="payload").model_dump()

    assert payload["success"] is True
    assert set(payload["targets"]) == {"payload"}
    assert any(
        item["name"] == "PayloadRetreatMode" for item in payload["targets"]["payload"]
    )


def test_schema_mission_catalog_can_filter_by_target():
    payload = mission_catalog(target="payload").model_dump()

    assert payload["success"] is True
    assert all(mission["target"] == "payload" for mission in payload["missions"])
    assert "payload_retreat" in payload["available_missions"]


def test_fleet_schema_exposes_section_shapes():
    payload = fleet_schema().model_dump()

    assert payload["success"] is True
    assert payload["backend_kinds"] == ["sim", "hardware"]
    assert "airframe" in payload["excluded_keys"]
    assert any(section["name"] == "defaults" for section in payload["sections"])
    assert payload["document_schema"]["type"] == "object"
