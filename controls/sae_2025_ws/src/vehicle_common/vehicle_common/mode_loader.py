from __future__ import annotations

from pydantic import BaseModel, ConfigDict, field_validator, field_serializer
from typing import cast
from vehicle_common import Mode, Vehicle
from uav.vision_nodes import VisionNode
import importlib
import pkgutil

_mode_registry: dict[str, RegisteredMode] = {}

def serialize_type(cls: type) -> str:
    return f"{cls.__module__}:{cls.__qualname__}"

def deserialize_type(path: str, base_cls: type) -> type:
    try:
        module_path, class_name = path.split(":", 1)
    except ValueError:
        raise ValueError(f"Invalid type path '{path}'. Expected 'module:ClassName'")

    module = importlib.import_module(module_path)
    obj = module
    for part in class_name.split("."):
        obj = getattr(obj, part)

    obj = cast(type, obj)
    if not issubclass(obj, base_cls):
        raise ValueError(f"{path} must be a {base_cls.__name__}")

    return obj


class RegisteredMode(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    id: str
    mode_cls: type[Mode]
    targets: list[type[Vehicle]] = []
    required_vision_nodes: list[type[VisionNode]] = []
    peer_vehicle_names: list[str] = []
    requires_camera: bool = False
    transition_labels: list[str] = []

    @field_serializer("mode_cls")
    def serialize_mode_cls(self, value: type[Mode]) -> str:
        return serialize_type(value)

    @field_serializer("targets")
    def serialize_targets(self, value: list[type[Vehicle]]) -> list[str]:
        return [serialize_type(v) for v in value]

    @field_serializer("required_vision_nodes")
    def serialize_vision_nodes(self, value: list[type[VisionNode]]) -> list[str]:
        return [serialize_type(v) for v in value]

    @field_validator("mode_cls", mode="before")
    @classmethod
    def derserialize_mode_cls(cls, path: str):
        return deserialize_type(path, type[Mode])

    @field_validator("targets", mode="before")
    @classmethod
    def derserialize_targets(cls, paths: list[str]):
        return [deserialize_type(p, type[Vehicle]) for p in paths]

    @field_validator("targets", mode="before")
    @classmethod
    def derserialize_vision_nodes(cls, paths: list[str]):
        return [deserialize_type(p, type[VisionNode]) for p in paths]



def register_mode(
    id: str,
    targets: list[type[Vehicle]],
    required_vision_nodes: list[type[VisionNode]] = [],
    peer_vehicle_names: list[str] = [],
    requires_camera: bool = False,
    transition_labels: list[str] = []
    ):
    def decorator(registered_mode_cls: type[Mode]):
        if id in _mode_registry:
            raise ValueError(
                    f"Id {id} already has a registered mode for it"
            )
        _mode_registry[id] = RegisteredMode(
            id=id,
            targets=targets,
            mode_cls=registered_mode_cls,
            required_vision_nodes=required_vision_nodes,
            peer_vehicle_names=peer_vehicle_names,
            requires_camera=requires_camera,
            transition_labels=transition_labels
        )
        return registered_mode_cls
    return decorator

def discover_modes(modules: list[str] = ["uav.modes", "payload.modes"]):
    for module in modules:
        try:
            pkg = importlib.import_module(module)
        except ImportError as e:
            print(e)
            continue

        if not hasattr(pkg, "__path__"):
            continue

        for _, name, _ in pkgutil.walk_packages(pkg.__path__, prefix=module + "."):
            try:
                importlib.import_module(name)
            except Exception as e:
                print(e)
                continue

def get_mode_cls(id: str) -> type[Mode]:
    try:
        return _mode_registry[id].mode_cls
    except KeyError:
        available = " ".join(_mode_registry.keys())

        raise KeyError(
            f"Mode '{id}' not found."
            f"Available Modes are {available}"
        )
