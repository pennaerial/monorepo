from __future__ import annotations
from pathlib import Path

from pydantic import BaseModel, ConfigDict, field_validator, field_serializer
from typing import cast
from vehicle_common.mode import Mode
from vehicle_common.vehicle import Vehicle
from vehicle_common.base import VisionNode  # don't want to depend on uav package
import importlib
import pkgutil


def serialize_type(cls: type) -> str:
    return f"{cls.__module__}:{cls.__qualname__}"


def deserialize_type(path: str, base_cls: type) -> type:
    """Resolve a serialized "module:QualName" path back into a class object.

    Tries to instantiate the class after importing the module. The resolved
    object must be a subclass of `base_cls`.

    Args:
        path: Serialized type reference in "module.path:QualName" form,
            as produced by `serialize_type`.
        base_cls: The class the resolved type must be a subclass of.

    Returns:
        The resolved class object.

    Raises:
        ValueError: If `path` isn't in "module:QualName" form, or the
            resolved object is not a subclass of `base_cls`.
    """

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

    # Each field_validator needs to support both actual types and strings so
    # manual instantiation and loading from json is valid

    @field_validator("mode_cls", mode="before")
    @classmethod
    def deserialize_mode_cls(cls, path) -> type[Mode]:
        if isinstance(path, str):
            return deserialize_type(path, Mode)
        return path  # mode type

    @field_validator("targets", mode="before")
    @classmethod
    def deserialize_targets(cls, paths) -> list[type[Vehicle]]:
        if all(isinstance(p, str) for p in paths):
            return [deserialize_type(p, Vehicle) for p in paths]
        return paths  # Vehicle types

    @field_validator("required_vision_nodes", mode="before")
    @classmethod
    def deserialize_vision_nodes(cls, paths) -> list[type[VisionNode]]:
        if all(isinstance(p, str) for p in paths):
            return [deserialize_type(p, VisionNode) for p in paths]
        return paths  # vision_node types


class ModeRegistry(BaseModel):
    modes: dict[str, RegisteredMode]

    def get_registered_mode(self, id: str) -> RegisteredMode:
        try:
            return self.modes[id]
        except KeyError:
            available = " ".join(self.modes.keys())
            raise KeyError(f"Mode '{id}' not found.Available Modes are {available}")

    def write_json(self, path: Path) -> None:
        path.write_text(self.model_dump_json(indent=2))

    def discover_modes(self, modules: list[str] = ["uav.modes", "payload.modes"]):
        """Import every submodule under the given packages to trigger mode registration.

        Recursively imports the passed in modules. Importing a file with `@register_mode(...)`-decorated
        classes populates them into the Mode Registry

        Args:
            modules: Dotted package paths to walk and import submodules from.
                Defaults to the UAV and payload mode packages.
        """

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


mode_registry = ModeRegistry(modes=dict())


def register_mode(
    id: str,
    targets: list[type[Vehicle]],
    required_vision_nodes: list[type[VisionNode]] = [],
    peer_vehicle_names: list[str] = [],
    requires_camera: bool = False,
    transition_labels: list[str] = [],
):
    """Decorator that registers a Mode in the Mode Registry.

    Args:
        id: used as main key to store and retrieve RegisteredMode objects
        targets: Vehicle types this mode is valid for
        required_vision_nodes: Vision node types that must be running for this mode to be usable.
        peer_vehicle_names: Names of peer vehicles this mode depends on
        requires_camera: Whether this mode requires a camera to operate.
        transition_labels: Labels describing valid transitions into/out
            of this mode, used by whatever drives mode switching.

    Returns:
        A decorator that registers the given `Mode` subclass and
        returns it unchanged.

    Raises:
        ValueError: If `id` is already exists
    """

    def decorator(registered_mode_cls: type[Mode]):
        if id in mode_registry.modes:
            raise ValueError(f"Id {id} already has a registered mode for it")
        mode_registry.modes[id] = RegisteredMode(
            id=id,
            targets=targets,
            mode_cls=registered_mode_cls,
            required_vision_nodes=required_vision_nodes,
            peer_vehicle_names=peer_vehicle_names,
            requires_camera=requires_camera,
            transition_labels=transition_labels,
        )
        return registered_mode_cls

    return decorator
