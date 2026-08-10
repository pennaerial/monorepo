from __future__ import annotations
from pathlib import Path
from typing import ClassVar
import importlib
import pkgutil

from pydantic import BaseModel, ConfigDict, field_validator, field_serializer

from vehicle_common.mode import Mode
from vehicle_common.vehicle import Vehicle
from vehicle_common.base import VisionNode  # don't want to depend on uav package
from vehicle_common.utils import serialize_type, deserialize_type


class ParamsBase(BaseModel):
    """Wrapper around BaseModel that every Mode's Params Type should inherit from"""

    # doing this allows a default "Empty" instantiation since BaseModel() cannot be instantiated.


class RegisteredMode(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    id: str
    mode_cls: type[Mode]
    params_cls: type[ParamsBase] = ParamsBase
    targets: list[type[Vehicle]] = []
    required_vision_nodes: list[type[VisionNode]] = []
    peer_vehicle_names: list[str] = []
    requires_camera: bool = False
    transition_labels: list[str] = []

    # define field serializers if we want future static inspection w/ JSONs
    @field_serializer("mode_cls")
    def serialize_mode_cls(self, value: type[Mode]) -> str:
        return serialize_type(value)

    @field_serializer("params_cls")
    def serialize_params_cls(self, value: type[ParamsBase]) -> str:
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

    @field_validator("params_cls", mode="before")
    @classmethod
    def deserialize_params_cls(cls, path) -> type[ParamsBase]:
        if isinstance(path, str):
            return deserialize_type(path, ParamsBase)
        if not isinstance(path, type) or not issubclass(path, ParamsBase):
            raise ValueError(f"{path} must be a {ParamsBase.__name__}")
        return path

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

    def __str__(self) -> str:
        return "\n".join(
            [
                f"Mode: {self.id}",
                f"  class:       {self.mode_cls.__name__}",
                f"  params:      {self.params_cls.__name__}",
                f"  targets:     {names(self.targets)}",
                f"  vision:      {names(self.required_vision_nodes)}",
                f"  peers:       {', '.join(self.peer_vehicle_names) or '—'}",
                f"  camera:      {'True' if self.requires_camera else 'No'}",
                f"  transitions: {', '.join(self.transition_labels) or '—'}",
            ]
        )


def names(types):
    return ", ".join(t.__name__ for t in types) or "—"


class ModeRegistry(BaseModel):
    modes: dict[str, RegisteredMode]

    _pending: ClassVar[list[RegisteredMode]] = []
    _instance: ClassVar[ModeRegistry | None] = None

    @classmethod
    def get(cls) -> ModeRegistry:
        if cls._instance is None:
            cls._instance = ModeRegistry(modes=dict())
            cls._instance.discover_modes()
        return cls._instance

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
        classes populates them into the _pending list, which this then commits into `self.modes`.

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

        for reg in ModeRegistry._pending:
            if reg.id in self.modes:
                raise ValueError(f"Id {reg.id} already has a registered mode for it")
            self.modes[reg.id] = reg


def register_mode(
    id: str,
    targets: list[type[Vehicle]],
    params_cls: type[ParamsBase] = ParamsBase,
    required_vision_nodes: list[type[VisionNode]] = [],
    peer_vehicle_names: list[str] = [],
    requires_camera: bool = False,
    transition_labels: list[str] = [],
):
    """Class decorator that registers a Mode in the Mode Registry.
    Class decorators can only run once, so the registered mode is put into a global
    pending list in ModeRegistry. A ModeRegistry instance only picks it up once its
    own discover_modes() runs.

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
    """

    def decorator(registered_mode_cls: type[Mode]):
        ModeRegistry._pending.append(
            RegisteredMode(
                id=id,
                targets=targets,
                mode_cls=registered_mode_cls,
                params_cls=params_cls,
                required_vision_nodes=required_vision_nodes,
                peer_vehicle_names=peer_vehicle_names,
                requires_camera=requires_camera,
                transition_labels=transition_labels,
            )
        )
        return registered_mode_cls

    return decorator
