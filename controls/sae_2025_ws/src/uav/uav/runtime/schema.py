from __future__ import annotations

import importlib
import inspect
import pkgutil
from functools import lru_cache
from pathlib import Path
from typing import Any, get_type_hints

from pydantic import BaseModel, ConfigDict, create_model

from uav.modes.Mode import Mode


class ModeRegistryEntry(BaseModel):
    model_config = ConfigDict(extra="forbid")

    class_path: str
    module_path: str
    class_name: str
    display_name: str
    description: str
    mission_target: str
    required_vision_nodes: tuple[str, ...]
    transition_labels: tuple[str, ...]
    params_schema: dict[str, Any]


def _canonical_mode_path(mode_class: type[Mode]) -> str:
    module_path = mode_class.__module__
    if module_path.rsplit(".", 1)[-1] == mode_class.__name__:
        return module_path
    return f"{module_path}.{mode_class.__name__}"


def _doc_summary(obj: object) -> str:
    doc = inspect.getdoc(obj) or ""
    return doc.strip().split("\n", 1)[0] if doc.strip() else ""


def _iter_mode_classes() -> list[type[Mode]]:
    import uav.modes as mode_package

    discovered: dict[str, type[Mode]] = {}
    for module_info in pkgutil.walk_packages(
        mode_package.__path__, prefix=f"{mode_package.__name__}."
    ):
        module = importlib.import_module(module_info.name)
        for value in vars(module).values():
            if (
                isinstance(value, type)
                and issubclass(value, Mode)
                and value is not Mode
                and value.__module__ == module.__name__
            ):
                discovered[_canonical_mode_path(value)] = value
    return [discovered[key] for key in sorted(discovered)]


def _normalized_annotation(
    mode_class: type[Mode], *, name: str, annotation: object, default: object
) -> object:
    if annotation in (inspect.Parameter.empty, None, Any, object):
        if default not in (inspect.Parameter.empty, None) and type(default) in {
            bool,
            int,
            float,
            str,
        }:
            return type(default)
        raise TypeError(
            f"Mode '{_canonical_mode_path(mode_class)}' parameter '{name}' requires an explicit schema-grade type annotation."
        )

    if annotation in {dict, list, tuple, set}:
        raise TypeError(
            f"Mode '{_canonical_mode_path(mode_class)}' parameter '{name}' must use a typed collection annotation, not bare '{annotation.__name__}'."
        )

    origin = getattr(annotation, "__origin__", None)
    args = getattr(annotation, "__args__", ())
    if origin in {dict, list, tuple, set} and not args:
        raise TypeError(
            f"Mode '{_canonical_mode_path(mode_class)}' parameter '{name}' must declare collection element types."
        )
    return annotation


@lru_cache
def mode_params_model(mode_class: type[Mode]) -> type[BaseModel]:
    signature = inspect.signature(mode_class.__init__)
    type_hints = get_type_hints(mode_class.__init__)
    field_definitions: dict[str, tuple[object, object]] = {}

    for name, parameter in signature.parameters.items():
        if name in {"self", "node", "vehicle"}:
            continue
        if parameter.kind in {
            inspect.Parameter.VAR_POSITIONAL,
            inspect.Parameter.VAR_KEYWORD,
        }:
            continue
        annotation = _normalized_annotation(
            mode_class,
            name=name,
            annotation=type_hints.get(name, parameter.annotation),
            default=parameter.default,
        )
        default = (
            ... if parameter.default is inspect.Parameter.empty else parameter.default
        )
        field_definitions[name] = (annotation, default)

    return create_model(
        f"{mode_class.__name__}Params",
        __config__=ConfigDict(extra="forbid"),
        __module__=mode_class.__module__,
        **field_definitions,
    )


def validate_mode_params(
    mode_class: type[Mode], params: dict[str, Any]
) -> dict[str, Any]:
    validated = mode_params_model(mode_class).model_validate(params or {})
    dumped = validated.model_dump(mode="python")
    return {key: dumped[key] for key in (params or {}) if key in dumped}


def mode_registry_entry(mode_class: type[Mode]) -> ModeRegistryEntry:
    mission_target = getattr(mode_class, "mission_target", None)
    if mission_target not in {"uav", "payload"}:
        raise ValueError(
            f"Mode '{_canonical_mode_path(mode_class)}' must declare mission_target as 'uav' or 'payload'."
        )

    return ModeRegistryEntry(
        class_path=_canonical_mode_path(mode_class),
        module_path=mode_class.__module__,
        class_name=mode_class.__name__,
        display_name=mode_class.__name__,
        description=_doc_summary(mode_class),
        mission_target=mission_target,
        required_vision_nodes=mode_class.required_vision_node_names(),
        transition_labels=mode_class.declared_transition_labels(),
        params_schema=mode_params_model(mode_class).model_json_schema(),
    )


@lru_cache
def mode_registry() -> dict[str, ModeRegistryEntry]:
    return {
        entry.class_path: entry
        for entry in (
            mode_registry_entry(mode_class) for mode_class in _iter_mode_classes()
        )
    }


def mode_registry_entries(
    *, mission_target: str | None = None
) -> list[ModeRegistryEntry]:
    entries = list(mode_registry().values())
    if mission_target:
        entries = [entry for entry in entries if entry.mission_target == mission_target]
    return sorted(entries, key=lambda entry: (entry.mission_target, entry.class_path))


def mode_entry_for_class_path(class_path: str) -> ModeRegistryEntry:
    from .mission_spec import load_mode_class

    mode_class = load_mode_class(class_path)
    return mode_registry_entry(mode_class)


def repo_mission_paths() -> list[Path]:
    from .mission_spec import mission_root

    return sorted(mission_root().glob("*.yaml"))


def repo_fleet_paths() -> list[Path]:
    try:
        from ament_index_python.packages import get_package_share_directory

        return sorted(Path(get_package_share_directory("uav")).glob("fleets/*.yaml"))
    except Exception:
        package_root = Path(__file__).resolve().parent.parent
        return sorted((package_root / "fleets").glob("*.yaml"))
