from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Any

from pydantic import BaseModel

from .fleet_spec import FleetDocumentModel
from .mission_spec import MissionDocumentModel
from .schema_registry import (
    ModeRegistryDocument,
    ModeRegistryEntry,
    load_mode_registry_document,
    params_model_for_entry,
)


@lru_cache
def mode_registry_document() -> ModeRegistryDocument:
    return load_mode_registry_document()


def schema_registry_document() -> ModeRegistryDocument:
    return mode_registry_document()


@lru_cache
def mode_registry() -> dict[str, ModeRegistryEntry]:
    return dict(mode_registry_document().modes)


def mode_registry_entries(
    *, mission_target: str | None = None
) -> list[ModeRegistryEntry]:
    entries = list(mode_registry().values())
    if mission_target:
        entries = [entry for entry in entries if entry.mission_target == mission_target]
    return sorted(entries, key=lambda entry: (entry.mission_target, entry.class_path))


def mode_entry_for_class_path(class_path: str) -> ModeRegistryEntry:
    entry = mode_registry().get(class_path)
    if entry is None and "." in class_path:
        module_path, _, class_name = class_path.rpartition(".")
        fallback = mode_registry().get(module_path)
        if fallback is not None and fallback.class_name == class_name:
            entry = fallback
    if entry is None:
        raise ValueError(
            f"Mode '{class_path}' is not present in the committed schema registry."
        )
    return entry


def mode_params_model(class_path: str) -> type[BaseModel]:
    entry = mode_entry_for_class_path(class_path)
    return params_model_for_entry(entry.class_path)


def validate_mode_params(class_path: str, params: dict[str, Any]) -> dict[str, Any]:
    validated = mode_params_model(class_path).model_validate(params or {})
    dumped = validated.model_dump(mode="python")
    return {key: dumped[key] for key in (params or {}) if key in dumped}


def mission_document_schema() -> dict[str, Any]:
    return MissionDocumentModel.model_json_schema()


def fleet_document_schema() -> dict[str, Any]:
    return FleetDocumentModel.model_json_schema()


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
