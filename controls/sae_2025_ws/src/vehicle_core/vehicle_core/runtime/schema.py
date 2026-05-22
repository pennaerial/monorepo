from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Any

from pydantic import BaseModel

from .mode_paths import mode_id_from_class_path, normalize_public_mode_id
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
    return sorted(entries, key=lambda entry: (entry.mission_target, entry.mode_id))


def mode_entry_for_mode_id(mode_id: str) -> ModeRegistryEntry:
    normalized = normalize_public_mode_id(mode_id)
    entry = mode_registry().get(normalized)
    if entry is None:
        raise ValueError(
            f"Mode '{normalized}' is not present in the committed schema registry."
        )
    return entry


def mode_entry_for_class_path(class_path: str) -> ModeRegistryEntry:
    normalized = str(class_path).strip()
    if not normalized:
        raise ValueError("Mode path cannot be empty.")

    entry = mode_registry().get(normalized)
    if entry is not None:
        return entry

    public_mode_id = mode_id_from_class_path(normalized)
    entry = mode_registry().get(public_mode_id)
    if entry is not None:
        return entry

    for candidate in mode_registry().values():
        if candidate.class_path == normalized:
            return candidate

    raise ValueError(
        f"Mode '{normalized}' is not present in the committed schema registry."
    )


def mode_params_model(mode_id: str) -> type[BaseModel]:
    entry = mode_entry_for_class_path(mode_id)
    return params_model_for_entry(entry.mode_id)


def validate_mode_params(mode_id: str, params: dict[str, Any]) -> dict[str, Any]:
    validated = mode_params_model(mode_id).model_validate(params or {})
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
