from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any

from sim.utils import load_sim_parameters


def parse_json_config(raw_value: str | None, *, field_name: str) -> dict[str, Any]:
    text = "" if raw_value is None else str(raw_value).strip()
    if not text:
        return {}
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f"{field_name} must be valid JSON.") from exc
    if not isinstance(parsed, dict):
        raise ValueError(f"{field_name} must decode to a JSON object.")
    return parsed


def deep_merge_dicts(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = deepcopy(base)
    for key, value in override.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = deep_merge_dicts(merged[key], value)
        else:
            merged[key] = deepcopy(value)
    return merged


def normalize_named_records(
    records: dict[str, Any] | list[dict[str, Any]] | None,
    *,
    record_type: str,
) -> dict[str, dict[str, Any]]:
    if records is None:
        return {}

    if isinstance(records, dict):
        normalized_items = records.items()
    elif isinstance(records, list):
        normalized_items = []
        for index, entry in enumerate(records):
            if not isinstance(entry, dict):
                raise ValueError(f"{record_type} entry at index {index} must be a mapping.")
            name = str(entry.get("name", "")).strip()
            if not name:
                raise ValueError(
                    f"{record_type} entry at index {index} must define a non-empty 'name'."
                )
            payload = dict(entry)
            payload.pop("name", None)
            normalized_items.append((name, payload))
    else:
        raise ValueError(f"{record_type} records must be a mapping or a list of mappings.")

    normalized: dict[str, dict[str, Any]] = {}
    for name, payload in normalized_items:
        if not isinstance(payload, dict):
            raise ValueError(f"{record_type} '{name}' must be a mapping.")
        normalized[str(name)] = dict(payload)
    return normalized


def normalized_stage_name(mission_stage: str | None) -> str:
    stage_name = "" if mission_stage is None else str(mission_stage).strip()
    return stage_name or "base"


def _looks_like_spawnable_record(value: Any) -> bool:
    return isinstance(value, dict) and any(
        field in value
        for field in {
            "path_to_sdf",
            "position",
            "rpy",
            "kind",
            "model",
            "px4_airframe_id",
        }
    )


def normalize_stage_world_params(
    raw_world_params: dict[str, Any] | None,
) -> dict[str, Any]:
    world_params = {} if raw_world_params is None else deepcopy(raw_world_params)
    if not isinstance(world_params, dict):
        raise ValueError("Resolved stage world params must be a mapping.")

    translated: dict[str, Any] = {}
    controllables: dict[str, dict[str, Any]] = normalize_named_records(
        world_params.get("controllables", {}),
        record_type="controllable",
    )
    entities = normalize_named_records(world_params.get("entities", {}), record_type="entity")

    for key, value in world_params.items():
        if key in {"vehicle_pose", "entities", "controllables"}:
            continue
        if _looks_like_spawnable_record(value):
            raise ValueError(
                f"Spawnable stage record '{key}' must live under world.params.entities or world.params.controllables."
            )
        translated[key] = deepcopy(value)

    if entities:
        translated["entities"] = entities
    translated["controllables"] = controllables
    return translated


def resolve_stage_world(
    *,
    world_name: str,
    mission_stage: str | None = "",
    world_overrides: dict[str, Any] | None = None,
    logger=None,
) -> dict[str, Any]:
    resolved_world_name = str(world_name).strip()
    if not resolved_world_name:
        raise ValueError("Sim stage resolution requires a non-empty world_name.")

    resolved_stage = normalized_stage_name(mission_stage)
    if world_overrides is None:
        resolved_overrides: dict[str, Any] = {}
    else:
        if not isinstance(world_overrides, dict):
            raise ValueError("world_overrides must be a mapping.")
        resolved_overrides = deepcopy(world_overrides)

    sim_stage_params, sim_config_path = load_sim_parameters(
        resolved_world_name,
        logger=logger,
        competition_name=resolved_world_name,
        mission_stage=resolved_stage,
    )
    if "world" not in sim_stage_params:
        raise ValueError(f"Missing 'world' section in simulation config: {sim_config_path}")

    world_spec = deep_merge_dicts(sim_stage_params["world"], resolved_overrides)
    if not isinstance(world_spec, dict):
        raise ValueError("Resolved world spec must be a mapping.")

    world_node_name = str(world_spec.get("name", "")).strip()
    if not world_node_name:
        raise ValueError(
            f"Simulation config '{sim_config_path}' must define a non-empty world.name."
        )

    world_spec["name"] = world_node_name
    world_spec["params"] = normalize_stage_world_params(world_spec.get("params", {}))

    return {
        "world_name": resolved_world_name,
        "mission_stage": resolved_stage,
        "world": world_spec,
        "sim_stage_params": deepcopy(sim_stage_params),
        "config_path": Path(sim_config_path),
    }
