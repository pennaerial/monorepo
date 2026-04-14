from __future__ import annotations

from typing import Any


def normalize_vehicle_name(vehicle_name: str | None) -> str:
    normalized = "" if vehicle_name is None else str(vehicle_name).strip()
    if not normalized:
        raise ValueError("scoring params must define a non-empty vehicle_name.")
    return normalized


def vehicle_px4_local_position_topic(vehicle_name: str) -> str:
    normalized = normalize_vehicle_name(vehicle_name)
    return f"/{normalized}/fmu/out/vehicle_local_position"


def uav_controllable_names(world_params: dict[str, Any]) -> list[str]:
    controllables = world_params.get("controllables", {})
    if not isinstance(controllables, dict):
        raise ValueError("world.params.controllables must be a mapping.")

    names: list[str] = []
    for name, cfg in controllables.items():
        if not isinstance(cfg, dict):
            continue
        kind = str(cfg.get("kind", "")).strip().lower()
        if kind == "uav" or "px4_airframe_id" in cfg:
            names.append(str(name))
    return names


def resolve_scoring_vehicle_name(
    scoring_params: dict[str, Any], world_params: dict[str, Any]
) -> str:
    explicit_vehicle_name = normalize_vehicle_name(
        scoring_params.get("vehicle_name")
    ) if "vehicle_name" in scoring_params else ""
    uav_names = uav_controllable_names(world_params)

    if explicit_vehicle_name:
        if not uav_names:
            raise ValueError(
                "scoring.params.vehicle_name was provided, but no UAV controllable "
                "was found in the resolved sim world."
            )
        if explicit_vehicle_name not in uav_names:
            raise ValueError(
                "scoring.params.vehicle_name must match one of the UAV "
                f"controllables. Known UAVs: {uav_names}. "
                f"Received: {explicit_vehicle_name}."
            )
        return explicit_vehicle_name

    if len(uav_names) == 1:
        return uav_names[0]

    if not uav_names:
        raise ValueError(
            "scoring.params.vehicle_name is required because no UAV controllable "
            "was found in the resolved sim world."
        )

    raise ValueError(
        "scoring.params.vehicle_name is required when multiple UAV controllables "
        f"are present. Known UAVs: {uav_names}."
    )
