#!/usr/bin/env python3
"""Dump mode/fleet/mission schema data as JSON, for the mission and fleet
YAML editors in the TypeScript dashboard.

Not a server -- invoked on demand by pi-agent (which sources the deployed
release's ROS2 environment first), e.g.:

    source /opt/ros/jazzy/setup.bash
    source <deploy_root>/current/install/setup.bash
    python3 schema_export.py --root <deploy_root>/current

Requires a sourced ROS2 + colcon environment where `vehicle_common` (and the
`uav`/`payload` packages whose modes it discovers) are importable.

Ported from the old (deleted) integration dashboard's `backend/schema.py`,
adapted to the current `vehicle_common` surface:

- `vehicle_common.runtime.fleet_spec` is unchanged, so `fleet_schema()`
  ports close to 1:1.
- `vehicle_common.runtime.mission_spec` and `vehicle_common.runtime.schema`
  (which the old code used for mission/fleet catalog file discovery) no
  longer exist -- mission/fleet YAML files are now colcon package share
  data, installed under `install/<pkg>/share/<pkg>/{missions,fleets}/*.yaml`
  (see `uav/setup.py`, `payload/setup.py`). `_available_mission_names`/
  `_available_fleet_names` below re-glob those installed locations directly
  instead of the old repo-relative source-tree helpers.
- `vehicle_common.mode_loader.ModeRegistry.get()` replaces the old static
  `mode_registry.json` file entirely (see the module docstring history in
  git blame). Its `RegisteredMode` entries carry no `mission_target`
  (a single string) or pre-baked `params_schema` the way the old compat
  layer's entries did -- `targets: list[type[Vehicle]]` and a live
  `mode_cls.get_params_cls().model_json_schema()` call replace them. A
  best-effort uav/payload target label is derived from each target
  Vehicle class's top-level package name (`uav.*` -> "uav",
  `payload.*` -> "payload"), since that label isn't carried as data.
  There's also no `description` field left on `RegisteredMode` -- each
  mode class's own docstring is used instead, which is a real (if less
  structured) substitute already present on every mode class.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


def _required_names(schema: dict[str, Any]) -> set[str]:
    return set(schema.get("required", []))


def _schema_field(
    name: str,
    property_schema: dict[str, Any],
    *,
    required: bool,
) -> dict[str, Any]:
    schema_type = "object"
    annotation = "object"
    nullable = False
    choices: list[Any] = []

    type_name_map = {
        "boolean": "bool",
        "integer": "int",
        "number": "float",
        "string": "str",
        "array": "list",
        "object": "dict",
    }

    if "enum" in property_schema:
        schema_type = "enum"
        annotation = "enum"
        choices = list(property_schema["enum"])
    elif "prefixItems" in property_schema:
        schema_type = "tuple"
        annotation = "tuple"
    elif "$ref" in property_schema:
        annotation = property_schema["$ref"].rsplit("/", 1)[-1]
    else:
        schema_type = str(property_schema.get("type", schema_type))
        annotation = schema_type

    variants = property_schema.get("anyOf") or property_schema.get("oneOf") or []
    if variants:
        non_null = []
        for variant in variants:
            if variant.get("type") == "null":
                nullable = True
            else:
                non_null.append(variant)
        if len(non_null) == 1:
            variant = non_null[0]
            if "enum" in variant:
                schema_type = "enum"
                annotation = "enum"
                choices = list(variant["enum"])
            elif "prefixItems" in variant:
                # An Optional[tuple[...]] field (e.g. FleetDefaultsModel's
                # camera_mount_offsets: tuple[float, float, float] | None)
                # wraps the tuple schema inside anyOf alongside {type: null}
                # -- prefixItems only ever appears on the top-level schema
                # for a non-Optional tuple, so without this check every
                # Optional tuple field is misclassified as a plain "list".
                schema_type = "tuple"
                annotation = "tuple"
            elif "$ref" in variant:
                annotation = variant["$ref"].rsplit("/", 1)[-1]
            else:
                schema_type = str(variant.get("type", schema_type))
                annotation = schema_type
        elif non_null:
            schema_type = "union"
            annotation = "union"

    schema_type = type_name_map.get(schema_type, schema_type)
    annotation = type_name_map.get(annotation, annotation)

    default_kind = "missing"
    default = None
    if "default" in property_schema:
        default = property_schema["default"]
        default_kind = "none" if default is None else "value"

    return {
        "name": name,
        "schemaType": schema_type,
        "annotation": annotation,
        "required": required,
        "default": default,
        "defaultKind": default_kind,
        "nullable": nullable,
        "choices": choices,
        "description": property_schema.get("description"),
    }


def _fields_from_model(schema: dict[str, Any], calibration_choices: list[str]) -> list[dict[str, Any]]:
    properties = schema.get("properties", {})
    required = _required_names(schema)
    fields = [
        _schema_field(name, property_schema, required=name in required) for name, property_schema in properties.items()
    ]
    if calibration_choices:
        for field in fields:
            if field["name"] == "camera_calibration_file":
                field["choices"] = calibration_choices
    return fields


def _available_camera_calibration_files(root: Path) -> list[str]:
    calibrations_dir = root / "install" / "uav" / "share" / "uav" / "config" / "camera_calibrations"
    if not calibrations_dir.is_dir():
        return []
    filenames = {path.name for pattern in ("*.yaml", "*.yml") for path in calibrations_dir.glob(pattern) if path.is_file()}
    return sorted(filenames)


def fleet_schema(root: Path) -> dict[str, Any]:
    from vehicle_common.runtime.fleet_spec import (
        FleetDefaultsModel,
        FleetVehicleModel,
        HardwareBackendModel,
        SimBackendModel,
    )

    calibration_choices = _available_camera_calibration_files(root)

    sim_backend_schema = SimBackendModel.model_json_schema()
    hardware_backend_schema = HardwareBackendModel.model_json_schema()
    defaults_schema = FleetDefaultsModel.model_json_schema()
    vehicle_schema = FleetVehicleModel.model_json_schema()

    sim_backend_fields = {f["name"]: f for f in _fields_from_model(sim_backend_schema, calibration_choices)}
    hardware_backend_fields = {f["name"]: f for f in _fields_from_model(hardware_backend_schema, calibration_choices)}

    backend_fields: list[dict[str, Any]] = []
    seen_backend: set[str] = set()
    for source in (sim_backend_fields, hardware_backend_fields):
        for name, field in source.items():
            if name in seen_backend:
                continue
            backend_fields.append(field)
            seen_backend.add(name)

    defaults_fields = _fields_from_model(defaults_schema, calibration_choices)
    vehicle_fields = {f["name"]: f for f in _fields_from_model(vehicle_schema, calibration_choices)}

    def section_fields(names: tuple[str, ...]) -> list[dict[str, Any]]:
        return [vehicle_fields[name] for name in names if name in vehicle_fields]

    return {
        "backendKinds": ["sim", "hardware", "real"],
        "sections": [
            {"name": "backend", "fields": backend_fields},
            {"name": "defaults", "fields": defaults_fields},
            {"name": "vehicle.common", "fields": section_fields(("name", "mission", "mission_path", "kind"))},
            {"name": "vehicle.sim", "fields": section_fields(("controllable", "px4_instance"))},
            {
                "name": "vehicle.hardware",
                "fields": section_fields(
                    (
                        "px4_airframe_id",
                        "px4_namespace",
                        "payload_controller",
                        "network_role",
                        "allowed_ap_vehicles",
                        "ap_selection",
                    )
                ),
            },
        ],
    }


def _target_label_for_vehicle_cls(vehicle_cls: type) -> str:
    top_level_package = (vehicle_cls.__module__ or "").split(".")[0]
    if top_level_package in ("uav", "payload"):
        return top_level_package
    return ""


def mode_registry() -> dict[str, list[dict[str, Any]]]:
    from vehicle_common.mode_loader import ModeRegistry

    registry = ModeRegistry.get()
    grouped: dict[str, list[dict[str, Any]]] = {}

    for mode_id, registered in registry.modes.items():
        targets = [_target_label_for_vehicle_cls(t) for t in registered.targets]
        target = next((t for t in targets if t), "")
        params_schema = registered.mode_cls.get_params_cls().model_json_schema()
        entry = {
            "name": registered.mode_cls.__name__,
            "mode": mode_id,
            "classPath": f"{registered.mode_cls.__module__}:{registered.mode_cls.__qualname__}",
            "target": target,
            "description": (registered.mode_cls.__doc__ or "").strip() or None,
            "requiredVisionNodes": [v.__name__ for v in registered.required_vision_nodes],
            "transitionLabels": list(registered.transition_labels),
            "params": _fields_from_model(params_schema, []),
        }
        grouped.setdefault(target or "unknown", []).append(entry)

    return grouped


def _available_mission_files(root: Path) -> list[dict[str, str]]:
    missions: list[dict[str, str]] = []
    for pkg, target in (("uav", "uav"), ("payload", "payload")):
        missions_dir = root / "install" / pkg / "share" / pkg / "missions"
        if not missions_dir.is_dir():
            continue
        for path in sorted(missions_dir.glob("*.yaml")):
            missions.append({"name": path.stem, "target": target})
    return missions


def _available_fleet_files(root: Path) -> list[str]:
    fleets_dir = root / "install" / "uav" / "share" / "uav" / "fleets"
    if not fleets_dir.is_dir():
        return []
    return sorted(path.stem for path in fleets_dir.glob("*.yaml"))


def schema_index(root: Path) -> dict[str, Any]:
    return {
        "modes": mode_registry(),
        "fleet": fleet_schema(root),
        "missions": _available_mission_files(root),
        "availableFleets": _available_fleet_files(root),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        default=".",
        help="Deploy release root containing install/ (defaults to the current directory).",
    )
    args = parser.parse_args()

    try:
        from vehicle_common.mode_loader import ModeRegistry  # noqa: F401
    except ImportError as exc:
        print(
            f"Could not import vehicle_common.mode_loader: {exc}\n"
            "Run this from a sourced ROS2 + colcon environment where "
            "vehicle_common is built and on PYTHONPATH.",
            file=sys.stderr,
        )
        return 1

    print(json.dumps(schema_index(Path(args.root)), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
