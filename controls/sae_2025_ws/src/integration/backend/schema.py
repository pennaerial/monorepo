from __future__ import annotations

from pathlib import Path
from typing import Any

from uav.runtime.fleet_spec import (
    FleetDefaultsModel,
    FleetDocumentModel,
    FleetVehicleModel,
    HardwareBackendModel,
    SimBackendModel,
)
from uav.runtime.mission_spec import (
    MissionDocumentModel,
    mission_path_for_name,
    mission_root,
)
from uav.runtime.schema import repo_fleet_paths

from .models import (
    FleetSchemaResponse,
    FleetSectionSchemaResponse,
    ModeMetadataResponse,
    ModeRegistryResponse,
    MissionCatalogResponse,
    MissionModeResponse,
    MissionSchemaResponse,
    SchemaFieldResponse,
    SchemaIndexResponse,
)
from .mission_compat import (
    load_mission_spec_compat,
    internal_mode_path,
    mode_entry_for_class_path,
    mode_registry_entries,
    public_mode_id,
)

_FLEET_BACKEND_KINDS = ["sim", "hardware", "real"]
_FLEET_EXCLUDED_KEYS = [
    "airframe",
    "custom_airframe_model",
    "model",
    "sim",
    "px4_instance",
]
_CAMERA_CALIBRATIONS_DIR = (
    Path(__file__).resolve().parents[2] / "uav" / "config" / "camera_calibrations"
)


def _required_names(schema: dict[str, Any]) -> set[str]:
    return set(schema.get("required", []))


def _available_camera_calibration_files() -> list[str]:
    if not _CAMERA_CALIBRATIONS_DIR.is_dir():
        return []
    filenames = {
        path.name
        for pattern in ("*.yaml", "*.yml")
        for path in _CAMERA_CALIBRATIONS_DIR.glob(pattern)
        if path.is_file()
    }
    return sorted(filenames)


def _mode_reference(entry: Any) -> str:
    return public_mode_id(
        getattr(entry, "mode", None) or getattr(entry, "class_path", None)
    )


def _lookup_mode_entry(mode_ref: str) -> Any:
    candidates = []
    for candidate in (internal_mode_path(mode_ref), public_mode_id(mode_ref)):
        if candidate and candidate not in candidates:
            candidates.append(candidate)

    last_error: Exception | None = None
    for candidate in candidates:
        try:
            return mode_entry_for_class_path(candidate)
        except Exception as exc:  # pragma: no cover - runtime compatibility shim
            last_error = exc

    if last_error is not None:
        raise last_error
    raise ValueError(f"Unable to resolve mode entry for {mode_ref!r}")


def _schema_field(
    name: str,
    property_schema: dict[str, Any],
    *,
    required: bool,
    applies_to: list[str] | None = None,
    editable: bool = True,
    derived: bool = False,
) -> SchemaFieldResponse:
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

    return SchemaFieldResponse(
        name=name,
        schema_type=schema_type,
        annotation=annotation,
        required=required,
        default=default,
        default_kind=default_kind,
        nullable=nullable,
        choices=choices,
        description=property_schema.get("description"),
        editable=editable,
        derived=derived,
        applies_to=list(applies_to or []),
    )


def _fields_from_model(
    schema: dict[str, Any],
    *,
    applies_to: list[str] | None = None,
    editable: bool = True,
    derived_fields: set[str] | None = None,
) -> list[SchemaFieldResponse]:
    properties = schema.get("properties", {})
    required = _required_names(schema)
    derived_fields = derived_fields or set()
    fields = [
        _schema_field(
            name,
            property_schema,
            required=name in required,
            applies_to=applies_to,
            editable=editable,
            derived=name in derived_fields,
        )
        for name, property_schema in properties.items()
    ]
    calibration_choices = _available_camera_calibration_files()
    if calibration_choices:
        fields = [
            field.model_copy(update={"choices": calibration_choices})
            if field.name == "camera_calibration_file"
            else field
            for field in fields
        ]
    return fields


def _mode_metadata_response(mode_ref: str | Any) -> ModeMetadataResponse:
    if isinstance(mode_ref, str):
        entry = _lookup_mode_entry(mode_ref)
    else:
        entry = mode_ref
    return ModeMetadataResponse(
        name=entry.display_name,
        mode=_mode_reference(entry),
        module=entry.module_path,
        mission_target=entry.mission_target,
        description=entry.description or None,
        required_vision_nodes=list(entry.required_vision_nodes),
        transition_labels=list(entry.transition_labels),
        params=_fields_from_model(entry.params_schema),
        params_schema=entry.params_schema,
    )


def mode_registry(target: str | None = None) -> ModeRegistryResponse:
    entries = mode_registry_entries(mission_target=target)
    grouped: dict[str, list[ModeMetadataResponse]] = {}
    for entry in entries:
        grouped.setdefault(entry.mission_target, []).append(
            _mode_metadata_response(entry)
        )
    return ModeRegistryResponse(targets=grouped)


def _available_mission_names() -> list[str]:
    return sorted(path.stem for path in mission_root().glob("*.yaml"))


def _available_fleet_names() -> list[str]:
    return sorted(path.stem for path in repo_fleet_paths())


def mission_schema_for_path(path: str | Path) -> MissionSchemaResponse:
    mission_path = Path(path)
    mission_spec = load_mission_spec_compat(mission_path)
    modes: list[MissionModeResponse] = []

    for mode_name, mode_spec in mission_spec.modes.items():
        mode_ref = public_mode_id(
            getattr(mode_spec, "mode", None) or getattr(mode_spec, "class_path", None)
        )
        modes.append(
            MissionModeResponse(
                name=mode_name,
                mode=mode_ref,
                params=dict(mode_spec.params),
                transitions=dict(mode_spec.transitions),
                metadata=_mode_metadata_response(mode_ref),
            )
        )

    return MissionSchemaResponse(
        name=mission_path.stem,
        path=str(mission_path),
        target=mission_spec.target,
        start_mode="start",
        vision_nodes=list(mission_spec.vision_nodes),
        modes=modes,
        document_schema=MissionDocumentModel.model_json_schema(),
    )


def mission_schema_for_name(name: str) -> MissionSchemaResponse:
    return mission_schema_for_path(mission_path_for_name(name))


def mission_catalog(target: str | None = None) -> MissionCatalogResponse:
    missions = []
    for mission_name in _available_mission_names():
        mission = mission_schema_for_name(mission_name)
        if target and mission.target != target:
            continue
        missions.append(mission)
    return MissionCatalogResponse(
        available_missions=[mission.name for mission in missions],
        missions=missions,
    )


def fleet_schema() -> FleetSchemaResponse:
    sim_backend_schema = SimBackendModel.model_json_schema()
    hardware_backend_schema = HardwareBackendModel.model_json_schema()
    defaults_schema = FleetDefaultsModel.model_json_schema()
    vehicle_schema = FleetVehicleModel.model_json_schema()

    sim_backend_fields = {
        field.name: field
        for field in _fields_from_model(sim_backend_schema, applies_to=["sim"])
    }
    hardware_backend_fields = {
        field.name: field
        for field in _fields_from_model(
            hardware_backend_schema, applies_to=["hardware", "real"]
        )
    }

    backend_fields: list[SchemaFieldResponse] = []
    seen_backend: set[str] = set()
    for source in (sim_backend_fields, hardware_backend_fields):
        for name, field in source.items():
            if name in seen_backend:
                continue
            applies = []
            if name in sim_backend_fields:
                applies.append("sim")
            if name in hardware_backend_fields:
                applies.extend(["hardware", "real"])
            backend_fields.append(field.model_copy(update={"applies_to": applies}))
            seen_backend.add(name)

    defaults_fields = _fields_from_model(
        defaults_schema, applies_to=["sim", "hardware", "real"]
    )
    vehicle_fields = {
        field.name: field
        for field in _fields_from_model(
            vehicle_schema, applies_to=["sim", "hardware", "real"]
        )
    }

    return FleetSchemaResponse(
        available_fleets=_available_fleet_names(),
        backend_kinds=list(_FLEET_BACKEND_KINDS),
        sections=[
            FleetSectionSchemaResponse(
                name="backend",
                description="Backend configuration shared by every vehicle in the fleet.",
                applies_to=["sim", "hardware", "real"],
                constraints=[
                    "kind must be sim, hardware, or real.",
                    "real is an alias for the hardware backend path.",
                    "Sim backends use world_name, optional mission_stage, and optional world_overrides.",
                ],
                fields=backend_fields,
            ),
            FleetSectionSchemaResponse(
                name="defaults",
                description="Shared per-vehicle knobs merged into every fleet entry.",
                applies_to=["sim", "hardware", "real"],
                constraints=[
                    "network_role, allowed_ap_vehicles, and ap_selection are only used for hardware/real deploys.",
                ],
                fields=defaults_fields,
            ),
            FleetSectionSchemaResponse(
                name="vehicle.common",
                description="Per-vehicle fleet fields shared across backends.",
                applies_to=["sim", "hardware", "real"],
                constraints=[
                    "Provide mission or mission_path after defaults are merged.",
                    "kind is optional and must match the mission target when present.",
                ],
                fields=[
                    vehicle_fields[name]
                    for name in ("name", "mission", "mission_path", "kind")
                    if name in vehicle_fields
                ],
            ),
            FleetSectionSchemaResponse(
                name="vehicle.sim",
                description="Sim-only fleet fields for backend controllable attachment.",
                applies_to=["sim"],
                constraints=[
                    "controllable is required for sim vehicles.",
                    "px4_instance is derived by the fleet launcher.",
                ],
                fields=[
                    vehicle_fields[name]
                    for name in ("controllable", "px4_instance")
                    if name in vehicle_fields
                ],
            ),
            FleetSectionSchemaResponse(
                name="vehicle.hardware",
                description="Hardware-only fleet fields for target-specific runtime wiring.",
                applies_to=["hardware", "real"],
                constraints=[
                    "UAV hardware/real entries require px4_airframe_id.",
                    "network_role may be ap, client, or default.",
                    "allowed_ap_vehicles names other fleet vehicles and only affects fallback Wi-Fi policy.",
                ],
                fields=[
                    vehicle_fields[name]
                    for name in (
                        "px4_airframe_id",
                        "px4_namespace",
                        "payload_controller",
                        "network_role",
                        "allowed_ap_vehicles",
                        "ap_selection",
                    )
                    if name in vehicle_fields
                ],
            ),
        ],
        excluded_keys=list(_FLEET_EXCLUDED_KEYS),
        document_schema=FleetDocumentModel.model_json_schema(),
    )


def schema_index() -> SchemaIndexResponse:
    return SchemaIndexResponse(
        missions=mission_catalog(),
        fleet=fleet_schema(),
        modes=mode_registry(),
    )
