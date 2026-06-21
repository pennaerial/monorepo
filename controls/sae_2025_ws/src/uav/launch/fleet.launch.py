#!/usr/bin/env python3
import json
import os
from copy import deepcopy
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration

from vehicle_common.runtime.fleet_spec import load_fleet_document
from vehicle_common.runtime.mission_spec import MissionSpec, mission_path_for_name

_SHARED_DEFAULT_KEYS = {
    "auto_launch",
    "debug",
    "vision_debug",
    "debug_vision_node",
    "force_camera",
    "save_vision_milliseconds",
    "servo_only",
    "camera_mount_offsets",
    "camera_input_transport",
    "camera_rotate_degrees",
    "camera_calibration_file",
    "camera_preprocess_hook",
    "network_role",
    "allowed_ap_vehicles",
    "ap_selection",
}

_EXCLUDED_FLEET_KEYS = {
    "airframe",
    "custom_airframe_model",
    "model",
    "sim",
    "px4_namespace",
    "px4_instance",
    "payload_controller",
}


def _normalize_backend_kind(kind: str) -> str:
    normalized = str(kind).strip()
    return "hardware" if normalized == "real" else normalized


def _prevalidate_raw_fleet_keys(fleet: dict) -> None:
    if not isinstance(fleet, dict):
        return

    backend = fleet.get("backend", {})
    backend_kind = ""
    if isinstance(backend, dict):
        backend_kind = _normalize_backend_kind(backend.get("kind", ""))

    _defaults_config(fleet)
    for raw_vehicle in _vehicles_config(fleet):
        name = str(raw_vehicle.get("name", "")).strip() or "<unnamed>"
        _validate_vehicle_keys(
            raw_vehicle,
            name_hint=name,
            backend_kind=backend_kind,
        )


def _resolve_fleet_path(fleet_path: str) -> Path:
    direct = Path(os.path.expanduser(fleet_path))
    if direct.is_absolute() or os.sep in fleet_path or "/" in fleet_path:
        return direct.resolve()
    if direct.exists():
        return direct.resolve()
    fleets_dir = Path(get_package_share_directory("uav")) / "fleets"
    candidate = fleets_dir / fleet_path
    if not candidate.suffix:
        candidate = candidate.with_suffix(".yaml")
    if candidate.exists():
        return candidate.resolve()
    raise FileNotFoundError(
        f"Could not find fleet file '{fleet_path}'. Tried cwd-relative "
        f"'{direct}' and fleets dir '{candidate}'."
    )


def _load_fleet_file(context) -> dict:
    fleet_path = LaunchConfiguration("fleet_file").perform(context).strip()
    if not fleet_path:
        raise ValueError("fleet.launch.py requires fleet_file:=...")
    resolved_path = _resolve_fleet_path(fleet_path)
    with resolved_path.open("r", encoding="utf-8") as fleet_file:
        raw_fleet = yaml.safe_load(fleet_file) or {}
    _prevalidate_raw_fleet_keys(raw_fleet)
    return load_fleet_document(resolved_path).model_dump(
        mode="python",
        exclude_none=True,
    )


def _backend_config(fleet: dict) -> dict:
    backend = deepcopy(fleet.get("backend", {}))
    if not isinstance(backend, dict):
        raise ValueError("Fleet file backend section must be a mapping.")
    kind = _normalize_backend_kind(backend.get("kind", ""))
    if kind == "hardware":
        return {
            "kind": "hardware",
            "px4_path": backend.get("px4_path", "~/PX4-Autopilot"),
        }
    if kind != "sim":
        raise ValueError(
            f"Unsupported backend kind '{kind}'. Expected one of: sim, hardware, real."
        )
    world_name = str(backend.get("world_name", "")).strip()
    if not world_name:
        raise ValueError("Sim backend requires a non-empty backend.world_name.")
    if "world" in backend:
        raise ValueError(
            "Fleet sim backends no longer support 'backend.world'. Use 'world_name', "
            "optional 'mission_stage', and optional 'world_overrides'."
        )

    from sim.orchestration import resolve_stage_world

    resolved_world = resolve_stage_world(
        world_name=world_name,
        mission_stage=backend.get("mission_stage", ""),
        world_overrides=backend.get("world_overrides"),
        logger=None,
    )
    controllables = deepcopy(resolved_world["world"]["params"].get("controllables", {}))
    if not isinstance(controllables, dict) or not controllables:
        raise ValueError(
            "Resolved sim stage requires a non-empty world.params.controllables mapping."
        )
    backend["world_name"] = world_name
    backend["mission_stage"] = resolved_world["mission_stage"]
    backend["world_overrides"] = deepcopy(backend.get("world_overrides", {}))
    backend["scoring"] = False
    backend["_resolved_world"] = resolved_world["world"]
    return backend


def _vehicles_config(fleet: dict) -> list[dict]:
    vehicles = deepcopy(fleet.get("vehicles", []))
    if not isinstance(vehicles, list) or not vehicles:
        raise ValueError("Fleet file requires a non-empty vehicles list.")
    for vehicle in vehicles:
        if not isinstance(vehicle, dict):
            raise ValueError("Each fleet vehicle entry must be a mapping.")
    return vehicles


def _defaults_config(fleet: dict) -> dict:
    defaults = deepcopy(fleet.get("defaults", {}))
    if defaults is None:
        return {}
    if not isinstance(defaults, dict):
        raise ValueError("Fleet file defaults section must be a mapping.")

    for key in defaults:
        if key in _EXCLUDED_FLEET_KEYS:
            raise ValueError(
                f"Fleet defaults do not support '{key}'. This setting is derived or "
                "single-vehicle-only."
            )
        if key not in _SHARED_DEFAULT_KEYS:
            raise ValueError(
                f"Fleet defaults field '{key}' is not supported. Allowed keys are: "
                f"{', '.join(sorted(_SHARED_DEFAULT_KEYS))}."
            )
    return defaults


def _validate_vehicle_keys(vehicle: dict, *, name_hint: str, backend_kind: str) -> None:
    for key in _EXCLUDED_FLEET_KEYS:
        if key in vehicle:
            raise ValueError(
                f"Fleet vehicle '{name_hint}' does not support '{key}'. "
                "This setting is derived or single-vehicle-only."
            )


def _resolve_mission_spec(vehicle: dict) -> tuple[str, MissionSpec]:
    mission_path = str(vehicle.get("mission_path", "")).strip()
    if mission_path:
        return mission_path, MissionSpec.load(mission_path)

    mission_name = str(vehicle.get("mission", "")).strip()
    if not mission_name:
        raise ValueError("Fleet vehicle requires mission or mission_path.")
    mission_path = mission_path_for_name(mission_name)
    return mission_path, MissionSpec.load(mission_path)


def _resolve_bool(value, *, key: str, default: bool) -> bool:
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
    raise ValueError(
        f"Fleet vehicle field '{key}' must be boolean, received {value!r}."
    )


def _vehicle_stack_configs(fleet: dict) -> tuple[dict, list[dict]]:
    _prevalidate_raw_fleet_keys(fleet)
    fleet = load_fleet_document(fleet).model_dump(mode="python", exclude_none=True)
    backend = _backend_config(fleet)
    defaults = _defaults_config(fleet)
    vehicles = _vehicles_config(fleet)
    backend_kind = backend["kind"]
    controllables = (
        backend["_resolved_world"]["params"]["controllables"]
        if backend_kind == "sim"
        else {}
    )

    resolved = []
    next_px4_instance = 0

    for raw_vehicle in vehicles:
        name = str(raw_vehicle.get("name", "")).strip() or "<unnamed>"
        _validate_vehicle_keys(raw_vehicle, name_hint=name, backend_kind=backend_kind)
        vehicle = deepcopy(defaults)
        vehicle.update(raw_vehicle)

        name = str(vehicle.get("name", "")).strip()
        controllable_name = str(vehicle.get("controllable", "")).strip()

        if not name:
            raise ValueError("Fleet vehicles require a non-empty name.")
        if backend_kind == "sim" and not controllable_name:
            raise ValueError(
                f"Fleet vehicle '{name}' requires a controllable reference."
            )
        if backend_kind == "sim" and controllable_name not in controllables:
            raise ValueError(
                f"Fleet vehicle '{name}' references unknown controllable '{controllable_name}'."
            )

        mission_path, mission_spec = _resolve_mission_spec(vehicle)
        mission_name = str(vehicle.get("mission", "")).strip()
        kind = mission_spec.target
        declared_kind = str(vehicle.get("kind", "")).strip()
        if declared_kind and declared_kind != kind:
            raise ValueError(
                f"Fleet vehicle '{name}' declared kind '{declared_kind}' "
                f"but mission target is '{kind}'."
            )

        if backend_kind == "sim":
            controllable = deepcopy(controllables[controllable_name])
            controllable_kind = str(controllable.get("kind", "")).strip()
            if controllable_kind and controllable_kind != kind:
                raise ValueError(
                    f"Fleet vehicle '{name}' kind '{kind}' does not match "
                    f"controllable '{controllable_name}' kind '{controllable_kind}'."
                )
        else:
            controllable = {}

        stack_config = {
            "kind": kind,
            "vehicle_name": name,
            "mission_name": mission_name,
            "mission_path": mission_path,
            "sim": backend_kind == "sim",
            "auto_launch": _resolve_bool(
                vehicle.get("auto_launch"),
                key="auto_launch",
                default=backend_kind == "sim",
            ),
            "debug": _resolve_bool(
                vehicle.get("debug", False), key="debug", default=False
            ),
            "vision_debug": _resolve_bool(
                vehicle.get("vision_debug", False), key="vision_debug", default=False
            ),
            "debug_vision_node": _resolve_bool(
                vehicle.get("debug_vision_node", False),
                key="debug_vision_node",
                default=False,
            ),
            "force_camera": _resolve_bool(
                vehicle.get("force_camera", False), key="force_camera", default=False
            ),
            "save_vision_milliseconds": int(vehicle.get("save_vision_milliseconds", 0)),
            "servo_only": _resolve_bool(
                vehicle.get("servo_only", False), key="servo_only", default=False
            ),
            "camera_input_transport": str(
                vehicle.get(
                    "camera_input_transport",
                    "raw" if backend_kind == "sim" else "compressed",
                )
            ).strip(),
            "camera_rotate_degrees": float(
                vehicle.get(
                    "camera_rotate_degrees", 0.0 if backend_kind == "sim" else 180.0
                )
            ),
            "camera_calibration_file": str(
                vehicle.get("camera_calibration_file", "")
            ).strip(),
            "camera_preprocess_hook": str(
                vehicle.get("camera_preprocess_hook", "")
            ).strip(),
            "camera_mount_offsets": list(
                vehicle.get("camera_mount_offsets", [0.0, 0.0, 0.0])
            ),
            "px4_path": os.path.expanduser(
                str(backend.get("px4_path", "~/PX4-Autopilot"))
            ),
            "sim_world_name": backend.get("world_name", ""),
            "sim_entity_name": controllable_name or name,
            "launch_middleware": backend_kind != "sim" and kind == "uav",
            "launch_px4_sitl": backend_kind == "sim" and kind == "uav",
            "launch_payload_backend": kind == "payload",
            "model": str(controllable.get("model", "")).strip(),
        }

        if kind == "payload":
            # udp_port: None = auto-compute (payload_N → 5000+N), 0 = disabled
            stack_config["udp_port"] = vehicle.get("udp_port")
            stack_config["udp_all_ports"] = list(
                vehicle.get("udp_all_ports") or [5000, 5001, 5002, 5003]
            )
            stack_config["udp_topics"] = list(
                vehicle.get("udp_topics")
                or ["heartbeat:payload_interfaces/msg/PayloadHeartbeat"]
            )
            stack_config["udp_broadcast_ip"] = str(
                vehicle.get("udp_broadcast_ip") or "10.42.0.255"
            ).strip()
            stack_config["udp_peer_ttl"] = float(vehicle.get("udp_peer_ttl") or 3.0)

        if kind == "uav":
            px4_airframe_id = (
                controllable.get("px4_airframe_id")
                if backend_kind == "sim"
                else vehicle.get("px4_airframe_id")
            )
            if px4_airframe_id is None:
                raise ValueError(
                    f"Fleet vehicle '{name}' requires controllable "
                    f"'{controllable_name or name}' to define px4_airframe_id."
                )
            px4_instance = next_px4_instance
            stack_config["px4_instance"] = int(px4_instance)
            stack_config["px4_airframe_id"] = int(px4_airframe_id)
            next_px4_instance = max(next_px4_instance, int(px4_instance) + 1)
        else:
            stack_config["payload_controller"] = (
                "SimController" if backend_kind == "sim" else "GPIOController"
            )

        resolved.append(stack_config)

    return backend, resolved


def launch_setup(context, *args, **kwargs):
    logger = get_logger("fleet.launch")
    fleet = _load_fleet_file(context)

    px4_path_override = LaunchConfiguration("px4_path").perform(context).strip()
    if px4_path_override:
        if not isinstance(fleet.get("backend"), dict):
            fleet["backend"] = {}
        fleet["backend"]["px4_path"] = px4_path_override

    backend, vehicles = _vehicle_stack_configs(fleet)

    actions = []

    if backend["kind"] == "sim":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("sim"), "launch", "sim.launch.py"
                    )
                ),
                launch_arguments={
                    "px4_path": str(backend.get("px4_path", "~/PX4-Autopilot")),
                    "backend_json": json.dumps(
                        {
                            key: value
                            for key, value in backend.items()
                            if not key.startswith("_")
                        }
                    ),
                }.items(),
            )
        )

    if backend["kind"] == "sim" and any(
        vehicle["kind"] == "uav" for vehicle in vehicles
    ):
        middleware_port = int(backend.get("middleware_port", 8888))
        logger.info(
            f"Launching shared MicroXRCEAgent for fleet on UDP port {middleware_port}."
        )
        actions.append(
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", str(middleware_port)],
                output="screen",
                name="fleet_middleware",
            )
        )

    vehicle_launch = os.path.join(
        get_package_share_directory("uav"), "launch", "vehicle_stack.launch.py"
    )
    for vehicle in vehicles:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vehicle_launch),
                launch_arguments={"vehicle_json": json.dumps(vehicle)}.items(),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("fleet_file", default_value=""),
            DeclareLaunchArgument("px4_path", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
