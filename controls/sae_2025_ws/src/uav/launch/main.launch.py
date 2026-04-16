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
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration

from uav.runtime.mission_spec import MissionSpec, mission_path_for_name
from uav.utils import find_folder_with_heuristic, get_airframe_details, vehicle_id_dict


def _load_launch_parameters(context) -> dict:
    params_path_override = LaunchConfiguration("params_file").perform(context).strip()
    if params_path_override:
        params_path = Path(os.path.expanduser(params_path_override)).resolve()
    else:
        params_path = Path(__file__).resolve().with_name("launch_params.yaml")
    with params_path.open("r", encoding="utf-8") as params_file:
        return yaml.safe_load(params_file) or {}


def _yaml_or_launch_string(context, name: str, config_value) -> str:
    override = LaunchConfiguration(name).perform(context).strip()
    if override:
        return override
    return "" if config_value is None else str(config_value).strip()


def _yaml_bool_value(config_value, *, name: str, default: bool) -> bool:
    if config_value is None:
        return bool(default)
    if isinstance(config_value, bool):
        return config_value
    raise ValueError(
        f"Launch parameter '{name}' must be a YAML boolean, received {config_value!r}."
    )


def _warn_logger(logger, message: str) -> None:
    if logger is None:
        return
    warn = getattr(logger, "warning", None) or getattr(logger, "warn", None)
    if callable(warn):
        warn(message)


def _resolve_force_camera(params: dict, *, logger=None) -> bool:
    has_force_camera = "force_camera" in params
    has_use_camera = "use_camera" in params

    if has_force_camera:
        force_camera = _yaml_bool_value(
            params.get("force_camera"), name="force_camera", default=False
        )
        if has_use_camera:
            _warn_logger(
                logger,
                "Launch parameter 'use_camera' is deprecated and ignored because "
                "'force_camera' is also set.",
            )
        return force_camera

    if has_use_camera:
        _warn_logger(
            logger,
            "Launch parameter 'use_camera' is deprecated; use 'force_camera' instead.",
        )
        return _yaml_bool_value(
            params.get("use_camera"), name="use_camera", default=False
        )

    return False


def _resolve_airframe_id(airframe_value) -> int:
    try:
        return int(airframe_value)
    except (TypeError, ValueError):
        try:
            return vehicle_id_dict[str(airframe_value)]
        except KeyError as exc:
            raise ValueError(f"Unknown airframe name: {airframe_value}") from exc


def resolve_stage_world(**kwargs):
    from sim.orchestration import resolve_stage_world as _resolve_stage_world

    return _resolve_stage_world(**kwargs)


def _resolved_sim_world_name(sim_params: dict) -> str:
    from sim.constants import COMPETITION_NAMES, DEFAULT_COMPETITION, Competition

    competition_num = sim_params.get("competition", DEFAULT_COMPETITION.value)
    try:
        competition_type = Competition(competition_num)
        return COMPETITION_NAMES[competition_type]
    except (ValueError, KeyError) as exc:
        valid_values = [entry.value for entry in Competition]
        raise ValueError(
            f"Invalid competition: {competition_num}. Must be one of {valid_values}"
        ) from exc


def _inject_single_uav_controllable(
    *,
    world_params: dict,
    vehicle_name: str,
    model: str,
    px4_airframe_id: int,
    vehicle_pose: list[float],
) -> None:
    if len(vehicle_pose) != 6:
        raise ValueError(
            f"vehicle_pose must contain exactly 6 values. Received: {vehicle_pose}"
        )
    sim_model_name = model[3:] if model.startswith("gz_") else model
    world_params.setdefault("controllables", {})
    world_params["controllables"][vehicle_name] = {
        "kind": "uav",
        "px4_airframe_id": int(px4_airframe_id),
        "path_to_sdf": f"~/.simulation-gazebo/models/{sim_model_name}/model.sdf",
        "model": model,
        "position": list(vehicle_pose[:3]),
        "rpy": list(vehicle_pose[3:]),
    }


def _legacy_backend_override(
    *,
    mission_spec: MissionSpec,
    sim_params: dict,
    vehicle_name: str,
    model: str,
    px4_airframe_id: int | None = None,
) -> tuple[dict, str]:
    world_name = _resolved_sim_world_name(sim_params)
    resolved_world = resolve_stage_world(
        world_name=world_name,
        mission_stage=sim_params.get("mission_stage", ""),
        logger=None,
    )
    sim_stage_params = resolved_world["sim_stage_params"]
    world_params = deepcopy(resolved_world["world"]["params"])
    world_overrides = {}

    if mission_spec.is_uav:
        vehicle_pose = sim_stage_params["world"]["params"].get("vehicle_pose")
        if vehicle_pose is None:
            legacy_uav = world_params.get("controllables", {}).get(vehicle_name)
            if not isinstance(legacy_uav, dict):
                legacy_uav = world_params.get("controllables", {}).get("uav_0", {})
            position = list(legacy_uav.get("position", [0.0, 0.0, 0.0]))
            rpy = list(legacy_uav.get("rpy", [0.0, 0.0, 0.0]))
            vehicle_pose = position[:3] + rpy[:3]
        if px4_airframe_id is None:
            raise ValueError(
                "Single-vehicle UAV sim launch requires a PX4 airframe ID."
            )
        injected_params = {}
        _inject_single_uav_controllable(
            world_params=injected_params,
            vehicle_name=vehicle_name,
            model=model,
            px4_airframe_id=int(px4_airframe_id),
            vehicle_pose=vehicle_pose,
        )
        world_overrides = {"params": injected_params}

    available_payloads = sorted(
        name
        for name, spec in dict(world_params.get("controllables", {})).items()
        if isinstance(spec, dict) and str(spec.get("kind", "")).strip() == "payload"
    )
    if mission_spec.is_payload and vehicle_name not in available_payloads:
        raise ValueError(
            f"Configured vehicle_name '{vehicle_name}' was not found in the selected sim world. "
            f"Available payloads: {available_payloads}"
        )

    backend = {
        "kind": "sim",
        "world_name": world_name,
        "mission_stage": resolved_world["mission_stage"],
        "world_overrides": world_overrides,
        "scoring": False,
    }
    return backend, world_name


def _single_vehicle_config(context, params: dict) -> tuple[dict, dict | None]:
    logger = get_logger("main.launch")
    mission_name = _yaml_or_launch_string(
        context, "mission_name", params.get("mission_name", "basic")
    )
    mission_path = mission_path_for_name(mission_name)
    mission_spec = MissionSpec.load(mission_path)

    sim = bool(params.get("sim", False))
    auto_launch = _yaml_bool_value(
        params.get("auto_launch", True), name="auto_launch", default=True
    )
    default_vehicle_name = "uav" if mission_spec.is_uav else "payload_0"
    vehicle_name = _yaml_or_launch_string(
        context, "vehicle_name", params.get("vehicle_name") or default_vehicle_name
    )
    if not vehicle_name:
        raise ValueError("Single-vehicle launch requires a non-empty vehicle name.")

    px4_path = find_folder_with_heuristic(
        "PX4-Autopilot",
        os.path.expanduser(LaunchConfiguration("px4_path").perform(context)),
    )
    model = str(params.get("custom_airframe_model", "")).strip()
    vehicle_class_name = None
    airframe_id = None
    if mission_spec.is_uav:
        airframe_id = _resolve_airframe_id(params.get("airframe", "quadcopter"))
        vehicle_class, _, airframe_model = get_airframe_details(px4_path, airframe_id)
        vehicle_class_name = vehicle_class.name
        if not model:
            model = airframe_model

    vehicle_config = {
        "kind": mission_spec.target,
        "vehicle_name": vehicle_name,
        "mission_name": mission_name,
        "mission_path": mission_path,
        "sim": sim,
        "auto_launch": auto_launch,
        "debug": bool(params.get("debug", False)),
        "vision_debug": bool(params.get("vision_debug", False)),
        "save_vision_milliseconds": int(params.get("save_vision_milliseconds", 0)),
        "servo_only": bool(params.get("servo_only", False)),
        "payload_controller": str(params.get("payload_controller", "")).strip(),
        "camera_input_transport": str(
            params.get("camera_input_transport", "raw" if sim else "compressed")
        ).strip(),
        "camera_rotate_degrees": float(
            params.get("camera_rotate_degrees", 0.0 if sim else 180.0)
        ),
        "camera_calibration_file": str(
            params.get("camera_calibration_file", "")
        ).strip(),
        "camera_preprocess_hook": str(params.get("camera_preprocess_hook", "")).strip(),
        "camera_mount_offsets": list(
            params.get("camera_mount_offsets", [0.0, 0.0, 0.0])
        ),
        "airframe": params.get("airframe", "quadcopter"),
        "model": model,
        "px4_path": px4_path,
        "px4_instance": int(params.get("px4_instance", 0)),
        "force_camera": _resolve_force_camera(params, logger=logger),
        "launch_middleware": mission_spec.is_uav,
        "launch_px4_sitl": bool(sim and mission_spec.is_uav),
        "launch_payload_backend": mission_spec.is_payload,
        "sim_entity_name": vehicle_name,
        # udp_port: None = auto-compute (payload_N → 5000+N), 0 = disabled, >0 = explicit
        "udp_port": params.get("udp_port"),
        "udp_all_ports": list(params.get("udp_all_ports") or [5000, 5001, 5002, 5003]),
        "udp_topics": list(
            params.get("udp_topics")
            or ["heartbeat:payload_interfaces/msg/PayloadHeartbeat"]
        ),
        "udp_broadcast_ip": str(
            params.get("udp_broadcast_ip") or "10.42.0.255"
        ).strip(),
        "udp_peer_ttl": float(params.get("udp_peer_ttl") or 3.0),
    }
    if vehicle_class_name is not None:
        vehicle_config["vehicle_class"] = vehicle_class_name
    if airframe_id is not None:
        vehicle_config["px4_airframe_id"] = int(airframe_id)

    if not sim:
        return vehicle_config, None

    from sim.utils import load_sim_launch_parameters

    sim_params = load_sim_launch_parameters()
    backend, world_name = _legacy_backend_override(
        mission_spec=mission_spec,
        sim_params=sim_params,
        vehicle_name=vehicle_name,
        model=model,
        px4_airframe_id=airframe_id,
    )
    vehicle_config["sim_world_name"] = world_name
    return vehicle_config, backend


def launch_setup(context, *args, **kwargs):
    params = _load_launch_parameters(context)
    vehicle_config, backend_override = _single_vehicle_config(context, params)

    actions = []
    if backend_override is not None:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("sim"), "launch", "sim.launch.py"
                    )
                ),
                launch_arguments={
                    "px4_path": vehicle_config["px4_path"],
                    "backend_json": json.dumps(backend_override),
                }.items(),
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("uav"),
                    "launch",
                    "vehicle_stack.launch.py",
                )
            ),
            launch_arguments={"vehicle_json": json.dumps(vehicle_config)}.items(),
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("px4_path", default_value="~/PX4-Autopilot"),
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("mission_name", default_value=""),
            DeclareLaunchArgument("vehicle_name", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
