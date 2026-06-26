#!/usr/bin/env python3

import json
import logging
import os

import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sim.constants import (
    COMPETITION_NAMES,
    DEFAULT_COMPETITION,
    DEFAULT_USE_SCORING,
    Competition,
)
from sim.scoring.namespacing import resolve_scoring_vehicle_name
from sim.orchestration import parse_json_config, resolve_stage_world
from sim.utils import (
    build_node_arguments,
    camel_to_snake,
    load_sim_launch_parameters,
)
from uav.utils import DEFAULT_PX4_PATH


def _is_truthy_env(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in {"1", "true", "yes", "on"}


def _is_falsey_env(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in {"0", "false", "no", "off"}


def _gui_enabled() -> bool:
    if _is_truthy_env("SAE_SIM_HEADLESS"):
        return False
    if _is_falsey_env("SAE_SIM_GUI"):
        return False
    return True


def _spawn_world_command(*, competition: str, model_store: str) -> list[str]:
    world_path = os.path.join(model_store, "worlds", f"{competition}.sdf")
    cmd = ["gz", "sim", "--render-engine", "ogre", "-r", world_path]
    if not _gui_enabled():
        cmd.extend(["-s", "--headless-rendering"])
    return cmd


def initialize_mode(logger: logging.Logger, node_path: str, params: dict) -> Node:
    logger.debug(f"Initializing mode: {node_path} with params: {params}")
    try:
        module_name, class_name = node_path.rsplit(".", 1)
    except ValueError as exc:
        raise ValueError(
            f"Invalid node path format: '{node_path}'. Expected 'module.ClassName'"
        ) from exc

    module = __import__(module_name, fromlist=[class_name])
    if not hasattr(module, class_name):
        raise AttributeError(
            f"Class '{class_name}' not found in module '{module_name}'"
        )
    node_class = getattr(module, class_name)
    args = build_node_arguments(node_class, params)
    return node_class(**args)


def _load_backend_override(context) -> dict:
    return parse_json_config(
        LaunchConfiguration("backend_json").perform(context),
        field_name="backend_json",
    )


def _competition_name(legacy_params: dict, backend_override: dict) -> str:
    explicit_world_name = str(backend_override.get("world_name", "")).strip()
    if explicit_world_name:
        return explicit_world_name

    competition_num = backend_override.get(
        "competition", legacy_params.get("competition", DEFAULT_COMPETITION.value)
    )
    try:
        competition_type = Competition(competition_num)
        return COMPETITION_NAMES[competition_type]
    except (ValueError, KeyError) as exc:
        valid_values = [entry.value for entry in Competition]
        raise ValueError(
            f"Invalid competition: {competition_num}. Must be one of {valid_values}"
        ) from exc


def launch_setup(context, *args, **kwargs):
    logger = launch.logging.get_logger("sim.launch")
    legacy_params = load_sim_launch_parameters()
    backend_override = _load_backend_override(context)

    competition = _competition_name(legacy_params, backend_override)
    logger.info(f"Running Competition: {competition}")

    mission_stage = str(
        backend_override.get("mission_stage", legacy_params.get("mission_stage", ""))
    ).strip()

    if backend_override:
        scoring_param = backend_override.get(
            "use_scoring", backend_override.get("scoring", False)
        )
    else:
        scoring_param = legacy_params.get("scoring", DEFAULT_USE_SCORING)
    use_scoring = (
        scoring_param.lower() == "true"
        if isinstance(scoring_param, str)
        else bool(scoring_param)
    )

    px4_path = os.path.expanduser(LaunchConfiguration("px4_path").perform(context))
    if px4_path is None:
        raise RuntimeError("PX4 path is required")
    cwd = os.path.expanduser(os.getcwd())

    model_store = os.path.expanduser("~/.simulation-gazebo")
    server_config_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "server.config"
    )
    gz_sim_env = {
        "GZ_SIM_RESOURCE_PATH": os.path.join(model_store, "models"),
        "GZ_SIM_SERVER_CONFIG_PATH": server_config_path,
    }
    if not _gui_enabled():
        gz_sim_env["LIBGL_ALWAYS_SOFTWARE"] = "1"
        gz_sim_env["QT_QPA_PLATFORM"] = "offscreen"
    else:
        gz_sim_env["QT_QPA_PLATFORM_PLUGIN_PATH"] = ""
        gz_sim_env["QT_QPA_FONTDIR"] = ""

    download_gz_models = ExecuteProcess(
        cmd=["python3", "Tools/simulation/gz/simulation-gazebo", "--dryrun"],
        cwd=px4_path,
        output="screen",
        name="download_gz_models",
    )
    spawn_world = ExecuteProcess(
        cmd=_spawn_world_command(competition=competition, model_store=model_store),
        cwd=px4_path,
        output="screen",
        name="spawn_world",
        additional_env=gz_sim_env,
    )
    gz_ros_bridge_create = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[f"/world/{competition}/create@ros_gz_interfaces/srv/SpawnEntity"],
        output="screen",
        name="gz_ros_bridge_create",
        cwd=cwd,
    )

    model = LaunchConfiguration("model").perform(context)
    spawn_uav_model = LaunchConfiguration("spawn_uav_model").perform(
        context
    ).strip().lower() in {"1", "true", "yes", "on"}

    if "world" in backend_override:
        raise ValueError(
            "backend_json no longer supports 'world'. Use 'world_name', optional "
            "'mission_stage', and optional 'world_overrides' instead."
        )

    resolved_world = resolve_stage_world(
        world_name=competition,
        mission_stage=mission_stage,
        world_overrides=backend_override.get("world_overrides"),
        logger=logger,
    )
    resolved_stage = resolved_world["mission_stage"]
    logger.info(f"Mission stage: {resolved_stage}")

    sim_stage_params = resolved_world["sim_stage_params"]
    world_spec = resolved_world["world"]
    world_params = dict(world_spec["params"])
    if spawn_uav_model:
        if not model:
            raise ValueError("spawn_uav_model requires a non-empty model name.")
        sim_model_name = model[3:] if model.startswith("gz_") else model
        vehicle_pose = sim_stage_params["world"]["params"].get(
            "vehicle_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        if len(vehicle_pose) != 6:
            raise ValueError(
                f"vehicle_pose must contain exactly 6 values. Received: {vehicle_pose}"
            )
        entities = dict(world_params.get("entities", {}))
        entities[f"{sim_model_name}_0"] = {
            "path_to_sdf": f"~/.simulation-gazebo/models/{sim_model_name}/model.sdf",
            "position": vehicle_pose[:3],
            "rpy": vehicle_pose[3:],
        }
        world_params["entities"] = entities

    world_node_name = camel_to_snake(world_spec["name"])
    world = Node(
        package="sim",
        executable=camel_to_snake(world_spec["name"]),
        arguments=[json.dumps(world_params)],
        output="screen",
        name=world_node_name,
        cwd=cwd,
    )

    trigger_world_gen = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            f"/{world_node_name}/trigger_world_gen",
            "std_srvs/srv/Trigger",
        ],
        cwd=cwd,
        output="screen",
        name="trigger_world_gen",
    )

    scoring = None
    if use_scoring and "scoring" in sim_stage_params:
        scoring_params = dict(sim_stage_params["scoring"].get("params") or {})
        vehicle_name = resolve_scoring_vehicle_name(scoring_params, world_params)
        scoring_params["vehicle_name"] = vehicle_name
        logger.info(f"Scoring vehicle: {vehicle_name}")
        scoring = Node(
            package="sim",
            executable=camel_to_snake(sim_stage_params["scoring"]["name"]),
            arguments=[json.dumps(scoring_params)],
            output="screen",
            name=sim_stage_params["scoring"]["name"],
            cwd=cwd,
        )

    sim_startup_started = {"value": False}

    def maybe_start_sim(event):
        text = event.text.decode() if isinstance(event.text, bytes) else event.text
        if (
            sim_startup_started["value"]
            or "Successfully generated world file:" not in text
        ):
            return None
        sim_startup_started["value"] = True
        startup_actions = [spawn_world, LogInfo(msg="Simulation world node started.")]
        if scoring is not None:
            startup_actions.append(scoring)
        return startup_actions

    actions = [
        download_gz_models,
        RegisterEventHandler(
            OnProcessExit(
                target_action=download_gz_models,
                on_exit=[LogInfo(msg="Gazebo models downloaded."), world],
            )
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=world,
                on_stderr=maybe_start_sim,
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=spawn_world,
                on_start=[LogInfo(msg="spawn_world started"), gz_ros_bridge_create],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=gz_ros_bridge_create,
                on_start=[trigger_world_gen, LogInfo(msg="World generation triggered")],
            )
        ),
    ]

    if scoring:
        actions.append(
            RegisterEventHandler(
                OnProcessStart(
                    target_action=scoring,
                    on_start=[LogInfo(msg="Scoring node started.")],
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("px4_path", default_value=DEFAULT_PX4_PATH),
            DeclareLaunchArgument("model", default_value=""),
            DeclareLaunchArgument("spawn_uav_model", default_value="false"),
            DeclareLaunchArgument("backend_json", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
