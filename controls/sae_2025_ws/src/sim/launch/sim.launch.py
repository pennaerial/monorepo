#!/usr/bin/env python3

import importlib
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
from sim.utils import (
    build_node_arguments,
    camel_to_snake,
    load_sim_launch_parameters,
    load_sim_parameters,
)


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

    module = importlib.import_module(module_name)
    if not hasattr(module, class_name):
        raise AttributeError(
            f"Class '{class_name}' not found in module '{module_name}'"
        )
    node_class = getattr(module, class_name)
    args = build_node_arguments(node_class, params)
    return node_class(**args)


def _camera_bridge_nodes(
    *,
    competition: str,
    model: str,
    vehicle_type: str,
    vehicle_name: str,
    cwd: str,
):
    if not vehicle_type or not vehicle_name:
        return []

    if vehicle_type == "uav":
        if not model:
            raise ValueError("UAV camera bridging requires a non-empty model name.")
        sim_model_name = model[3:] if model.startswith("gz_") else model
        gz_image_topic = f"/world/{competition}/model/{sim_model_name}_0/link/camera_link/sensor/camera/image"
        gz_camera_info_topic = f"/world/{competition}/model/{sim_model_name}_0/link/camera_link/sensor/camera/camera_info"
        ros_namespace = f"/{vehicle_name}"
    elif vehicle_type == "payload":
        gz_image_topic = f"/world/{competition}/model/{vehicle_name}/link/camera_link/sensor/camera/image"
        gz_camera_info_topic = f"/world/{competition}/model/{vehicle_name}/link/camera_link/sensor/camera/camera_info"
        ros_namespace = f"/{vehicle_name}"
    else:
        raise ValueError(f"Unsupported camera_vehicle_type '{vehicle_type}'.")

    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[f"{gz_image_topic}@sensor_msgs/msg/Image[gz.msgs.Image"],
            remappings=[(gz_image_topic, f"{ros_namespace}/camera")],
            output="screen",
            name=f"{vehicle_name}_camera_bridge",
            cwd=cwd,
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                f"{gz_camera_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
            ],
            remappings=[(gz_camera_info_topic, f"{ros_namespace}/camera_info")],
            output="screen",
            name=f"{vehicle_name}_camera_info_bridge",
            cwd=cwd,
        ),
    ]


def launch_setup(context, *args, **kwargs):
    logger = launch.logging.get_logger("sim.launch")
    sim_params = load_sim_launch_parameters()

    competition_num = sim_params.get("competition", DEFAULT_COMPETITION.value)
    try:
        competition_type = Competition(competition_num)
        competition = COMPETITION_NAMES[competition_type]
    except (ValueError, KeyError) as exc:
        valid_values = [entry.value for entry in Competition]
        raise ValueError(
            f"Invalid competition: {competition_num}. Must be one of {valid_values}"
        ) from exc
    logger.info(f"Running Competition: {competition}")

    mission_stage = str(sim_params.get("mission_stage", "")).strip()
    if mission_stage:
        logger.info(f"Mission stage: {mission_stage}")

    scoring_param = sim_params.get("scoring", DEFAULT_USE_SCORING)
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
        # OpenCV's Python package mutates QT_QPA_* env vars at import time.
        # Gazebo GUI inheriting those points it at cv2's bundled Qt plugins,
        # which crashes the GUI startup on this machine.
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
    camera_vehicle_type = LaunchConfiguration("camera_vehicle_type").perform(context)
    camera_vehicle_name = LaunchConfiguration("camera_vehicle_name").perform(context)
    camera_bridge_actions = _camera_bridge_nodes(
        competition=competition,
        model=model,
        vehicle_type=str(camera_vehicle_type).strip(),
        vehicle_name=str(camera_vehicle_name).strip(),
        cwd=cwd,
    )

    sim_stage_params, sim_config_path = load_sim_parameters(
        competition, logger, competition, mission_stage
    )
    if "world" not in sim_stage_params:
        raise ValueError(
            f"Missing 'world' section in simulation config: {sim_config_path}"
        )

    world_params = sim_stage_params["world"].copy()
    world_node_params = world_params["params"].copy()
    if spawn_uav_model:
        if not model:
            raise ValueError("spawn_uav_model requires a non-empty model name.")
        sim_model_name = model[3:] if model.startswith("gz_") else model
        vehicle_pose = world_node_params.get(
            "vehicle_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        if len(vehicle_pose) != 6:
            raise ValueError(
                f"vehicle_pose must contain exactly 6 values. Received: {vehicle_pose}"
            )
        entities = dict(world_node_params.get("entities", {}))
        entities[f"{sim_model_name}_0"] = {
            "path_to_sdf": f"~/.simulation-gazebo/models/{sim_model_name}/model.sdf",
            "position": vehicle_pose[:3],
            "rpy": vehicle_pose[3:],
        }
        world_node_params["entities"] = entities

    world_node_name = camel_to_snake(world_params["name"])
    world = Node(
        package="sim",
        executable=camel_to_snake(world_params["name"]),
        arguments=[json.dumps(world_node_params)],
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
    if use_scoring and "scoring" in sim_params:
        scoring = Node(
            package="sim",
            executable=camel_to_snake(sim_params["scoring"]["name"]),
            arguments=[sim_params["scoring"]["params"]],
            output="screen",
            name=sim_params["scoring"]["name"],
            cwd=cwd,
        )

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
                on_stderr=lambda event: (
                    [
                        spawn_world,
                        LogInfo(msg="Simulation world node started."),
                        scoring,
                    ]
                    if b"Successfully generated world file:" in event.text
                    else None
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=spawn_world,
                on_start=[
                    LogInfo(msg="spawn_world started, creating bridges"),
                    gz_ros_bridge_create,
                    *camera_bridge_actions,
                ],
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
            DeclareLaunchArgument("px4_path", default_value="~/PX4-Autopilot"),
            DeclareLaunchArgument("model", default_value=""),
            DeclareLaunchArgument("spawn_uav_model", default_value="false"),
            DeclareLaunchArgument("camera_vehicle_type", default_value=""),
            DeclareLaunchArgument("camera_vehicle_name", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
