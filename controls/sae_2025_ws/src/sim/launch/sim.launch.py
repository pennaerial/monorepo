#!/usr/bin/env python3

import os
import launch
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
    DeclareLaunchArgument,
)
import logging
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart, OnProcessExit, OnProcessIO
from launch_ros.actions import Node
from sim.utils import (
    find_package_resource,
    load_yaml_to_dict,
    load_sim_launch_parameters,
    build_node_arguments,
    camel_to_snake,
)
import importlib
from pathlib import Path
from sim.constants import (
    Competition,
    COMPETITION_NAMES,
    DEFAULT_COMPETITION,
    DEFAULT_USE_SCORING,
)
import json


def load_sim_parameters(competition: str, logger: logging.Logger) -> str:
    """
    Find simulation configuration file, checking source location first (for development),
    then falling back to installed location.

    Args:
        competition: Competition name (e.g., 'in_house')

    Returns:
        Path to the simulation config file (as string)

    Raises:
        FileNotFoundError: If config file cannot be found
    """
    config_filename = f"{competition}.yaml"
    config_path = find_package_resource(
        relative_path=f"simulations/{config_filename}",
        package_name="sim",
        resource_type="file",
        logger=logger,
        base_file=Path(__file__),
    )
    return load_yaml_to_dict(config_path), config_path


def initialize_mode(logger: logging.Logger, node_path: str, params: dict) -> Node:
    """
    Initialize a node from a class path and parameters.

    Args:
        node_path: Dot-separated path to the node class (e.g., 'sim.world_gen.HoopCourseNode')
        params: Dictionary of parameters to pass to the node constructor

    Returns:
        Initialized node instance

    Raises:
        ValueError: If node_path is invalid or parameters are missing/invalid
        ImportError: If the module cannot be imported
        AttributeError: If the class is not found in the module
    """
    logger.debug(f"Initializing mode: {node_path} with params: {params}")

    # Parse node path
    try:
        module_name, class_name = node_path.rsplit(".", 1)
    except ValueError:
        raise ValueError(
            f"Invalid node path format: '{node_path}'. Expected 'module.ClassName'"
        )

    # Import module
    try:
        module = importlib.import_module(module_name)
    except ImportError as e:
        raise ImportError(f"Failed to import module '{module_name}': {e}")

    # Get class
    if not hasattr(module, class_name):
        raise AttributeError(
            f"Class '{class_name}' not found in module '{module_name}'"
        )

    node_class = getattr(module, class_name)

    # Build arguments from parameters
    try:
        args = build_node_arguments(node_class, params)
        logger.debug(f"Initializing {class_name} with args: {list(args.keys())}")
    except ValueError as e:
        raise ValueError(f"Failed to build arguments for {node_path}: {e}")

    # Instantiate node
    try:
        return node_class(**args)
    except Exception as e:
        logger.error(f"Failed to initialize {node_path} with args {args}: {e}")
        raise


def launch_setup(context, *args, **kwargs):
    """Setup launch configuration."""

    logger = launch.logging.get_logger("sim.launch")

    # Get launch arguments
    try:
        params = load_sim_launch_parameters()
    except Exception as e:
        raise RuntimeError(f"Failed to load launch parameters: {e}")

    # Map competition numbers to names
    competition_num = params.get("competition", DEFAULT_COMPETITION.value)
    try:
        competition_type = Competition(competition_num)
        competition = COMPETITION_NAMES[competition_type]
    except (ValueError, KeyError):
        valid_values = [e.value for e in Competition]
        raise ValueError(
            f"Invalid competition: {competition_num}. Must be one of {valid_values}"
        )
    logger.info(f"Running Competition: {competition}")

    scoring_param = params.get("scoring", DEFAULT_USE_SCORING)

    if isinstance(scoring_param, str):
        use_scoring = scoring_param.lower() == "true"
    else:
        use_scoring = bool(scoring_param)

    px4_path_raw = LaunchConfiguration("px4_path").perform(context)

    if px4_path_raw is None:
        raise RuntimeError("PX4 path is required")

    px4_path = os.path.expanduser(px4_path_raw)

    sae_ws_path = os.path.expanduser(os.getcwd())

    download_gz_models = ExecuteProcess(
        cmd=[
            "python3",
            "Tools/simulation/gz/simulation-gazebo",
            "--dryrun",
        ],
        cwd=px4_path,
        output="screen",
        name="download_gz_models",
    )

    spawn_world = ExecuteProcess(
        cmd=[
            "python3",
            "Tools/simulation/gz/simulation-gazebo",
            f"--world={competition}",
        ],
        cwd=px4_path,
        output="screen",
        name="spawn_world",
    )

    gz_ros_bridge_create = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[f"/world/{competition}/create@ros_gz_interfaces/srv/SpawnEntity"],
        remappings=[],
        output="screen",
        name="gz_ros_bridge_create",
        cwd=sae_ws_path,
    )

    model = LaunchConfiguration("model").perform(context)

    GZ_CAMERA_TOPIC = (
        f"/world/{competition}/model/{model[3:]}_0/link/camera_link/sensor/camera/image"
    )
    GZ_CAMERA_INFO_TOPIC = f"/world/{competition}/model/{model[3:]}_0/link/camera_link/sensor/camera/camera_info"

    gz_ros_bridge_camera = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[f"{GZ_CAMERA_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image"],
        remappings=[(GZ_CAMERA_TOPIC, "/camera")],
        output="screen",
        name="gz_ros_bridge_camera",
        cwd=sae_ws_path,
    )

    gz_ros_bridge_camera_info = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"{GZ_CAMERA_INFO_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        remappings=[(GZ_CAMERA_INFO_TOPIC, "/camera_info")],
        output="screen",
        name="gz_ros_bridge_camera_info",
        cwd=sae_ws_path,
    )

    sim_params, sim_config_path = load_sim_parameters(competition, logger)

    if "world" not in sim_params:
        raise ValueError(
            f"Missing 'world' section in simulation config: {sim_config_path}"
        )

    world_params = sim_params["world"].copy()

    # Initialize world node - it will generate the world file automatically
    world_node_name = camel_to_snake(world_params["name"])
    world = Node(
        package="sim",
        executable=camel_to_snake(world_params["name"]),
        arguments=[json.dumps(world_params["params"])],
        output="screen",
        name=world_node_name,
        cwd=sae_ws_path,
    )

    trigger_world_gen = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            f"/{world_node_name}/trigger_world_gen",
            "std_srvs/srv/Trigger",
        ],
        cwd=sae_ws_path,
        output="screen",
        name="trigger_world_gen",
    )

    # Initialize scoring node if requested
    scoring = None
    if use_scoring:
        if "scoring" not in sim_params:
            logger.warning(
                f"Scoring requested but no 'scoring' section in sim config: {sim_config_path}"
            )
        else:
            scoring = Node(
                package="sim",
                executable=camel_to_snake(sim_params["scoring"]["name"]),
                arguments=[sim_params["scoring"]["params"]],
                output="screen",
                name=sim_params["scoring"]["name"],
                cwd=sae_ws_path,
            )

    # Build and return the complete list of actions
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
                    gz_ros_bridge_camera,
                    gz_ros_bridge_camera_info,
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
            DeclareLaunchArgument("model", default_value="gz_x500_mono_cam"),
            OpaqueFunction(function=launch_setup),
        ]
    )
