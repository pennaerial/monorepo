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
from sim.utils import find_package_resource, load_yaml_to_dict, build_node_arguments, camel_to_snake
import importlib
from pathlib import Path
from sim.constants import (
    Competition, 
    COMPETITION_NAMES, 
    DEFAULT_COMPETITION,
    DEFAULT_USE_SCORING
)
import json


def find_folder_with_heuristic(folder_name, home_dir, keywords=('penn', 'air')):
    """Find folder using heuristic search."""
    immediate_dirs = [d for d in os.listdir(home_dir) if os.path.isdir(os.path.join(home_dir, d))]
    if folder_name in immediate_dirs:
        return os.path.join(home_dir, folder_name)
    for d in immediate_dirs:
        if any(kw.lower() in d.lower() for kw in keywords):
            result = find_folder(folder_name, os.path.join(home_dir, d))
            if result:
                return result
    return find_folder(folder_name, home_dir)


def find_folder(folder_name, search_path):
    """Find folder recursively."""
    for root, dirs, files in os.walk(search_path):
        if folder_name in dirs:
            return os.path.join(root, folder_name)
    return None


def load_launch_params():
    """Load parameters from launch_params.yaml file."""
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    params_file = os.path.join(launch_file_dir, 'launch_params.yaml')
    
    for path in source_paths:
        if path.exists():
            return load_yaml_to_dict(path)
    
    # Fallback to installed location (for production/deployed packages)
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share = Path(get_package_share_directory('sim'))
        installed_params = package_share / 'launch' / 'launch_params.yaml'
        if installed_params.exists():
            return load_yaml_to_dict(installed_params)
    except Exception:
        pass
    
    # If not found, raise error
    raise FileNotFoundError(
        f"Launch params file not found. Checked source paths: {source_paths} "
        f"and installed location."
    )

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
        package_name='sim',
        resource_type='file',
        logger=logger,
        base_file=Path(__file__)
    )
    return load_yaml_to_dict(config_path), config_path

def launch_setup(context, *args, **kwargs):
    """Setup launch configuration."""
    
    # Load parameters from YAML file
    params = load_launch_params()
    
    # Use YAML values directly
    competition_type = params['competition']['type']
    
    # Extract simulation parameters
    sim_params = params['simulation']
    ros_params = params['ros2']
    
    print(f"Launching {params['competition']['type']} competition: {params['competition']['name']}")
    
    # Generate world file using worldgen.py
    world_name = f"{params['competition']['type']}_{params['competition']['name']}"
    

    # Ensure output directory exists
    os.makedirs(os.path.dirname(course_params_obj.op_file), exist_ok=True)
    
    # Find required paths
    px4_path = find_folder_with_heuristic('PX4-Autopilot', os.path.expanduser('~'))
    if not px4_path:
        raise RuntimeError("PX4-Autopilot directory not found")
    
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

    sim_params, sim_config_path = load_sim_parameters(competition, logger)

    if "world" not in sim_params:
        raise ValueError(f"Missing 'world' section in simulation config: {sim_config_path}")
    
    world_params = sim_params["world"].copy()
    
    # Define the simulation process

    sim_cmd = ['ros2', 'run', 'sim', 'simulation', uav_debug, YAML_PATH, world_name]
    sim = ExecuteProcess(
        cmd=sim_cmd,
        output='screen',
        emulate_tty=True,
        name='mission'
    )
    
    # Define the ROS-Gazebo bridge for camera topics (only if enabled)
    gz_ros_bridge_camera = None
    gz_ros_bridge_camera_info = None
    
    if ros_params.get('enable_camera_bridge', True):
        camera_topic = ros_params['topics']['camera']
        camera_info_topic = ros_params['topics']['camera_info']
        
        print("start gz_ros_bridge_camera")
        gz_ros_bridge_camera = ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'/world/{world_name}/model/{sim_params["uav_model"]}_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '--ros-args', '--remap', f'/world/{world_name}/model/{sim_params["uav_model"]}_0/link/camera_link/sensor/imager/image:={camera_topic}'],
            output='screen',
            cwd=sae_ws_path,
            name='gz_ros_bridge_camera'
        )
        
        gz_ros_bridge_camera_info = ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'/world/{world_name}/model/{sim_params["uav_model"]}_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '--ros-args', '--remap', f'/world/{world_name}/model/{sim_params["uav_model"]}_0/link/camera_link/sensor/imager/camera_info:={camera_info_topic}'],
            output='screen',
            cwd=sae_ws_path,
            name='gz_ros_bridge_camera_info'
        )
    
    # Delayed scoring node start (only if scoring is enabled)
    # delayed_scoring = None
    # if scoring_node is not None:
    #     delayed_scoring = TimerAction(
    #         period=10.0,
    #         actions=[scoring_node]
    #     )
    
    # Build action list based on enabled features
    bridge_actions = []
    if gz_ros_bridge_camera is not None:
        bridge_actions.append(gz_ros_bridge_camera)
    if gz_ros_bridge_camera_info is not None:
        bridge_actions.append(gz_ros_bridge_camera_info)
    
    # # Add delayed scoring if enabled
    # if delayed_scoring is not None:
    #     bridge_actions.append(delayed_scoring)
    
    # Build and return the complete list of actions
    actions = [
        download_gz_models,
        RegisterEventHandler(
            OnProcessStart(target_action=middleware, on_start=[sim, LogInfo(msg="Middleware started.")])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=sim, on_start=[gazebo, LogInfo(msg="Sim node started.")])
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=world,
                on_stderr=lambda event: (
                    [spawn_world, LogInfo(msg="Simulation world node started."), scoring] if b"Successfully generated world file:" in event.text else None
                )
            )
        )
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
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
