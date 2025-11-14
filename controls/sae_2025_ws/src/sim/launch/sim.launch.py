#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from sim.in_house.worldgen import generate_world


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
    
    try:
        with open(params_file, 'r') as f:
            params = yaml.safe_load(f)
        return params
    except FileNotFoundError:
        print(f"Warning: {params_file} not found, using default parameters")
        return get_default_params()


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
    
    sae_ws_path = os.getcwd()
    
    # Define the middleware process
    print("start middleware")
    middleware = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', str(ros_params['middleware_port'])],
        output='screen',
        name='middleware'
    )
    
    # Define the Gazebo process
    print("start gazebo")
    gazebo = ExecuteProcess(
        cmd=['python3', 'Tools/simulation/gz/simulation-gazebo', f'--world={world_name}'],
        cwd=px4_path,
        output='screen',
        name='gazebo'
    )
    
    # Define the PX4 SITL process
    print("start px4_sitl")
    px4_sitl = ExecuteProcess(
        cmd=['bash', '-c', f'PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL={sim_params["uav_model"]} PX4_GZ_WORLD={world_name} ./build/px4_sitl_default/bin/px4'],
        cwd=px4_path,
        output='screen',
        name='px4_sitl'
    )
    
    
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
    return [
        middleware,
        RegisterEventHandler(
            OnProcessStart(target_action=middleware, on_start=[sim, LogInfo(msg="Middleware started.")])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=sim, on_start=[gazebo, LogInfo(msg="Sim node started.")])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=gazebo, on_start=[px4_sitl, LogInfo(msg="Gazebo started.")])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=px4_sitl, on_start=bridge_actions + [LogInfo(msg="PX4 SITL started.")])
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
