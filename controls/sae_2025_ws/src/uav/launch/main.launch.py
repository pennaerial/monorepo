#!/usr/bin/env python3
import os
import yaml
import re

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            RegisterEventHandler, TimerAction, OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

GZ_CAMERA_TOPIC = '/world/custom/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
GZ_CAMERA_INFO_TOPIC = '/world/custom/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info'
HARDCODE_PATH = False

def find_folder(folder_name, search_path):
    for root, dirs, files in os.walk(search_path):
        if folder_name in dirs:
            return os.path.join(root, folder_name)
    return None

def find_folder_with_heuristic(folder_name, home_dir, keywords=('penn', 'air')):
    immediate_dirs = [d for d in os.listdir(home_dir) if os.path.isdir(os.path.join(home_dir, d))]
    if folder_name in immediate_dirs:
        return os.path.join(home_dir, folder_name)
    for d in immediate_dirs:
        if any(kw.lower() in d.lower() for kw in keywords):
            result = find_folder(folder_name, os.path.join(home_dir, d))
            if result:
                return result
    return find_folder(folder_name, home_dir)

def extract_vision_nodes(yaml_path):
    """
    Reads the YAML mission file, retrieves the corresponding vision node source files from
    os.getcwd()/src/uav/vision_nodes/, searches for imports from uav.vision_nodes,
    and returns a set of vision node class names.
    """
    vision_nodes = set()
    with open(yaml_path, 'r') as f:
        mission_config = yaml.safe_load(f)
    
    for mode, config in mission_config.items():
        class_path = config.get('class')
        if class_path:
            # Extract the class name from the fully qualified path.
            _, _, class_name = class_path.rpartition('.')
            # Build the file path assuming the file is located at:
            # os.getcwd()/src/uav/vision_nodes/{class_name}.py
            file_path = os.path.join(os.getcwd(), 'src', 'uav', 'uav', 'autonomous_modes', f"{class_name}.py")
            try:
                with open(file_path, 'r') as source_file:
                    source = source_file.read()
                    # Look for any "from uav.vision_nodes import ..." lines.
                    matches = re.findall(r'from\s+uav\.vision_nodes\s+import\s+([^\n]+)', source)
                    for match in matches:
                        # Allow for multiple imports on the same line, e.g., "A, B"
                        imported_nodes = [n.strip() for n in match.split(',')]
                        for node in imported_nodes:
                            if node:
                                vision_nodes.add(node)
            except Exception as e:
                print(f"Error processing {file_path}: {e}")
    return vision_nodes

def launch_setup(context, *args, **kwargs):
    # Resolve launch parameters from the context.
    mission_name = LaunchConfiguration('mission_name').perform(context)
    debug_str = LaunchConfiguration('debug').perform(context)
    # Convert the debug string to a boolean.
    debug = debug_str.lower() == 'true'
    
    # Build the YAML path using the resolved mission name.
    YAML_PATH = f'{os.getcwd()}/src/uav/uav/missions/{mission_name}.yaml'
    
    # Build vision node actions.
    vision_nodes = []
    vision_node_actions = [Node(
        package='uav',
        executable='camera',
        name='camera',
        output='screen'
    )]
    for node in extract_vision_nodes(YAML_PATH):
        vision_nodes.append(node)
        # Convert CamelCase node names to snake_case executable names.
        s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', node)
        exe_name = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()
        vision_node_actions.append(Node(
            package='uav',
            executable=exe_name,
            name=exe_name,
            output='screen',
            parameters=[{'debug': debug}]
        ))
    
    # If no vision nodes found, clear the actions.
    if len(vision_nodes) == 0:
        vision_node_actions = []
    
    # Find paths.
    px4_path = (find_folder_with_heuristic('PX4-Autopilot', os.path.expanduser('~'))
                if not HARDCODE_PATH else os.path.expanduser('~/PX4-Autopilot'))
    sae_ws_path = os.getcwd()
    
    # Define the processes.
    middleware = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        name='middleware'
    )
    
    gazebo = ExecuteProcess(
        cmd=['bash', 'standalone_gazebo_cmd.sh'],
        cwd=px4_path,
        output='screen',
        name='gazebo'
    )
    
    px4_sitl = ExecuteProcess(
        cmd=['bash', 'standalone_px4_cmd.sh'],
        cwd=px4_path,
        output='screen',
        name='px4_sitl'
    )
    
    gz_ros_bridge_camera = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             f'{GZ_CAMERA_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image',
             '--ros-args', '--remap', '/world/custom/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image:=/camera'],
        output='screen',
        cwd=sae_ws_path,
        name='gz_ros_bridge_camera'
    )
    
    gz_ros_bridge_camera_info = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             f'{GZ_CAMERA_INFO_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
             '--ros-args', '--remap', '/world/custom/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info:=/camera_info'],
        output='screen',
        cwd=sae_ws_path,
        name='gz_ros_bridge_camera_info'
    )
    
    mission = ExecuteProcess(
        cmd=['ros2', 'run', 'uav', 'mission', YAML_PATH, ','.join(vision_nodes)],
        output='screen',
        emulate_tty=True,
        name='mission'
    )
    
    delayed_mission = TimerAction(
        period=15.0,
        actions=[mission]
    )
    
    # Build and return the complete list of actions.
    return [
        *vision_node_actions,
        middleware,
        RegisterEventHandler(
            OnProcessStart(target_action=middleware, on_start=[gazebo, LogInfo(msg="Middleware started.")])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=gazebo, on_start=[px4_sitl, LogInfo(msg="Gazebo started.")])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=px4_sitl, on_start=[gz_ros_bridge_camera, LogInfo(msg="PX4 SITL started.")])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=gz_ros_bridge_camera, on_start=[gz_ros_bridge_camera_info, LogInfo(msg="gz_ros_bridge_camera started.")])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=gz_ros_bridge_camera_info, on_start=[delayed_mission, LogInfo(msg="gz_ros_bridge_camera_info started.")])
        ),
    ]

def generate_launch_description():
    # Declare launch arguments for mission_name and debug.
    mission_name_arg = DeclareLaunchArgument(
        'mission_name',
        default_value='basic',
        description='Name of the mission (used to select the mission YAML file)'
    )
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode (true or false)'
    )
    
    # Use an OpaqueFunction to set up the rest of the launch configuration
    # after resolving the launch parameters.
    return LaunchDescription([
        mission_name_arg,
        debug_arg,
        OpaqueFunction(function=launch_setup)
    ])
