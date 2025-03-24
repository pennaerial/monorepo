#!/usr/bin/env python3
import os
import yaml
import re

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction, OpaqueFunction
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
    Reads the mission YAML file, retrieves the vision node source files from
    os.getcwd()/src/uav/uav/autonomous_modes/, searches for imports from uav.vision_nodes,
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
            # os.getcwd()/src/uav/uav/autonomous_modes/{class_name}.py
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

def load_launch_parameters():
    params_file = os.path.join(os.getcwd(), 'src', 'uav', 'launch', 'launch_params.yaml')
    with open(params_file, 'r') as f:
         return yaml.safe_load(f)

def launch_setup(context, *args, **kwargs):
    # Load launch parameters from the YAML file.
    params = load_launch_parameters()
    mission_name = params.get('mission_name', 'basic')
    uav_debug = str(params.get('uav_debug', 'false'))
    vision_debug = str(params.get('vision_debug', 'false'))
    sim = str(params.get('sim', 'false'))
    
    # Convert debug and simulation flags to booleans.
    vision_debug_bool = vision_debug.lower() == 'true'
    sim_bool = sim.lower() == 'true'
    
    # Build the mission YAML file path using the mission name.
    YAML_PATH = os.path.join(os.getcwd(), 'src', 'uav', 'uav', 'missions', f"{mission_name}.yaml")
    
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
            parameters=[{'debug': vision_debug_bool}]
        ))
    
    # Clear vision node actions if none are found.
    if len(vision_nodes) == 0:
        vision_node_actions = []
    
    # Find required paths.
    px4_path = (find_folder_with_heuristic('PX4-Autopilot', os.path.expanduser('~'))
                if not HARDCODE_PATH else os.path.expanduser('~/PX4-Autopilot'))
    sae_ws_path = os.getcwd()
    
    # Define the middleware process.
    middleware = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        name='middleware'
    )
    
    # Define the Gazebo process.
    gazebo = ExecuteProcess(
        cmd=['bash', 'standalone_gazebo_cmd.sh'],
        cwd=px4_path,
        output='screen',
        name='gazebo'
    )
    
    # Define the PX4 SITL process.
    px4_sitl = ExecuteProcess(
        cmd=['bash', 'standalone_px4_cmd.sh'],
        cwd=px4_path,
        output='screen',
        name='px4_sitl'
    )
    
    # Define the ROS-Gazebo bridge for the camera topics.
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
    
    # Define the mission process.
    mission_cmd = ['ros2', 'run', 'uav', 'mission', uav_debug, YAML_PATH, ','.join(vision_nodes)]
    if not sim_bool:
         mission_cmd += ['--ros-args', '-r', '/image_raw:=/camera']
    mission = ExecuteProcess(
        cmd=mission_cmd,
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
            OnProcessStart(target_action=middleware, on_start=[gazebo if sim_bool else delayed_mission, LogInfo(msg="Middleware started.")])
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
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
