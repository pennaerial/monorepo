#!/usr/bin/env python3
import os
import yaml
import re

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
# from  uav.utils import camel_to_snake #TODO: investigate why importing this breaks gazebo

HARDCODE_PATH = False
YAML_PATH = f'{os.getcwd()}/src/uav/uav/missions/basic_payload_landing.yaml'
GZ_CAMERA_TOPIC = '/world/custom/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
GZ_CAMERA_INFO_TOPIC = '/world/custom/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info'

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
            # Example: if class_path is "uav.vision_nodes.SomeVisionNode", then class_name is "SomeVisionNode".
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


def generate_launch_description():
    px4_path = (find_folder_with_heuristic('PX4-Autopilot', os.path.expanduser('~'))
                if not HARDCODE_PATH else os.path.expanduser('~/PX4-Autopilot'))
    sae_ws_path = os.getcwd()

    vision_nodes = []
    vision_node_actions = [Node(
        package='uav',
        executable='camera',
        name='camera',
        output='screen'
    )]
    for node in extract_vision_nodes(YAML_PATH):
        vision_nodes.append(node)

        s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', node)
        exe_name =  re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()
        vision_node_actions.append(Node(
            package='uav',
            executable=exe_name,
            name=exe_name,
            output='screen'
        ))

    if len(vision_nodes) == 0:
        vision_node_actions = []

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

    # Build the launch description.
    ld = LaunchDescription([
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
    ])

    return ld