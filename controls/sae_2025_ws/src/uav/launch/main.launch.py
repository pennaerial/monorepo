#!/usr/bin/env python3
import os
import yaml
import re

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, TimerAction, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from uav.utils import vehicle_map
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

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
    run_mission = str(params.get('run_mission', 'true'))
    vehicle_type = vehicle_map[params.get('vehicle_type', 0)]
    save_vision = str(params.get('save_vision', 'false'))
    camera_offsets = params.get('camera_offsets', [0, 0, 0])
    servo_only = str(params.get('servo_only', 'false'))
    
    # Convert debug and simulation flags to booleans.
    vision_debug_bool = vision_debug.lower() == 'true'
    sim_bool = sim.lower() == 'true'
    run_mission_bool = run_mission.lower() == 'true'
    save_vision_bool = save_vision.lower() == 'true'
    
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
            parameters=[{'debug': vision_debug_bool, 'sim': sim_bool, 'save_vision': save_vision_bool}],
        ))
    
    # Clear vision node actions if none are found.
    if len(vision_nodes) == 0:
        vision_node_actions = []
    
    if not sim_bool:
        vision_node_actions.insert(0, ExecuteProcess(
            cmd=['ros2', 'run', 'v4l2_camera', 'v4l2_camera_node', '--ros-args', '-p', 'image_size:=[640,480]', '--ros-args', '--remap', '/image_raw:=/camera'],
            output='screen',
            name='cam2image'
        ))
    
    # Find required paths.
    px4_path = (find_folder_with_heuristic('PX4-Autopilot', os.path.expanduser('~'))
                if not HARDCODE_PATH else os.path.expanduser('~/PX4-Autopilot'))
    
    # Define the middleware process.
    middleware = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'] if sim_bool else ['MicroXRCEAgent', 'serial', '--dev', '/dev/serial0', '-b', '921600'],
        output='screen',
        name='middleware'
    )
    
    # Define the PX4 SITL process.
    if vehicle_type == 'quadcopter':
        autostart = 4001
        model = 'gz_x500_mono_cam' # append '_down' for down-facing camera
    elif vehicle_type == 'tiltrotor_vtol':
        autostart = 4020
        model = 'gz_tiltrotor'
    elif vehicle_type == 'fixed_wing':
        autostart = 4003
        model = 'gz_rc_cessna'
    elif vehicle_type == 'standard_vtol':
        autostart = 4004
        model = 'gz_standard_vtol'
    else:
        raise ValueError(f"Invalid vehicle type: {vehicle_type}")

    # Define the Gazebo process.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sim'),
                'launch',
                'sim.launch.py'
            )
        ),
        launch_arguments={'model': model, 'px4_path': px4_path}.items()
    )
    
    px4_sitl = ExecuteProcess(
        cmd=['bash', '-c', f'PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART={autostart} PX4_SIM_MODEL={model} ./build/px4_sitl_default/bin/px4'],
        cwd=px4_path,
        output='screen',
        name='px4_sitl'
    )    
    
    # Define the mission process.
    camera_offsets_str = ','.join(str(offset) for offset in camera_offsets)
    mission_cmd = ['ros2', 'run', 'uav', 'mission', uav_debug, YAML_PATH, servo_only, camera_offsets_str, ','.join(vision_nodes)]
    mission = ExecuteProcess(
        cmd=mission_cmd,
        output='screen',
        emulate_tty=True,
        name='mission'
    )
    
    # Build and return the complete list of actions.
    return [
        sim,
        LogInfo(msg="Gazebo started."),
        *vision_node_actions,
        LogInfo(msg="Vision nodes started."),
        middleware,
        RegisterEventHandler(
            OnProcessStart(target_action=middleware, on_start=[LogInfo(msg="Middleware started."), px4_sitl])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=px4_sitl, on_start=[LogInfo(msg="PX4 SITL started."), TimerAction(period=15.0, actions=[mission] if run_mission_bool else [])])
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
