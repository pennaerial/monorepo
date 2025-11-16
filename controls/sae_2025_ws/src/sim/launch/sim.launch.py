#!/usr/bin/env python3

import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart


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
        with open(params_file, 'r') as f:
            params = yaml.safe_load(f)
        return params
    except FileNotFoundError:
        print(f"Warning: {params_file} not found")

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
    model = LaunchConfiguration('model').perform(context)

    # Only override if not empty / None (usually always present because of defaults)
    if model is not None:
        params['model'] = model
    
    print(f"Launching {params['competition']} competition: with model {params['model']}")
    
    YAML_PATH = os.path.join(os.getcwd(), 'src', 'sim', 'sim', 'simulations', f"{params['competition']}.yaml")
    # Ensure output directory exists
    os.makedirs(os.path.dirname(os.path.expanduser(f"~/.simulation-gazebo/worlds/{params['competition']}.sdf")), exist_ok=True)
    print(os.path.dirname(os.path.expanduser(f"~/.simulation-gazebo/worlds/{params['competition']}.sdf")),)
    
    # Find required paths
    px4_path = find_folder_with_heuristic('PX4-Autopilot', os.path.expanduser('~'))
    if not px4_path:
        raise RuntimeError("PX4-Autopilot directory not found")
    
    sae_ws_path = os.getcwd()
    
    print("launch world")
    spawn_world = ExecuteProcess(
        cmd=['python3', 'Tools/simulation/gz/simulation-gazebo', f"--world={params['competition']}"],
        cwd=px4_path,
        output='screen',
        name='launch world'
    )
    
    # Define the simulation process
    uav_debug = 'False'
    sim = Node(
        package='sim',
        executable='simulation',
        arguments=[uav_debug, YAML_PATH, params['competition']],
        output='screen',
        emulate_tty=True,
        name='sim'
    )

    topic_model_name = params['model'][3:] # remove 'gz_' prefix
    GZ_CAMERA_TOPIC = f'/world/custom/model/{topic_model_name}_0/link/camera_link/sensor/camera/image'
    GZ_CAMERA_INFO_TOPIC = f'/world/custom/model/{topic_model_name}_0/link/camera_link/sensor/camera/camera_info'

    gz_ros_bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'{GZ_CAMERA_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        remappings=[
            (GZ_CAMERA_TOPIC, '/camera')
        ],
        output='screen',
        name='gz_ros_bridge_camera',
        cwd=sae_ws_path
    )
    
    gz_ros_bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'{GZ_CAMERA_INFO_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        remappings=[
            (GZ_CAMERA_INFO_TOPIC, '/camera_info')
        ],
        output='screen',
        name='gz_ros_bridge_camera_info',
        cwd=sae_ws_path
    )
    
    # Build and return the complete list of actions
    return [
        spawn_world,
        RegisterEventHandler(
            OnProcessStart(target_action=spawn_world, on_start=[gz_ros_bridge_camera, gz_ros_bridge_camera_info, LogInfo(msg="World Launched.")])
        ),      
        RegisterEventHandler( 
            OnProcessStart(target_action=gz_ros_bridge_camera, on_start=[sim, LogInfo(msg="Bridge camera topics started.")])
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
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='gz_x500_mono_cam'),
        OpaqueFunction(function=launch_setup)
    ])
