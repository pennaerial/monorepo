#!/usr/bin/env python3

import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart


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
    px4_path = LaunchConfiguration('px4_path').perform(context)

    if model is None or px4_path is None:
        raise RuntimeError("Model and PX4 path are required")

    sae_ws_path = os.getcwd()
    
    YAML_PATH = os.path.join(sae_ws_path, 'src', 'sim', 'sim', 'simulations', f"{params['competition']}.yaml")
    # Ensure output directory exists
    os.makedirs(os.path.dirname(os.path.expanduser(f"~/.simulation-gazebo/worlds/{params['competition']}.sdf")), exist_ok=True)
    
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

    topic_model_name = model[3:] # remove 'gz_' prefix
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
        LogInfo(msg=f"Launching {params['competition']} competition: with model {model}"),
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
        DeclareLaunchArgument('px4_path', default_value='~/PX4-Autopilot'),
        OpaqueFunction(function=launch_setup)
    ])
