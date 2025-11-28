#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, OpaqueFunction, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart


def launch_setup(context, *args, **kwargs):
    """Setup launch configuration."""
    # Get launch arguments
    model = LaunchConfiguration('model').perform(context)
    px4_path = LaunchConfiguration('px4_path').perform(context)
    competition = LaunchConfiguration('competition').perform(context)
    enable_scoring = LaunchConfiguration('enable_scoring').perform(context)
    generation_type = LaunchConfiguration('generation_type').perform(context)
    platform = LaunchConfiguration('platform').perform(context)

    print(f'PLATFORM: {platform}')
    if not platform:
        raise RuntimeError("The 'platform' argument is required (x86 or arm).")

    if model is None or px4_path is None:
        raise RuntimeError("Model and PX4 path are required")
    
    # Expand user home directory path (e.g., ~/PX4-Autopilot -> /home/user/PX4-Autopilot)
    px4_path = os.path.expanduser(px4_path)
    
    # Default competition if not provided
    if competition is None or competition == '':
        competition = 'in_house'
    
    sae_ws_path = os.getcwd()
    YAML_PATH = os.path.join(sae_ws_path, 'src', 'sim', 'sim', 'simulations', f"{competition}.yaml")
        
    spawn_world = ExecuteProcess(
        cmd=['python3', 'Tools/simulation/gz/simulation-gazebo', f"--world={competition}"],
        cwd=px4_path,
        output='screen',
        name='launch world'
    )
    
    # Define the simulation process
    uav_debug = 'False'
    sim = Node(
        package='sim',
        executable='simulation',
        arguments=[uav_debug, YAML_PATH, competition],
        output='screen',
        emulate_tty=True,
        name='sim'
    )

    topic_model_name = model[3:] # remove 'gz_' prefix

    camera_topic_name = 'imager' if platform == 'x86' else 'camera'  #windows -> 'imager'   mac -> 'camera'
    GZ_CAMERA_TOPIC = f'/world/custom/model/{topic_model_name}_0/link/camera_link/sensor/{camera_topic_name}/image'
    GZ_CAMERA_INFO_TOPIC = f'/world/custom/model/{topic_model_name}_0/link/camera_link/sensor/{camera_topic_name}/camera_info'


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
        LogInfo(msg=f"Launching {competition} competition: with model {model}"),
        spawn_world,
        RegisterEventHandler(
            OnProcessStart(target_action=spawn_world, on_start=[gz_ros_bridge_camera, gz_ros_bridge_camera_info, LogInfo(msg="World Launched.")])
        ),      
        RegisterEventHandler( 
            OnProcessStart(target_action=gz_ros_bridge_camera, on_start=[sim, LogInfo(msg="Bridge camera topics started.")])
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='gz_x500_mono_cam'),
        DeclareLaunchArgument('px4_path', default_value='~/PX4-Autopilot'),
        DeclareLaunchArgument('competition', default_value='in_house', description='Competition name (e.g., in_house, iarc, custom)'),
        DeclareLaunchArgument('enable_scoring', default_value='true', description='Enable scoring node'),
        DeclareLaunchArgument('generation_type', default_value='', description='Generation type (optional)'),
        DeclareLaunchArgument('platform', description='Target platform (x86 or arm)'),
        OpaqueFunction(function=launch_setup)
    ])
