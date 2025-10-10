#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


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


def launch_setup(context, *args, **kwargs):
    """Setup launch configuration."""
    
    print("Starting slalom competition simulation...")
    
    # Find required paths
    px4_path = find_folder_with_heuristic('PX4-Autopilot', os.path.expanduser('~'))
    if not px4_path:
        raise RuntimeError("PX4-Autopilot directory not found")
    
    sae_ws_path = os.getcwd()
    
    # Define the middleware process
    middleware = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        name='middleware'
    )
    
    # Define the Gazebo process
    gazebo = ExecuteProcess(
        cmd=['python3', 'Tools/simulation/gz/simulation-gazebo', '--world=custom'],
        cwd=px4_path,
        output='screen',
        name='gazebo'
    )
    
    # Define the PX4 SITL process
    px4_sitl = ExecuteProcess(
        cmd=['bash', '-c', 'PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_mono_cam_down PX4_GZ_WORLD=custom ./build/px4_sitl_default/bin/px4'],
        cwd=px4_path,
        output='screen',
        name='px4_sitl'
    )
    
    # Define the ROS-Gazebo bridge for camera topics
    gz_ros_bridge_camera = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/custom/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '--ros-args', '--remap', '/world/custom/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image:=/camera'],
        output='screen',
        cwd=sae_ws_path,
        name='gz_ros_bridge_camera'
    )
    
    gz_ros_bridge_camera_info = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/custom/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '--ros-args', '--remap', '/world/custom/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info:=/camera_info'],
        output='screen',
        cwd=sae_ws_path,
        name='gz_ros_bridge_camera_info'
    )
    
    # Build and return the complete list of actions
    return [
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
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
