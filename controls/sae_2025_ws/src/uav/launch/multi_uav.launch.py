#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    uav_share = get_package_share_directory('uav')
    main_launch = os.path.join(uav_share, 'launch', 'main.launch.py')

    # NOTE: set px4_path once here; both includes receive it
    px4_path = '~/Tools-users/PX4-Autopilot'

    uav0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main_launch),
        launch_arguments={
            'px4_path': px4_path,
            'vehicle_id': '0',
            'start_sim': 'true',
            'start_middleware': 'true',
            'start_vision': 'true',
        }.items()
    )

    uav1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main_launch),
        launch_arguments={
            'px4_path': px4_path,
            'vehicle_id': '1',
            'start_sim': 'false',
            'start_middleware': 'false',
            'start_vision': 'true',
        }.items()
    )

    return LaunchDescription([uav0, uav1])
