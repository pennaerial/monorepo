#!/usr/bin/env python3
"""
Launch multiple UAVs in simulation by including main.launch.py once per vehicle.

- vehicle_id=0: starts Gazebo (sim), first PX4, MicroXRCEAgent, vision, mission.
- vehicle_id>=1: starts only that PX4 (connects to existing Gazebo), vision, mission.

Usage:
  ros2 launch uav multi_uav.launch.py
  ros2 launch uav multi_uav.launch.py num_vehicles:=3
  ros2 launch uav multi_uav.launch.py num_vehicles:=2 px4_path:=/path/to/PX4-Autopilot
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def _launch_multiple(context, *args, **kwargs):
    num_vehicles = int(LaunchConfiguration("num_vehicles", default="2").perform(context))
    px4_path = LaunchConfiguration("px4_path", default="~/Tools-users/PX4-Autopilot").perform(context)

    if num_vehicles < 1:
        raise ValueError("num_vehicles must be >= 1")

    uav_share = get_package_share_directory("uav")
    main_launch = os.path.join(uav_share, "launch", "main.launch.py")

    actions = []
    for vehicle_id in range(num_vehicles):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(main_launch),
                launch_arguments={
                    "px4_path": px4_path,
                    "vehicle_id": str(vehicle_id),
                }.items(),
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "num_vehicles",
                default_value="2",
                description="Number of UAVs to launch (vehicle_id 0..num_vehicles-1).",
            ),
            DeclareLaunchArgument(
                "px4_path",
                default_value="~/Tools-users/PX4-Autopilot",
                description="Path to PX4-Autopilot directory.",
            ),
            OpaqueFunction(function=_launch_multiple),
        ]
    )
