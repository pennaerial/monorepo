#!/usr/bin/env python3
"""
Launch multiple UAVs from a YAML config (vehicle count and types).

Reads multi_uav_params.yaml (or path given by multi_uav_config) for:
  - px4_path: path to PX4-Autopilot
  - vehicles: list of { type: <airframe name or ID> } (one per vehicle)

Gazebo (sim.launch.py) is launched once up-front, then each vehicle is started
via main.launch.py with launch_sim:=false.  Camera bridges are created per-vehicle
inside main.launch.py.

Usage:
  ros2 launch uav multi_uav.launch.py
  ros2 launch uav multi_uav.launch.py multi_uav_config:=/path/to/multi_uav_params.yaml
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def _resolve_config_path(context):
    """Resolve path to multi_uav_params.yaml: launch arg, then package share, then cwd."""
    config_arg = LaunchConfiguration("multi_uav_config", default="").perform(context)
    if config_arg and os.path.isfile(os.path.expanduser(config_arg)):
        return os.path.expanduser(config_arg)
    try:
        uav_share = get_package_share_directory("uav")
        path = os.path.join(uav_share, "launch", "multi_uav_params.yaml")
        if os.path.isfile(path):
            return path
    except Exception:
        pass
    cwd_path = os.path.join(os.getcwd(), "src", "uav", "launch", "multi_uav_params.yaml")
    if os.path.isfile(cwd_path):
        return cwd_path
    raise FileNotFoundError(
        "multi_uav_params.yaml not found. Set multi_uav_config:=/path/to/multi_uav_params.yaml "
        "or run from workspace root with launch/multi_uav_params.yaml present."
    )


def _load_multi_uav_params(config_path):
    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    vehicles = data.get("vehicles")
    if not vehicles:
        raise ValueError("multi_uav_params.yaml must contain a non-empty 'vehicles' list")
    px4_path = data.get("px4_path", "~/Tools-users/PX4-Autopilot")
    return vehicles, os.path.expanduser(px4_path)


def _launch_multiple(context, *args, **kwargs):
    config_path = _resolve_config_path(context)
    vehicles, px4_path = _load_multi_uav_params(config_path)

    sim_share = get_package_share_directory("sim")
    uav_share = get_package_share_directory("uav")
    sim_launch = os.path.join(sim_share, "launch", "sim.launch.py")
    main_launch = os.path.join(uav_share, "launch", "main.launch.py")

    actions = []

    # --- 1. Launch Gazebo environment (no per-vehicle camera bridges) ---
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "px4_path": px4_path,
                "create_bridges": "false",
            }.items(),
        )
    )

    # --- 2. Launch each vehicle (PX4 + bridges + vision + mission) ---
    for vehicle_id, vehicle_spec in enumerate(vehicles):
        if isinstance(vehicle_spec, dict):
            vehicle_type = vehicle_spec.get("type")
        else:
            vehicle_type = vehicle_spec
        if vehicle_type is None:
            raise ValueError(f"Vehicle {vehicle_id}: missing 'type' (airframe name or ID)")
        airframe_str = str(vehicle_type)

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(main_launch),
                launch_arguments={
                    "px4_path": px4_path,
                    "vehicle_id": str(vehicle_id),
                    "airframe": airframe_str,
                    "launch_sim": "false",
                }.items(),
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "multi_uav_config",
                default_value="",
                description="Path to multi_uav_params.yaml (default: package launch/multi_uav_params.yaml).",
            ),
            OpaqueFunction(function=_launch_multiple),
        ]
    )
