from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    payload_share_dir = get_package_share_directory("payload")
    payload_params_path = os.path.join(
        payload_share_dir, "config", "payload_params.yaml"
    )

    payload_name = LaunchConfiguration("payload_name").perform(context)
    controller_override = LaunchConfiguration("controller").perform(context)

    parameters = [payload_params_path]
    if controller_override:
        parameters.append({"controller": controller_override})

    payload = Node(
        package="payload",
        executable="payload",
        parameters=parameters,
        output="screen",
        name=payload_name,
    )

    actions = [payload]

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("payload_name", default_value="payload_0"),
            DeclareLaunchArgument("controller", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
