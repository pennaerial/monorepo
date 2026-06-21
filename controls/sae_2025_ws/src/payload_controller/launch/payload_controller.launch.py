from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    payload_share_dir = get_package_share_directory("payload_controller")
    payload_params_path = os.path.join(
        payload_share_dir, "config", "payload_controller_params.yaml"
    )

    vehicle_name = LaunchConfiguration("vehicle_name").perform(context)
    sim_entity_name = LaunchConfiguration("sim_entity_name").perform(context).strip()
    controller_override = LaunchConfiguration("controller").perform(context)
    payload_node_name = sim_entity_name or vehicle_name

    parameters = [payload_params_path]
    if controller_override:
        parameters.append({"controller": controller_override})

    payload = Node(
        package="payload_controller",
        executable="payload_controller",
        parameters=parameters,
        output="screen",
        name=payload_node_name,
        namespace=vehicle_name,
    )

    actions = [payload]

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle_name", default_value="payload_0"),
            DeclareLaunchArgument("sim_entity_name", default_value=""),
            DeclareLaunchArgument("controller", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
