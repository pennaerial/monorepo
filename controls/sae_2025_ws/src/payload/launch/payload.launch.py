from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context):
    payload_share_dir = get_package_share_directory("payload")
    payload_params = os.path.join(payload_share_dir, "config", "payload_params.yaml")


    payload_name = LaunchConfiguration('payload_name').perform(context)
    payload = Node(
        package="payload",
        executable="payload",
        parameters=[payload_params],
        output="screen",
        name=payload_name,
    )

    actions = [
        payload,
    ]

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('payload_name', default_value='payload_0'),
        OpaqueFunction(function=launch_setup),
    ])