from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context):
    domain_id = LaunchConfiguration('domain_id').perform(context)
    incoming_name = LaunchConfiguration('incoming_name').perform(context)
    hooked_name = LaunchConfiguration('hooked_name').perform(context)
    controller_override = LaunchConfiguration('controller').perform(context)

    payload_share_dir = get_package_share_directory('payload')
    payload_params_path = os.path.join(
        payload_share_dir, 'config', 'payload_params.yaml')

    incoming_params = [payload_params_path]
    hooked_params = [payload_params_path]
    if controller_override:
        incoming_params.append({'controller': controller_override})
        hooked_params.append({'controller': controller_override})

    actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain_id),

        Node(
            package='payload',
            executable='payload',
            name=incoming_name,
            parameters=incoming_params,
            output='screen',
        ),

        Node(
            package='payload',
            executable='payload',
            name=hooked_name,
            parameters=hooked_params,
            output='screen',
        ),

        Node(
            package='payload',
            executable='payload_agent',
            name=f'{incoming_name}_agent',
            parameters=[{'payload_name': incoming_name, 'role': 'incoming'}],
            output='screen',
        ),

        Node(
            package='payload',
            executable='payload_agent',
            name=f'{hooked_name}_agent',
            parameters=[{'payload_name': hooked_name, 'role': 'hooked'}],
            output='screen',
        ),
    ]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('domain_id', default_value='0',
                              description='ROS_DOMAIN_ID for DDS partitioning'),
        DeclareLaunchArgument('incoming_name', default_value='payload_incoming',
                              description='Node name for the incoming payload'),
        DeclareLaunchArgument('hooked_name', default_value='payload_hooked',
                              description='Node name for the hooked payload'),
        DeclareLaunchArgument('controller', default_value='',
                              description='Controller override (e.g. SimController)'),
        OpaqueFunction(function=launch_setup),
    ])
