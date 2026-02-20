from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_param_file(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def launch_setup(context):
    payload_share_dir = get_package_share_directory("payload")
    payload_params_path = os.path.join(
        payload_share_dir, "config", "payload_params.yaml"
    )

    payload_name = LaunchConfiguration("payload_name").perform(context)

    payload_params = load_param_file(payload_params_path)

    payload = Node(
        package="payload",
        executable="payload",
        parameters=[payload_params_path],
        output="screen",
        name=payload_name,
    )

    actions = [
        payload,
    ]

    ros_params = payload_params.get("/**").get("ros__parameters")
    controller = ros_params.get("controller")
    # if controller == "GPIOController": #Real mode
    #     v4l2 = ExecuteProcess(
    #                 cmd=[
    #                     "ros2",
    #                     "run",
    #                     "v4l2_camera",
    #                     "v4l2_camera_node",
    #                     "--ros-args",
    #                     "-p",
    #                     "image_size:=[640,1600]",
    #                     "--ros-args",
    #                     "--remap",
    #                     f"/image_raw:=/{payload_name}/camera",
    #                 ],
    #                 output="screen",
    #                 name="cam2image",
    #             )
    #     actions.append(v4l2)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("payload_name", default_value="payload_0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
