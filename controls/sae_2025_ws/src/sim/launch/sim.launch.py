#!/usr/bin/env python3

import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
    DeclareLaunchArgument,
    TimerAction,
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart

from sys import platform as sys_platform


def load_sim_launch_parameters():
    """Load simulation launch parameters from YAML."""
    params_file = os.path.join(
        os.getcwd(), "src", "sim", "launch", "launch_params.yaml"
    )
    with open(params_file, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    """Setup launch configuration."""
    # Get launch arguments
    params = load_sim_launch_parameters()
    competition = params.get("competition", 0)
    if competition == 0:
        competition = "in_house"
    elif competition == 1:
        competition = "iarc"
    elif competition == 2:
        competition = "custom"
    else:
        raise ValueError(f"Invalid competition: {competition}")

    competition_course = params.get("competition_course", 0)
    if competition_course == 0:
        competition_course = "ascent"
    elif competition_course == 1:
        competition_course = "descent"
    elif competition_course == 2:
        competition_course = "slalom"
    elif competition_course == 3:
        competition_course = "bezier"
    elif competition_course == 4:
        competition_course = "random"
    elif competition_course == 5:
        competition_course = "previous"
    else:
        raise ValueError(f"Invalid competition course: {competition_course}")

    model = LaunchConfiguration("model").perform(context)
    px4_path = LaunchConfiguration("px4_path").perform(context)
    use_scoring = LaunchConfiguration("use_scoring").perform(context)
    if use_scoring == "true":
        use_scoring = True
    else:
        use_scoring = False

    if "win" in sys_platform:
        platform = "x86"
    elif sys_platform == "linux":
        platform = "arm"
    else:
        raise ValueError(f"Invalid platform: {sys_platform}")

    if model is None or px4_path is None:
        raise RuntimeError("Model and PX4 path are required")

    sae_ws_path = os.path.expanduser(os.getcwd())

    spawn_world = ExecuteProcess(
        cmd=[
            "python3",
            "Tools/simulation/gz/simulation-gazebo",
            f"--world={competition}",
        ],
        cwd=px4_path,
        output="screen",
        name="launch world",
    )

    # Define the simulation process
    sim = Node(
        package="sim",
        executable="simulation",
        arguments=[competition, competition_course, use_scoring],
        output="screen",
        emulate_tty=True,
        name="sim",
    )

    topic_model_name = model[3:]  # remove 'gz_' prefix

    camera_topic_name = (
        "imager" if platform == "x86" else "camera"
    )  # windows -> 'imager'   mac -> 'camera'
    GZ_CAMERA_TOPIC = f"/world/custom/model/{topic_model_name}_0/link/camera_link/sensor/{camera_topic_name}/image"
    GZ_CAMERA_INFO_TOPIC = f"/world/custom/model/{topic_model_name}_0/link/camera_link/sensor/{camera_topic_name}/camera_info"

    gz_ros_bridge_camera = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[f"{GZ_CAMERA_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image"],
        remappings=[(GZ_CAMERA_TOPIC, "/camera")],
        output="screen",
        name="gz_ros_bridge_camera",
        cwd=sae_ws_path,
    )

    gz_ros_bridge_camera_info = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"{GZ_CAMERA_INFO_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        remappings=[(GZ_CAMERA_INFO_TOPIC, "/camera_info")],
        output="screen",
        name="gz_ros_bridge_camera_info",
        cwd=sae_ws_path,
    )

    # Build and return the complete list of actions
    return [
        spawn_world,
        RegisterEventHandler(
            OnProcessStart(
                target_action=spawn_world,
                on_start=[
                    gz_ros_bridge_camera,
                    gz_ros_bridge_camera_info,
                    LogInfo(msg="World Launched."),
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=gz_ros_bridge_camera,
                on_start=[sim, LogInfo(msg="Bridge camera topics started.")],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="gz_x500_mono_cam"),
            DeclareLaunchArgument("px4_path", default_value="~/PX4-Autopilot"),
            OpaqueFunction(function=launch_setup),
        ]
    )
