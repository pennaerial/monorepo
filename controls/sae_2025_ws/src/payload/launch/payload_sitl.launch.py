from enum import StrEnum


from ament_index_python import get_package_share_path

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable
from launch import LaunchDescription, Action
from launch.actions import (
    LogInfo,
    OpaqueFunction,
    DeclareLaunchArgument
)

from vehicle_common.launch_utils import (
    get_logger,
    LaunchError,
    check_unknown_launch_args,
    include_launch,
    format_bullet_list,
    is_truthy,
)

logger = get_logger("uav_sitl.launch")


class Args(StrEnum):
    """Maps constants to launch argument keyords"""
    
    NS_ID = "ns_id"
    LAUNCH_RVIZ = "launch_rviz"

def launch_setup(context) -> list[Action]:
    config = context.launch_configurations

    check_unknown_launch_args(Args, config, logger)

    ns_id = int(config[Args.NS_ID])
    vehicle_ns = f"payload_{ns_id}"
    launch_rviz = is_truthy(config[Args.LAUNCH_RVIZ])

    payload_share = get_package_share_path("payload")
    xacro_path = payload_share / "urdf" / "payload.urdf.xacro"

    # Feed in string-valued ROS parameter "xacro <path>/payload.urdf.xacro"
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", str(xacro_path)])
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=vehicle_ns,
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False, # True to keep tf synched with sim, False rn temporarily before sim
            }
        ],
        output="screen"
    )

    temporary_payload_pose = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="temporary_payload_pose",
        namespace=vehicle_ns,
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
        ],
        output="screen",
    )

    actions = [
        LogInfo(msg=f"Launching RViz prototype for {vehicle_ns}"),
        robot_state_publisher,
        temporary_payload_pose,
    ]

    rviz_config_path = payload_share / "rviz" / "temp_payload.rviz"

    if launch_rviz:
        rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[
                {
                    "use_sim_time": False,
                }
            ],
            output="screen",
            arguments=["-d", str(rviz_config_path)]
        )
        actions.append(rviz)

    return actions



def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                Args.NS_ID,
                default_value="0",
                description="Integer namespace id for the vehicle. An id of 0 makes the namespace uav_0. Correponds to the ROS node namespace and PX4 SITL namespace",
            ),
            DeclareLaunchArgument(
                Args.LAUNCH_RVIZ,
                default_value="true",
                description="If true, launch RViz.",
                choices=["true", "false", "t", "f", "0", "1"]
            ),
            OpaqueFunction(function=launch_setup),
            LogInfo(msg="payload_sitl.launch.py loaded successfully"),
        ]
    )