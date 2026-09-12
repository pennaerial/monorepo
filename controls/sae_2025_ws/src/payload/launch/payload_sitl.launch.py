from enum import StrEnum


from ament_index_python import get_package_share_path

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable
from launch import LaunchDescription, Action
from launch.actions import OpaqueFunction, DeclareLaunchArgument

from vehicle_common.launch_utils import (
    get_logger,
    check_unknown_launch_args,
    include_launch,
    is_truthy,
)

logger = get_logger("uav_sitl.launch")


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    NS_ID = "ns_id"
    LAUNCH_RVIZ = "launch_rviz"
    WORLD = "world"
    LAUNCH_SIM = "launch_sim"
    HEADLESS = "headless"


def launch_setup(context) -> list[Action]:
    config = context.launch_configurations

    check_unknown_launch_args(Args, config, logger)

    ns_id = int(config[Args.NS_ID])
    vehicle_ns = f"payload_{ns_id}"
    launch_rviz = is_truthy(config[Args.LAUNCH_RVIZ])
    world = config[Args.WORLD]
    launch_sim = is_truthy(config[Args.LAUNCH_SIM])
    headless = config[Args.HEADLESS]

    # PRINTING HEADER
    logger.debug("LAUNCH PARAMS")
    # logger.debug(f"Mission:             {mission}")
    logger.debug(f"Vehicle Namespace:   {vehicle_ns}")
    logger.debug(f"Sim World:           {world}")
    # logger.debug(f"Middleware:          {run_mw}")
    logger.debug(f"Launch Sim:          {launch_sim}")
    logger.debug(f"Headless Mode:       {headless}")

    ## create actions
    actions = []

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
                "use_sim_time": False,  # True to keep tf synched with sim, False rn temporarily before sim
            }
        ],
        output="screen",
    )
    actions.append(robot_state_publisher)

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
    actions.append(temporary_payload_pose)

    include_sim_launch = include_launch(
        "sim",
        "sim2.launch.py",
        launch_arguments={
            "world": world,
            "headless": headless,
        },
    )
    actions.extend([include_sim_launch] if launch_sim else [])

    rviz_config_path = payload_share / "rviz" / "temp_payload.rviz"

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
        arguments=["-d", str(rviz_config_path)],
    )
    actions.extend([rviz] if launch_rviz else [])

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
                choices=["true", "false", "t", "f", "0", "1"],
            ),
            DeclareLaunchArgument(
                Args.WORLD,
                default_value="default",
                description="name of the simulation world that this uav instance belongs to. If standalone=true, then it launches this world using sim package.",
            ),
            DeclareLaunchArgument(
                Args.LAUNCH_SIM,
                default_value="true",
                description="if this or standalone is true, runs sim.launch.py to launch gazebo with the specified world argument",
                choices=["true", "false", "t", "f", "0", "1"],
            ),
            DeclareLaunchArgument(
                Args.HEADLESS,
                default_value="false",
                description="Run Gazebo without its graphical interface.",
                choices=["true", "false", "t", "f", "0", "1"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
