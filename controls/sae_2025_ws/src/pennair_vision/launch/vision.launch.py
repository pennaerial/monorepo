import ast
from enum import StrEnum

from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from vehicle_common.launch_utils import get_logger, check_unknown_launch_args


logger = get_logger("vision.launch")


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    VEHICLE_NAME = "vehicle_name"
    NODE_NAME = "node_name"
    LOG_LEVEL = "log_level"
    CAMERA_TOPIC = "camera_topic"
    PLUGINS = "plugins"


def launch_setup(context) -> list[Action]:
    config = context.launch_configurations  # dict containing declared launch arguments
    check_unknown_launch_args(Args, config, logger)  # warn for unknown args

    vehicle_name = config[Args.VEHICLE_NAME]
    node_name = config[Args.NODE_NAME]
    log_level = config[Args.LOG_LEVEL]
    camera_topic = config[Args.CAMERA_TOPIC]

    # launch arguments always arrive as strings, so parse the list literal into a real list[str].
    # Otherwise the parameter would be typed as a string rather than a string array.
    plugins_arg = config[Args.PLUGINS].strip()
    plugins: list[str] = ast.literal_eval(plugins_arg) if plugins_arg else []

    # PRINTING HEADER
    logger.debug("LAUNCH PARAMS")
    logger.debug(f"Vehicle Namespace:   {vehicle_name}")
    logger.debug(f"Node Name:           {node_name}")
    logger.debug(f"Camera Topic:        {camera_topic}")
    logger.debug(f"Vision Plugins:      {plugins}")
    logger.debug(f"Log Level:           {log_level}")

    ## create actions
    vision_manager = Node(
        package="pennair_vision",
        executable="vision_manager",
        name=node_name,
        namespace=vehicle_name,
        output="screen",
        parameters=[
            {
                "camera_topic": camera_topic,
                "plugins": plugins,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    actions = [vision_manager]
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                Args.VEHICLE_NAME,
                default_value="uav_0",
                description="ROS namespace for the vision stack. Corresponds to the vehicle namespace used by the rest of the stack.",
            ),
            DeclareLaunchArgument(
                Args.NODE_NAME,
                default_value="vision_manager",
                description="Node name for the vision manager. Override to run multiple instances in one namespace.",
            ),
            DeclareLaunchArgument(
                Args.LOG_LEVEL,
                default_value="info",
                description="rclcpp log level for the vision manager process",
                choices=["debug", "info", "warn", "error", "fatal"],
            ),
            DeclareLaunchArgument(
                Args.CAMERA_TOPIC,
                default_value="camera",
                description="topic the vision manager reads camera frames from, relative to the vehicle namespace",
            ),
            DeclareLaunchArgument(
                Args.PLUGINS,
                default_value="[]",
                description="list of VisionPlugin names to load, as a python list literal, e.g. \"['AprilTagPlugin']\"",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
