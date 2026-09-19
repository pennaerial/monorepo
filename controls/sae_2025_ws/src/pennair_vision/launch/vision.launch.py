import ast
from enum import StrEnum

from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from vehicle_common.launch_utils import check_unknown_launch_args, get_logger, is_truthy

logger = get_logger("vision.launch")


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    NODE_NAME = "node_name"
    DEBUG = "debug"
    CAMERA_TOPIC = "camera_topic"
    PLUGINS = "plugins"


def launch_setup(context) -> list[Action]:
    config = context.launch_configurations  # dict containing declared launch arguments
    check_unknown_launch_args(Args, config, logger)  # warn for unknown args

    node_name = config[Args.NODE_NAME]
    debug: bool = is_truthy(config[Args.DEBUG])
    camera_topic = config[Args.CAMERA_TOPIC]

    # launch arguments always arrive as strings, so parse the list literal into a real list[str].
    # Otherwise the parameter would be typed as a string rather than a string array.
    plugins_arg = config[Args.PLUGINS].strip()
    plugins: list[str] = ast.literal_eval(plugins_arg) if plugins_arg else []

    # PRINTING HEADER
    logger.debug("LAUNCH PARAMS")
    logger.debug(f"Node Name:           {node_name}")
    logger.debug(f"Camera Topic:        {camera_topic}")
    logger.debug(f"Vision Plugins:      {plugins}")
    logger.debug(f"Debug:               {debug}")

    ## create actions
    vision_manager = Node(
        package="pennair_vision",
        executable="vision_manager",
        name=node_name,
        output="screen",
        parameters=[
            {
                "camera_topic": camera_topic,
                "plugins": plugins,
                "debug": debug,
            }
        ],
        arguments=["--ros-args", "--log-level", "debug" if debug else "info"],
    )

    actions = [vision_manager]
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                Args.NODE_NAME,
                default_value="vision_manager",
                description="Node name for the vision manager. Override to run multiple instances in one namespace.",
            ),
            DeclareLaunchArgument(
                Args.DEBUG,
                default_value="false",
                description="if true, runs the vision manager at debug log level. otherwise info.",
                choices=["true", "false", "t", "f", "0", "1"],
            ),
            DeclareLaunchArgument(
                Args.CAMERA_TOPIC,
                default_value="camera",
                description="topic the vision manager reads camera frames from, relative to the namespace it is launched into",
            ),
            DeclareLaunchArgument(
                Args.PLUGINS,
                default_value="[]",
                description="list of VisionPlugin names to load, as a python list literal, e.g. \"['AprilTagPlugin']\"",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
