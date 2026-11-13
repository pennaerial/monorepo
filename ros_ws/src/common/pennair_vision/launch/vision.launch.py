import ast
from datetime import datetime
from enum import StrEnum
from pathlib import Path

from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from vehicle_common.launch_utils import check_unknown_launch_args, get_logger, is_truthy

logger = get_logger("vision.launch")

# sentinel meaning "let camera_ros detect this from the sensor rather than forcing a value"
AUTO = "auto"


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    NODE_NAME = "node_name"
    DEBUG_LOGS = "debug_logs"
    DEBUG_IMAGE_RATE = "debug_image_rate"
    CAMERA_TOPIC = "camera_topic"
    PLUGINS = "plugins"
    SIM = "sim"
    CAMERA_FORMAT = "camera_format"
    CAMERA_WIDTH = "camera_width"
    CAMERA_HEIGHT = "camera_height"
    CAMERA_ORIENTATION = "camera_orientation"
    RECORD_TOPICS = "record_topics"


def launch_setup(context) -> list[Action]:
    config = context.launch_configurations  # dict containing declared launch arguments
    check_unknown_launch_args(Args, config, logger)  # warn for unknown args

    node_name = config[Args.NODE_NAME]
    debug_logs: bool = is_truthy(config[Args.DEBUG_LOGS])
    debug_image_rate = float(config[Args.DEBUG_IMAGE_RATE])
    camera_topic = config[Args.CAMERA_TOPIC]
    sim: bool = is_truthy(config[Args.SIM])
    # "auto" means leave the setting to camera_ros, which picks whatever the sensor reports.
    # Anything we do not override is simply left out of the parameter dict below.
    camera_format = config[Args.CAMERA_FORMAT]
    camera_width = config[Args.CAMERA_WIDTH]
    camera_height = config[Args.CAMERA_HEIGHT]
    camera_orientation = int(config[Args.CAMERA_ORIENTATION])

    # launch arguments always arrive as strings, so parse the list literal into a real list[str].
    # Otherwise the parameter would be typed as a string rather than a string array.
    plugins_arg = config[Args.PLUGINS].strip()
    plugins: list[str] = ast.literal_eval(plugins_arg) if plugins_arg else []
    record_topics_arg = config[Args.RECORD_TOPICS].strip()
    record_topics = ast.literal_eval(record_topics_arg) if record_topics_arg else []

    # PRINTING HEADER
    logger.debug("LAUNCH PARAMS")
    logger.debug(f"Node Name:           {node_name}")
    logger.debug(f"Camera Topic:        {camera_topic}")
    logger.debug(f"Vision Plugins:      {plugins}")
    logger.debug(f"Debug:               {debug_logs}")
    logger.debug(f"Image Debug Rate (Hz):     {debug_image_rate}")
    logger.debug(f"Sim:                 {sim}")
    if not sim:
        logger.debug(f"Camera Format:       {camera_format}")
        logger.debug(f"Camera Resolution:   {camera_width}x{camera_height}")
        logger.debug(f"Camera Orientation:  {camera_orientation}")

    ## create actions
    # camera_ros autodetects anything left unset, so only send the settings that were overridden.
    camera_params: dict = {
        "orientation": camera_orientation,
    }
    if camera_format != AUTO:
        camera_params["format"] = camera_format
    if camera_width != AUTO:
        camera_params["width"] = int(camera_width)
    if camera_height != AUTO:
        camera_params["height"] = int(camera_height)

    # On real hardware nothing publishes camera frames, so run the libcamera driver.
    composable_nodes = [
        ComposableNode(
            package="pennair_vision",
            plugin="pennair_vision::VisionManager",
            name=node_name,
            parameters=[
                {
                    "camera_topic": camera_topic,
                    "debug_logs": debug_logs,
                    "debug_image_rate": debug_image_rate,
                },
                {"plugins": plugins} if plugins else {},
            ],
            extra_arguments=[
                {"use_intra_process_comms": True},
            ],
        ),
    ]
    if not sim:
        # CameraSubscriber expects camera_info alongside the image topic.
        camera_info_topic = (
            camera_topic.rpartition("/")[0] + "/camera_info"
            if "/" in camera_topic
            else "camera_info"
        )
        composable_nodes.append(
            ComposableNode(
                package="camera_ros",
                plugin="camera::CameraNode",
                name="camera",
                parameters=[camera_params],
                remappings=[
                    ("~/image_raw", camera_topic),
                    ("~/image_raw/compressed", camera_topic + "/compressed"),
                    ("~/camera_info", camera_info_topic),
                ],
            )
        )

    # type hint stops type-checking error about actions containing both ComposableNodeContainer and Node
    actions: list[Action] = [
        ComposableNodeContainer(
            name="vision_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            arguments=[
                "--ros-args",
                "--log-level",
                f"{node_name}:={'debug' if debug_logs else 'info'}",
            ],
            composable_node_descriptions=composable_nodes,
            output="screen",
        ),
    ]

    if record_topics:
        bag_output = str(Path.home() / "bags" / f"vision_{datetime.now():%Y%m%d_%H%M%S_%f}")

        rosbagger = Node(
            package="rosbag2_transport",
            executable="recorder",
            name="vision_recorder",
            output="screen",
            parameters=[
                {
                    "record.topics": record_topics,
                    "record.disable_keyboard_controls": True,
                    "storage.uri": bag_output,
                    "storage.storage_id": "mcap",
                }
            ],
        )
        actions.append(rosbagger)

    return actions


# camera_topic expects frames, will error out otherwise
def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                Args.NODE_NAME,
                default_value="vision_manager",
                description="Node name for the vision manager. Override to run multiple instances in one namespace.",
            ),
            DeclareLaunchArgument(
                Args.DEBUG_LOGS,
                default_value="false",
                description="if true, runs the vision manager at debug log level. otherwise info.",
                choices=["true", "false", "t", "f", "0", "1"],
            ),
            DeclareLaunchArgument(
                Args.DEBUG_IMAGE_RATE,
                default_value="5.0",
                description="Maximum debug image rate per plugin in Hz; 0 means unlimited. Also limits processing when only debug has subscribers. Algorithm output subscribers always request full-speed processing. Independent of the debug log-level flag.",
            ),
            DeclareLaunchArgument(
                Args.CAMERA_TOPIC,
                default_value="camera/image_raw",
                description="Image topic used by the vision manager and hardware camera driver. Relative names use the node namespace; names starting with / are absolute.",
            ),
            DeclareLaunchArgument(
                Args.PLUGINS,
                default_value="[]",
                description="list of VisionPlugin names to load, as a python list literal, e.g. \"['AprilTagPlugin']\"",
            ),
            DeclareLaunchArgument(
                Args.SIM,
                default_value="false",
                description="if true, camera frames are expected from the Gazebo bridge. if false, launches the camera_ros driver for a real camera.",
                choices=["true", "false", "t", "f", "0", "1"],
            ),
            DeclareLaunchArgument(
                Args.CAMERA_FORMAT,
                default_value=AUTO,
                description=f'pixel format for the camera_ros driver, e.g. YUYV, RGB888, BGR888. "{AUTO}" lets the driver pick the sensor default. ignored when sim is true.',
            ),
            DeclareLaunchArgument(
                Args.CAMERA_WIDTH,
                default_value=AUTO,
                description=f'camera capture width in pixels. must be a mode the sensor supports. "{AUTO}" lets the driver pick. ignored when sim is true.',
            ),
            DeclareLaunchArgument(
                Args.CAMERA_HEIGHT,
                default_value=AUTO,
                description=f'camera capture height in pixels. must be a mode the sensor supports. "{AUTO}" lets the driver pick. ignored when sim is true.',
            ),
            DeclareLaunchArgument(
                Args.CAMERA_ORIENTATION,
                default_value="0",
                description="clockwise rotation applied to the camera image, in degrees. set this to match how the camera is physically mounted. ignored when sim is true.",
                choices=["0", "90", "180", "270"],
            ),
            DeclareLaunchArgument(
                Args.RECORD_TOPICS,
                default_value="[]",
                description="List of topics to record in a rosbag",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
