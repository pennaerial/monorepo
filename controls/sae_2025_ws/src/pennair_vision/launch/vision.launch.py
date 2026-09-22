import ast
from enum import StrEnum

from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from vehicle_common.launch_utils import check_unknown_launch_args, get_logger, is_truthy

logger = get_logger("vision.launch")

# sentinel meaning "let camera_ros detect this from the sensor rather than forcing a value"
AUTO = "auto"


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    NODE_NAME = "node_name"
    DEBUG = "debug"
    CAMERA_TOPIC = "camera_topic"
    PLUGINS = "plugins"
    SIM = "sim"
    CAMERA_FORMAT = "camera_format"
    CAMERA_WIDTH = "camera_width"
    CAMERA_HEIGHT = "camera_height"
    CAMERA_ORIENTATION = "camera_orientation"


def launch_setup(context) -> list[Action]:
    config = context.launch_configurations  # dict containing declared launch arguments
    check_unknown_launch_args(Args, config, logger)  # warn for unknown args

    node_name = config[Args.NODE_NAME]
    debug: bool = is_truthy(config[Args.DEBUG])
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

    # PRINTING HEADER
    logger.debug("LAUNCH PARAMS")
    logger.debug(f"Node Name:           {node_name}")
    logger.debug(f"Camera Topic:        {camera_topic}")
    logger.debug(f"Vision Plugins:      {plugins}")
    logger.debug(f"Debug:               {debug}")
    logger.debug(f"Sim:                 {sim}")
    if not sim:
        logger.debug(f"Camera Format:       {camera_format}")
        logger.debug(f"Camera Resolution:   {camera_width}x{camera_height}")
        logger.debug(f"Camera Orientation:  {camera_orientation}")

    ## create actions
    vision_manager = Node(
        package="pennair_vision",
        executable="vision_manager",
        name=node_name,
        output="screen",
        parameters=[
            {
                "camera_topic": camera_topic,
                "debug": debug,
            },
            {"plugins": plugins} if plugins else {},
        ],
        arguments=["--ros-args", "--log-level", "debug" if debug else "info"],
    )

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
    camera = Node(
        package="camera_ros",
        executable="camera_node",
        # camera_ros publishes <node_name>/image_raw, so naming the node after camera_topic
        # is what makes the vision manager and the driver agree on a topic.
        name=camera_topic,
        output="screen",
        parameters=[camera_params],
    )

    actions = [vision_manager]
    if not sim:
        actions.append(camera)
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
            OpaqueFunction(function=launch_setup),
        ]
    )
