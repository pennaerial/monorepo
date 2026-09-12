from enum import StrEnum

from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from vehicle_common.launch_utils import (
    get_logger,
    check_unknown_launch_args,
    is_truthy,
)


logger = get_logger("vision.launch")

# Both camera sources are remapped onto these names so that whatever consumes the frames cannot
# tell sim from real. Neither source publishes here by default: v4l2_camera uses image_raw, and the
# gz bridge emits a long /world/... path.
CAMERA_TOPIC = "camera"
CAMERA_INFO_TOPIC = "camera_info"


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    VEHICLE_NAME = "vehicle_name"
    NODE_NAME = "node_name"
    LOG_LEVEL = "log_level"
    SIM = "sim"
    WORLD = "world"
    SIM_ENTITY_NAME = "sim_entity_name"
    VIDEO_DEVICE = "video_device"
    IMAGE_WIDTH = "image_width"
    IMAGE_HEIGHT = "image_height"


def sim_camera_actions(world: str, vehicle_name: str, sim_entity_name: str) -> list[Action]:
    """Bridge the gazebo camera sensor into ROS. Mirrors uav/launch/vehicle_stack.launch.py"""
    base = f"/world/{world}/model/{sim_entity_name}/link/camera_link/sensor/camera"
    gz_image_topic = f"{base}/image"
    gz_camera_info_topic = f"{base}/camera_info"

    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[f"{gz_image_topic}@sensor_msgs/msg/Image[gz.msgs.Image"],
            remappings=[(gz_image_topic, CAMERA_TOPIC)],
            output="screen",
            name=f"{vehicle_name}_camera_bridge",
            namespace=vehicle_name,
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[f"{gz_camera_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
            remappings=[(gz_camera_info_topic, CAMERA_INFO_TOPIC)],
            output="screen",
            name=f"{vehicle_name}_camera_info_bridge",
            namespace=vehicle_name,
        ),
    ]


def real_camera_actions(
    vehicle_name: str, video_device: str, image_size: list[int]
) -> list[Action]:
    """Drive a real camera off /dev/video*. Mirrors uav/launch/vehicle_stack.launch.py"""
    return [
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name=f"{vehicle_name}_camera",
            namespace=vehicle_name,
            output="screen",
            parameters=[
                {
                    "video_device": video_device,
                    "image_size": image_size,
                    "output_encoding": "bgr8",
                }
            ],
            remappings=[
                ("image_raw", CAMERA_TOPIC),
                ("camera_info", CAMERA_INFO_TOPIC),
            ],
        )
    ]


def launch_setup(context) -> list[Action]:
    config = context.launch_configurations  # dict containing declared launch arguments
    check_unknown_launch_args(Args, config, logger)  # warn for unknown args

    vehicle_name = config[Args.VEHICLE_NAME]
    node_name = config[Args.NODE_NAME]
    log_level = config[Args.LOG_LEVEL]
    sim: bool = is_truthy(config[Args.SIM])
    world = config[Args.WORLD]
    sim_entity_name = config[Args.SIM_ENTITY_NAME].strip() or vehicle_name
    video_device = config[Args.VIDEO_DEVICE]
    image_size = [int(config[Args.IMAGE_WIDTH]), int(config[Args.IMAGE_HEIGHT])]

    # PRINTING HEADER
    logger.debug("LAUNCH PARAMS")
    logger.debug(f"Vehicle Namespace:   {vehicle_name}")
    logger.debug(f"Node Name:           {node_name}")
    logger.debug(f"Sim Mode:            {sim}")
    logger.debug(f"Sim World:           {world}")
    logger.debug(f"Sim Entity:          {sim_entity_name}")
    logger.debug(f"Video Device:        {video_device}")
    logger.debug(f"Image Size:          {image_size}")
    logger.debug(f"Log Level:           {log_level}")

    ## create actions
    vision_manager = Node(
        package="pennair_vision",
        executable="vision_manager",
        name=node_name,
        namespace=vehicle_name,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    actions = [vision_manager]
    actions.extend(
        sim_camera_actions(world, vehicle_name, sim_entity_name)
        if sim
        else real_camera_actions(vehicle_name, video_device, image_size)
    )
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
                Args.SIM,
                default_value="true",
                description="if true, bridges camera frames from gazebo. if false, runs the v4l2 camera driver against a real device",
                choices=["true", "false", "t", "f", "0", "1"],
            ),
            DeclareLaunchArgument(
                Args.WORLD,
                default_value="default",
                description="name of the simulation world the camera sensor lives in. Only applies when sim=true.",
            ),
            DeclareLaunchArgument(
                Args.SIM_ENTITY_NAME,
                default_value="",
                description="gz model name carrying the camera sensor. Defaults to vehicle_name when blank. Only applies when sim=true.",
            ),
            DeclareLaunchArgument(
                Args.VIDEO_DEVICE,
                default_value="/dev/video0",
                description="v4l2 device to capture from. Only applies when sim=false.",
            ),
            DeclareLaunchArgument(
                Args.IMAGE_WIDTH,
                default_value="640",
                description="capture width in pixels. Only applies when sim=false.",
            ),
            DeclareLaunchArgument(
                Args.IMAGE_HEIGHT,
                default_value="480",
                description="capture height in pixels. Only applies when sim=false.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
