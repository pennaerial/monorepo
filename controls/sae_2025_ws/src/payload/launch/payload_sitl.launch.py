from enum import StrEnum
from pathlib import Path

from ament_index_python import get_package_share_path
from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pydantic import ValidationError
from vehicle_common.launch_utils import (
    LaunchError,
    check_unknown_launch_args,
    format_bullet_list,
    get_logger,
    include_launch,
    is_truthy,
)
from vehicle_common.runtime.mission_loader import RuntimeMission, get_mission_path
from vehicle_common.utils import get_available_missions

logger = get_logger("payload_sitl.launch")


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    MISSION = "mission"
    NS_ID = "ns_id"
    LAUNCH_RVIZ = "launch_rviz"
    WORLD = "world"
    LAUNCH_SIM = "launch_sim"
    HEADLESS = "headless"


def launch_setup(context) -> list[Action]:
    config = context.launch_configurations

    check_unknown_launch_args(Args, config, logger)

    mission: str = config[Args.MISSION]  # validate mission
    if mission not in get_available_missions("payload"):
        logger.warning( f"{mission} is not an installed mission. Using filepath as fallback...")  # fmt: skip
        mission_path = Path(mission).expanduser().resolve()
        if not mission_path.is_file() or mission_path.suffix != ".yaml":
            raise LaunchError(f"{mission} is not installed or a valid path to a mission yaml file")
    else:
        mission_path = get_mission_path(mission, "payload")
    try:
        _ = RuntimeMission.load_from_path(mission_path)  # run this step only for mission validation
    except ValidationError as e:
        logger.info(f"PYDANTIC RUNTIME MISSION VALIDATION ERROR: {e}")
        raise LaunchError(f"Make sure {mission} is a valid mission file")

    ns_id = int(config[Args.NS_ID])
    vehicle_ns = f"payload_{ns_id}"
    launch_rviz = is_truthy(config[Args.LAUNCH_RVIZ])
    world = config[Args.WORLD]
    launch_sim = is_truthy(config[Args.LAUNCH_SIM])
    headless = config[Args.HEADLESS]

    # PRINTING HEADER
    logger.debug("LAUNCH PARAMS")
    logger.debug(f"Mission:             {mission}")
    logger.debug(f"Vehicle Namespace:   {vehicle_ns}")
    logger.debug(f"Sim World:           {world}")
    # logger.debug(f"Middleware:          {run_mw}")
    logger.debug(f"Launch Sim:          {launch_sim}")
    logger.debug(f"Headless Mode:       {headless}")

    ## create actions
    actions = []

    payload_mode_manager = Node(
        executable="payload_mission",
        package="payload",
        name="payload_mode_manager",
        namespace=vehicle_ns,
        parameters=[
            {"mode_map": str(mission_path), "auto_launch": True, "vehicle_name": vehicle_ns}
        ],
    )
    actions.append(payload_mode_manager)

    # Launch sim
    include_sim_launch = include_launch(
        "sim",
        "sim2.launch.py",
        launch_arguments={
            "world": world,
            "headless": headless,
        },
    )
    actions.extend([include_sim_launch] if launch_sim else [])

    # Initialize payload controller for sim (replace with new payload controller when ready)
    payload_controller = include_launch(
        "payload_controller",
        "payload_controller.launch.py",
        launch_arguments={
            "vehicle_name": vehicle_ns,
            "sim_entity_name": vehicle_ns,
            "controller": "SimController",
        },
    )
    actions.append(payload_controller)

    # Bridge gazebo topics representing payload state to corresponding ros topics for visualization
    # TODO: when we get the new version of payload_controller working, that will handle the tf transfomations
    # instead of gazebo, so we would need to replace this ros_gz_bridge with the middleware bridge
    gz_tf_topic = (
        f"/model/{vehicle_ns}/tf"  # gazebo topic containing movement/transforms of the vehicle
    )
    gz_joint_state_topic = f"/world/{world}/model/{vehicle_ns}/joint_state"  # gazebo topic containing joint states like wheel angles

    payload_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="payload_state_bridge",
        namespace=vehicle_ns,
        arguments=[
            f"{gz_tf_topic}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            f"{gz_joint_state_topic}@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[
            (
                gz_tf_topic,
                "/tf",
            ),  # /tf: ros topic used for defining transforms between world and robot frames
            (
                gz_joint_state_topic,
                f"/{vehicle_ns}/joint_states",
            ),  # /join_states: ros topic used for visualizing joint info like wheel angles
        ],
        output="screen",
    )
    actions.append(payload_state_bridge)

    # Connect the payload's odometry to RViz's world frame:
    # - `world` is the fixed frame used by RViz.
    # - `odom` stays where the payload started measuring its movement.
    # - `base_link` moves with the payload.
    #
    # This publishes the fixed `world -> odom` transform. Gazebo publishes the
    # changing `odom -> base_link` transform, so ROS can determine the payload's
    # pose through the complete chain: `world -> odom -> base_link`.
    world_to_payload_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_payload_odom",
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
            "odom",
        ],
        output="screen",
    )
    actions.append(world_to_payload_odom)

    payload_share = get_package_share_path("payload")
    xacro_path = payload_share / "urdf" / "payload.urdf.xacro"

    # Expand the Xacro into the URDF string used as `robot_description`.
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", str(xacro_path)]),
        value_type=str,
    )
    # Publishes the transforms of internal frames of the robot relative to its base link (ex. base_link -> wheel1)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=vehicle_ns,
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )
    actions.append(robot_state_publisher)

    # Spin up RViz
    rviz_config_path = payload_share / "rviz" / "payload.rviz"
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[
            {
                "use_sim_time": True,
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
                Args.MISSION,
                default_value="basic",
                description=format_bullet_list(
                    "Name of the mission to load.\n\tAvailable missions:",
                    get_available_missions("payload"),
                ),
            ),
            DeclareLaunchArgument(
                Args.NS_ID,
                default_value="0",
                description="Integer namespace id for the vehicle. An id of 0 makes the namespace payload_0. Correponds to the ROS node namespace and payload_controller namespace",
            ),
            DeclareLaunchArgument(
                Args.LAUNCH_RVIZ,
                default_value="true",
                description="If true, launch RViz.",
                choices=["true", "false", "t", "f", "0", "1"],
            ),
            DeclareLaunchArgument(
                Args.WORLD,
                default_value="custom",
                description="name of the simulation world that this payload instance belongs to.",
            ),
            DeclareLaunchArgument(
                Args.LAUNCH_SIM,
                default_value="true",
                description="if this is true, runs sim.launch.py to launch gazebo with the specified world argument",
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
