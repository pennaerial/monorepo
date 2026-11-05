from enum import StrEnum
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import Action, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch_ros.actions import Node
from pydantic import ValidationError
from uav.vehicles.AirframeClass import PX4Airframe
from vehicle_common.launch_utils import (
    LaunchError,
    check_unknown_launch_args,
    get_logger,
    is_truthy,
)
from vehicle_common.mission_loader import RuntimeMission, get_mission_path
from vehicle_common.utils import get_available_missions

logger = get_logger("uav_sitl.launch")


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    MISSION = "mission"
    MISSION_PACKAGE = "mission_package"
    NS_ID = "ns_id"
    AIRFRAME = "airframe"
    ENABLE_ULOG = "enable_ulog"


def launch_setup(context) -> list[Action]:
    config = context.launch_configurations  # dict containing declared launch arguments
    check_unknown_launch_args(Args, config, logger)  # warn for unknown args

    mission: str = config[Args.MISSION]  # validate mission
    mission_package: str = config[Args.MISSION_PACKAGE]
    if mission not in get_available_missions(mission_package):
        logger.warning( f"{mission} is not an installed mission. Using filepath as fallback...")  # fmt: skip
        mission_path = Path(mission).expanduser().resolve()
        if not mission_path.is_file() or mission_path.suffix != ".yaml":
            raise LaunchError(f"{mission} is not installed or a valid path to a mission yaml file")
    else:
        mission_path = get_mission_path(mission, mission_package)
    try:
        _ = RuntimeMission.load_from_path(mission_path)  # run this step only for mission validation
    except ValidationError as e:
        logger.info(f"PYDANTIC RUNTIME MISSION VALIDATION ERROR: {e}")
        raise LaunchError(f"Make sure {mission} is a valid mission file")

    try:
        airframe: PX4Airframe = PX4Airframe.lookup_airframe(config[Args.AIRFRAME])
    except KeyError:
        raise LaunchError(f"Airframe: {config[Args.AIRFRAME]} is not valid. Use --show-args to see list of valid airframes")  # fmt: skip

    ns_id = int(config[Args.NS_ID])
    vehicle_ns = f"uav_{ns_id}"

    enable_ulog = is_truthy(config[Args.ENABLE_ULOG])

    # PRINTING HEADER
    # logger.debug(f"ENV VAR DETECTED: PENNAIR_PX4_PATH={PENNAIR_PX4_PATH}")
    logger.debug("LAUNCH PARAMS")
    logger.debug(f"Mission:             {mission}")
    logger.debug(f"Vehicle Namespace:   {vehicle_ns}")
    logger.debug(f"PX4 Airframe:        {airframe}")
    logger.debug(f"Enable ULOG:         {enable_ulog}")
    ## create actions

    mode_manager = Node(
        executable="uav_mission",
        package="uav",
        name="mode_manager",
        namespace=vehicle_ns,
        parameters=[
            {
                "mode_map": str(mission_path),
                "vehicle_name": vehicle_ns,
                "vehicle_class": airframe.airframe_class.name,
                "auto_launch": False,
            }
        ],
        output="screen",
    )

    middleware = ExecuteProcess(
        # -b 921600 = baud rate of UART connection
        # /dev/serial0 is the device name
        cmd=["MicroXRCEAgent", "serial", "--dev", "/dev/serial0", "-b", "921600"],
        output="screen",
        name="MicroXRCEAgent",
    )
    actions = [
        mode_manager,
        middleware,
    ]

    if enable_ulog:
        ulog_script = (
            Path(get_package_share_directory("uav")) / "scripts" / "mavlink_ulog_streaming.py"
        )
        ulog_dir = Path.home() / "px4-logs"
        ulog_dir.mkdir(parents=True, exist_ok=True)
        ulog_logger = ExecuteProcess(
            cmd=["/usr/bin/python3", "-u", str(ulog_script), "/dev/ttyACM0", "-o", str(ulog_dir)],
            output="screen",
            name="ulog_logger",
        )
        actions.append(ulog_logger)
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                Args.MISSION,
                default_value="basic",
                description="Name of the mission to load. Run 'pennair mission <mission_package>' to see the available missions.",
            ),
            DeclareLaunchArgument(
                Args.MISSION_PACKAGE,
                default_value="uav",
                description="Package the mission is installed in, e.g. uav or iarc_flight.",
            ),
            DeclareLaunchArgument(
                Args.NS_ID,
                default_value="0",
                description="Integer namespace id for the vehicle. An id of 0 makes the namespace uav_0. Correponds to the ROS node namespace and PX4 UXRCE namespace",
            ),
            DeclareLaunchArgument(
                Args.AIRFRAME,
                default_value="quadcopter",
                description="UAV airframe to load. Run 'pennair airframe ls' to see all available airframes.",
            ),
            DeclareLaunchArgument(
                Args.ENABLE_ULOG,
                default_value="false",
                description="Enables px4 ulog transfer over MAVLink from flight controller to Pi. Needs a usb(Pi) to usb-c(FC) connection.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
