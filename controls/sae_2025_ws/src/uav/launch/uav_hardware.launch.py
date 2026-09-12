from enum import StrEnum
from pathlib import Path

from pydantic import ValidationError
from launch import Action, LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
)

from uav.vehicles.AirframeClass import PX4Airframe
from vehicle_common.utils import get_available_missions
from vehicle_common.runtime.mission_loader import RuntimeMission, get_mission_path
from vehicle_common.launch_utils import (
    get_logger,
    LaunchError,
    check_unknown_launch_args,
    format_bullet_list,
)


logger = get_logger("uav_sitl.launch")


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    MISSION = "mission"
    NS_ID = "ns_id"
    AIRFRAME = "airframe"


def launch_setup(context) -> list[Action]:
    config = context.launch_configurations  # dict containing declared launch arguments
    check_unknown_launch_args(Args, config, logger)  # warn for unknown args

    mission: str = config[Args.MISSION]  # validate mission
    if mission not in get_available_missions("uav"):
        logger.warning( f"{mission} is not an installed mission. Using filepath as fallback...")  # fmt: skip
        mission_path = Path(mission).expanduser().resolve()
        if not mission_path.is_file() or mission_path.suffix != ".yaml":
            raise LaunchError(f"{mission} is not installed or a valid path to a mission yaml file")
    else:
        mission_path = get_mission_path(mission, "uav")
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

    # PRINTING HEADER
    # logger.debug(f"ENV VAR DETECTED: PENNAIR_PX4_PATH={PENNAIR_PX4_PATH}")
    logger.debug("LAUNCH PARAMS")
    logger.debug(f"Mission:             {mission}")
    logger.debug(f"Vehicle Namespace:   {vehicle_ns}")
    logger.debug(f"PX4 Airframe:        {airframe}")
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
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                Args.MISSION,
                default_value="basic",
                description=format_bullet_list(
                    "Name of the mission to load.\n\tAvailable missions:",
                    get_available_missions("uav"),
                ),
            ),
            DeclareLaunchArgument(
                Args.NS_ID,
                default_value="0",
                description="Integer namespace id for the vehicle. An id of 0 makes the namespace uav_0. Correponds to the ROS node namespace and PX4 SITL namespace",
            ),
            DeclareLaunchArgument(
                Args.AIRFRAME,
                default_value="quadcopter",
                description=format_bullet_list(
                    "UAV airframe to load.\n\tAvailable airframes: (alias/id/model)",
                    [str(a) for a in PX4Airframe.get_flying()],
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
