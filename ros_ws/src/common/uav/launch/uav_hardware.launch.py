import signal
from enum import StrEnum
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import Action, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.events import matches_action
from launch.events.process import SignalProcess
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
    ENABLE_ULOG = "ulog"
    MAVPROXY = "mavproxy"


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
    enable_mavproxy = is_truthy(config[Args.MAVPROXY])

    # PRINTING HEADER
    # logger.debug(f"ENV VAR DETECTED: PENNAIR_PX4_PATH={PENNAIR_PX4_PATH}")
    logger.debug("LAUNCH PARAMS")
    logger.debug(f"Mission:             {mission}")
    logger.debug(f"Vehicle Namespace:   {vehicle_ns}")
    logger.debug(f"PX4 Airframe:        {airframe}")
    logger.debug(f"Enable ULOG:         {enable_ulog}")
    logger.debug(f"Forward to QGC:      {enable_mavproxy}")
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
        # /dev/ttyAMA0 is the Pi UART used for the XRCE-DDS connection.
        cmd=["MicroXRCEAgent", "serial", "--dev", "/dev/ttyAMA0", "-b", "921600"],
        output="screen",
        name="MicroXRCEAgent",
    )
    actions = [
        mode_manager,
        middleware,
    ]

    # /dev/ttyAMA5 is the Pi UART used for the MAVLink connection.
    # one process owns UART5 and the flags select its UDP destinations
    if enable_mavproxy or enable_ulog:
        mavproxy_cmd = [
            "mavproxy.py",
            "--master=/dev/ttyAMA5",
            "--baudrate=921600",
            "--non-interactive",
            "--default-modules=output",
            # Disable mavproxy sending gcs heartbeat by itself
            "--heartbeat-rate=0",
            "--nowait",
        ]
        if enable_mavproxy:
            mavproxy_cmd.append("--out=udp:10.10.0.254:14550")
        if enable_ulog:
            mavproxy_cmd.append("--out=udp:127.0.0.1:14551")

        mavproxy = ExecuteProcess(
            cmd=mavproxy_cmd,
            # Keep UART5 available for LOGGING_STOP while the logger handles Ctrl+C.
            prefix="/usr/bin/env --block-signal=INT" if enable_ulog else None,
            output="screen",
            name="mavproxy",
        )
        actions.append(mavproxy)

        if enable_ulog:

            def stop_proxy_after_logger(_event, context):
                if context.is_shutdown or not enable_mavproxy:
                    context.emit_event_sync(
                        SignalProcess(
                            signal_number=signal.SIGTERM, process_matcher=matches_action(mavproxy)
                        )
                    )

            ulog_script = (
                Path(get_package_share_directory("uav")) / "scripts" / "mavlink_ulog_streaming.py"
            )
            ulog_dir = Path.home() / "px4-logs"
            ulog_dir.mkdir(parents=True, exist_ok=True)
            ulog_logger = ExecuteProcess(
                cmd=[
                    "/usr/bin/python3",
                    "-u",
                    str(ulog_script),
                    "udpin:127.0.0.1:14551",
                    "-o",
                    str(ulog_dir),
                ],
                output="screen",
                name="ulog_logger",
                on_exit=stop_proxy_after_logger,
            )
            actions.append(ulog_logger)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                Args.MISSION,
                default_value="basic",
                description="Name of the mission to load. Run 'ros2 run pennair_cli pennair mission <mission_package>' to see the available missions.",
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
                description="UAV airframe to load. Run 'ros2 run pennair_cli pennair airframe ls' to see all available airframes.",
            ),
            DeclareLaunchArgument(
                Args.ENABLE_ULOG,
                default_value="false",
                description="Record PX4 ULog over UART5; starts MAVProxy.",
            ),
            DeclareLaunchArgument(
                Args.MAVPROXY,
                default_value="false",
                description="Forward UART5 MAVLink to QGC at 10.10.0.254:14550.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
