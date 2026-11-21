import shutil
from enum import StrEnum
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import Action, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
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
from vehicle_common.utils import get_available_missions, pi_id

logger = get_logger("uav_sitl.launch")


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    MISSION = "mission"
    MISSION_PACKAGE = "mission_package"
    NS_ID = "ns_id"
    AIRFRAME = "airframe"
    ENABLE_ULOG = "enable_ulog"
    PEERS = "peers"


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
    pi = pi_id()  # 6 on air-06, None off a Pi

    enable_ulog = is_truthy(config[Args.ENABLE_ULOG])

    # PRINTING HEADER
    # logger.debug(f"ENV VAR DETECTED: PENNAIR_PX4_PATH={PENNAIR_PX4_PATH}")
    logger.debug("LAUNCH PARAMS")
    logger.debug(f"Mission:             {mission}")
    logger.debug(f"Vehicle Namespace:   {vehicle_ns}")
    logger.debug(f"ROS Domain:          {pi if pi is not None else 'unchanged'}")
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
        # puts PX4's topics in this drone's domain, whatever UXRCE_DDS_DOM_ID is set to
        additional_env={"XRCE_DOMAIN_ID_OVERRIDE": str(pi)} if pi is not None else {},
        output="screen",
        name="MicroXRCEAgent",
    )
    actions = []
    if pi is not None:
        # The Pi's hostname gives the ROS domain: 6 on air-06.
        # Each drone gets its own domain, so DDS never crosses the mesh.
        # The bridge passes only the topics zenoh_bridge.json5 allows.
        actions.append(SetEnvironmentVariable("ROS_DOMAIN_ID", str(pi)))
        if shutil.which("zenoh-bridge-ros2dds"):
            bridge_config = (
                Path(get_package_share_directory("uav")) / "config" / "zenoh_bridge.json5"
            )
            cmd = ["zenoh-bridge-ros2dds", "-c", str(bridge_config), "-d", str(pi)]
            cmd += ["-l", "tcp/0.0.0.0:7447"]
            for peer in config[Args.PEERS].split(","):
                if peer.strip():
                    cmd += ["-e", f"tcp/10.10.0.{int(peer)}:7447"]
            actions.append(ExecuteProcess(cmd=cmd, output="screen", name="zenoh_bridge"))
        else:
            logger.warning("zenoh-bridge-ros2dds is not installed. Other drones won't see this one.")  # fmt: skip
    actions += [
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
                description="Enables px4 ulog transfer over MAVLink from flight controller to Pi. Needs a usb(Pi) to usb-c(FC) connection.",
            ),
            DeclareLaunchArgument(
                Args.PEERS,
                default_value="",
                description="Other drones' numbers for the Zenoh bridge to connect to, e.g. 4,6. Only needed if the drones don't find each other on the mesh.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
