import os
from enum import StrEnum

from launch import Action, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
)

from uav.vehicles.AirframeClass import PX4Airframe
from uav.utils import get_available_missions
from vehicle_common.runtime.mission_loader import RuntimeMission
from vehicle_common.launch_utils import (
    get_logger,
    LaunchError,
    check_unknown_launch_args,
    include_launch,
    format_bullet_list,
)


logger = get_logger("uav_sitl.launch")

### env vars
PENNAIR_PX4_PATH = os.environ.get("PENNAIR_PX4_PATH", "")
if not PENNAIR_PX4_PATH:
    raise LaunchError("PENNAIR_PX4_PATH is not set. Please source dev_env.sh or manually set it")


def as_bool(value: str) -> bool:
    return value.lower() == "true"


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    MISSION = "mission"
    NS_ID = "ns_id"
    AIRFRAME = "airframe"
    WORLD = "world"
    RUN_MW = "run_mw"
    LAUNCH_SIM = "launch_sim"
    STANDALONE = "standalone"


def px4_sitl_action(
    autostart_id: int,
    vehicle_ns: str,  # e.g. uav_0
    world: str,
) -> Action:
    env_export = {
        "PX4_GZ_MODEL_NAME": vehicle_ns,
        "PX4_GZ_WORLD": world,
        "PX4_GZ_STANDALONE": "1",
        "PX4_SYS_AUTOSTART": str(autostart_id),
        "PX4_UXRCE_DDS_NS": vehicle_ns,
    }

    return ExecuteProcess(
        cmd=["./build/px4_sitl_default/bin/px4"],
        cwd=PENNAIR_PX4_PATH,
        output="screen",
        name=f"{vehicle_ns}_px4_sitl",
        additional_env=env_export,  # type: ignore (dict[str, str] works instead of SomeSubstitutionsType)
        log_cmd=True,
    )


def launch_setup(context) -> list[Action]:
    config = context.launch_configurations  # dict containing declared launch arguments
    check_unknown_launch_args(Args, config, logger)  # warn for unknown args

    mission: str = config[Args.MISSION]  # validate mission
    if mission not in get_available_missions():
        raise LaunchError(
            f"Mission: {mission} is not valid. Use --show-args to see list of valid missions"
        )

    ns_id = int(config[Args.NS_ID])
    vehicle_ns = f"uav_{ns_id}"

    try:
        airframe: PX4Airframe = PX4Airframe.lookup_airframe(config[Args.AIRFRAME])
    except KeyError:
        raise LaunchError(
            f"Airframe: {config[Args.AIRFRAME]} is not valid. Use --show-args to see list of valid airframes"
        )

    sim_world = config[Args.WORLD]
    standalone: bool = as_bool(config[Args.STANDALONE])
    run_mw: bool = standalone or as_bool(config[Args.RUN_MW])
    launch_sim: bool = standalone or as_bool(config[Args.LAUNCH_SIM])

    # PRINTING HEADER
    logger.debug(f"ENV VAR DETECTED: PENNAIR_PX4_PATH={PENNAIR_PX4_PATH}")
    logger.debug("LAUNCH PARAMS")
    logger.debug(f"Mission:             {mission}")
    logger.debug(f"Vehicle Namespace:   {vehicle_ns}")
    logger.debug(f"PX4 Airframe:        {airframe}")
    logger.debug(f"Sim World:           {sim_world}")
    logger.debug(f"Standalone Mode:     {standalone}")
    logger.debug(f"Middleware:          {run_mw}")
    logger.debug(f"Launch Sim:          {launch_sim}")

    actions = []

    middleware = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        name="MicroXRCEAgent",
    )
    if run_mw:
        actions.append(middleware)

    px4_sitl = px4_sitl_action(airframe.id, vehicle_ns, sim_world)
    actions.append(px4_sitl)

    include_sim_launch = include_launch("sim", "sim.launch.py")
    if launch_sim:
        actions.append(include_sim_launch)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                Args.MISSION,
                default_value="basic",
                description=format_bullet_list(
                    "Name of the mission to load.\n\tAvailable missions:",
                    get_available_missions(),
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
                    PX4Airframe.get_flying(),
                ),
            ),
            DeclareLaunchArgument(
                Args.WORLD,
                default_value="default",
                description="name of the simulation world that this uav instance belongs to. If standalone=true, then it launches this world using sim package.",
            ),
            DeclareLaunchArgument(
                Args.RUN_MW,
                default_value="false",
                description="if this or standalone is true, runs the MicroXRCEAgent middleware to bridge ROS to PX4 SITL",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                Args.LAUNCH_SIM,
                default_value="false",
                description="if this or standalone is true, runs sim.launch.py to launch gazebo with the specified world argument",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                Args.STANDALONE,
                default_value="true",
                description="if true, runs all necessary standalone components, like middleware, sim, etc",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
