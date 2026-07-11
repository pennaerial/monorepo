from launch import Action, LaunchDescription, EventHandler, Event
from launch.actions import (
    DeclareLaunchArgument,
    # IncludeLaunchDescription,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from rclpy.impl.rcutils_logger import RcutilsLogger
from vehicle_common.px4 import get_px4_submodule_path
from rclpy.logging import get_logger
from uav.vehicles.AirframeClass import PX4Airframe
from enum import StrEnum
# from launch.logging import get_logger

logger = get_logger("uav_sitl.launch")


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    PX4_PATH = "px4_path"
    MISSION = "mission"
    NS_ID = "ns_id"
    AIRFRAME = "airframe"
    WORLD = "world"
    RUN_MW = "run_mw"


# TOOD: Move to a launch_utils file
def on_emitted_event(actions: list[Action], event: type[Event]) -> RegisterEventHandler:
    """Wraps a sequences of actions around an EventHandler so that the actions will
    fire after the specified Event type has been emitted

    Args:
        actions: list of actions
        event: Event type to listen for

    Returns:
        The RegisterEventHandler listener object
    """
    return RegisterEventHandler(
        EventHandler(matcher=lambda e: isinstance(e, event), entities=actions)
    )


# TODO: Move to launch_utils
RED = "\033[31m"
RESET = "\033[0m"


class LaunchError(RuntimeError):
    def __init__(self, msg: str):
        super().__init__(f"{RED}{msg}{RESET}")


# TODO: Move to a launch_utils file
def reject_unknown_launch_args(
    arg_enum: type[StrEnum], launch_configurations: dict, logger: RcutilsLogger
):
    """Raises an error if any undeclared launch arguments are present.

    Args:
        arg_enum: StrEnum containing the set of valid launch argument names.
        launch_configurations: Launch configuration mapping from the launch context.

    Raises:
        RuntimeError: If one or more unknown launch arguments are found.
    """
    valid_args = {arg.value for arg in arg_enum}
    unknown = set(launch_configurations) - valid_args
    if unknown:
        raise LaunchError(
            f"Unknown launch argument(s): {', '.join(sorted(unknown))}\n"
            "Run `ros2 launch <package> <launch_file> --show-args` to see valid arguments."
        )


def px4_sitl_action(
    px4_path: str,
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
        cmd="./build/px4_sitl_default/bin/px4",
        cwd=px4_path,
        output="screen",
        name=f"{vehicle_ns}_px4_sitl",
        additional_env=env_export,
        log_cmd=True,
    )

    return Action()


def launch_setup(context):
    config = context.launch_configurations  # dict containing declared launch arguments
    reject_unknown_launch_args(Args, config, logger)
    ns_id = config["ns_id"]
    vehicle_ns = f"uav_{ns_id}"

    try:
        airframe: PX4Airframe = PX4Airframe.lookup_airframe(config["airframe"])
    except KeyError:
        raise LaunchError(
            f"Airframe: {config['airframe']} is not valid. Use ros2 launch uav uav_sitl.launch.py --show-args to see list of valid airframes"
        )

    px4_path = config["px4_path"]
    print(vehicle_ns)
    print(px4_path)
    print(airframe)

    middleware = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        name="MicroXRCEAgent",
        condition=IfCondition(config["run_mw"]),
        log_cmd=True,
    )

    px4_sitl = on_emitted_event(actions=[], event=Event)

    actions = [middleware, px4_sitl]
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                Args.PX4_PATH,
                default_value=get_px4_submodule_path(),
                description="Path to PX4-Autopilot installation. Defaults to PX4-Autopilot submodule path",
            ),
            DeclareLaunchArgument(
                Args.MISSION,
                default_value="basic",
                description="Name of the mission to load. Available missions are found in uav/missions/",  # TODO: print out available
                choices=["basic", "triangle"],
            ),
            DeclareLaunchArgument(
                Args.NS_ID,
                default_value="0",
                description="Integer namespace id for the vehicle. An id of 0 makes the namespace uav_0. Correponds to the ROS node namespace and PX4 SITL namespace",
            ),
            DeclareLaunchArgument(
                Args.AIRFRAME,
                default_value="quadcopter",
                description=(
                    "UAV airframe to load.\n"
                    "\tAvailable airframes: (alias/id/model)\n"
                    + "\n".join(f"\t  • {a}" for a in PX4Airframe.get_flying())
                ),
            ),
            DeclareLaunchArgument(
                Args.WORLD,
                default_value="default",
                description="name of the simulation world to load",
            ),
            DeclareLaunchArgument(
                Args.RUN_MW,
                default_value="true",
                description="if true, runs the MicroXRCEAgent middleware to bridge ROS to PX4 SITL",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
