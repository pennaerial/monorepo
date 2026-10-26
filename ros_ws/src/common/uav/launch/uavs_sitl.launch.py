import logging
from enum import StrEnum

from launch import Action, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from vehicle_common.launch_utils import (
    check_unknown_launch_args,
    get_logger,
    include_launch,
    is_truthy,
)


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    NUM_UAVS = "num_uavs"
    MISSION = "mission"
    AIRFRAME = "airframe"
    WORLD = "world"
    SPACING = "spacing"
    HEADLESS = "headless"
    DEBUG = "debug"


def launch_setup(context) -> list[Action]:
    config = context.launch_configurations  # dict containing declared launch arguments
    debug: str = config[Args.DEBUG]
    logger = get_logger("uavs_sitl.launch", logging.DEBUG if is_truthy(debug) else logging.INFO)
    check_unknown_launch_args(Args, config, logger)  # warn for unknown args

    num_uavs = int(config[Args.NUM_UAVS])
    spacing = float(config[Args.SPACING])
    world = config[Args.WORLD]
    headless: str = config[Args.HEADLESS]

    # One mission for every UAV, or a comma-separated list with one mission per UAV
    missions = config[Args.MISSION].split(",")
    if len(missions) == 1:
        missions *= num_uavs
    if len(missions) != num_uavs:
        raise ValueError(f"Got {len(missions)} missions for {num_uavs} UAVs")

    actions: list[Action] = [
        include_launch(
            "sim",
            "sim2.launch.py",
            launch_arguments={"world": world, "headless": headless, "debug": debug},
        ),
        # One agent serves every PX4 instance
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
            output="screen",
            name="MicroXRCEAgent",
        ),
    ]

    for i in range(num_uavs):
        actions.append(
            include_launch(
                "uav",
                "uav_sitl.launch.py",
                launch_arguments={
                    "ns_id": str(i),
                    "mission": missions[i].strip(),
                    "airframe": config[Args.AIRFRAME],
                    "world": world,
                    "model_pose": f"{i * spacing},0",
                    "standalone": "false",
                    "run_mw": "false",
                    "launch_sim": "false",
                    "headless": headless,
                    "debug": debug,
                },
            )
        )

    logger.info(f"Launching {num_uavs} UAVs in world '{world}'")
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                Args.NUM_UAVS,
                default_value="4",
                description="Number of UAVs to launch. They get namespaces uav_0 to uav_<n-1>.",
            ),
            DeclareLaunchArgument(
                Args.MISSION,
                default_value="basic",
                description="Mission for every UAV, or a comma-separated list with one per UAV (uav_0 first). "
                "Each UAV's LOCAL frame starts at its own spawn point.",
            ),
            DeclareLaunchArgument(
                Args.AIRFRAME,
                default_value="quadcopter",
                description="UAV airframe to load. Run 'pennair airframe ls' to see all available airframes.",
            ),
            DeclareLaunchArgument(
                Args.WORLD,
                default_value="default",
                description="Gazebo world shared by all UAVs.",
            ),
            DeclareLaunchArgument(
                Args.SPACING,
                default_value="5.0",
                description="Distance in meters between spawn points, along the Gazebo x axis.",
            ),
            DeclareLaunchArgument(
                Args.HEADLESS,
                default_value="false",
                description="If true, runs gz server in headless mode (no GUI).",
                choices=["true", "false", "t", "f", "0", "1"],
            ),
            DeclareLaunchArgument(
                Args.DEBUG,
                default_value="false",
                description="If true, sets the launch logger's log level to DEBUG.",
                choices=["true", "false", "t", "f", "0", "1"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
