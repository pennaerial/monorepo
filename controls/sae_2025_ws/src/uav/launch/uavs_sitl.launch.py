import logging
from enum import StrEnum

from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from vehicle_common.launch_utils import get_logger, include_launch, is_truthy

logger = get_logger("uavs_sitl.launch")


class Args(StrEnum):
    WORLD = "world"
    HEADLESS = "headless"
    DEBUG = "debug"
    MISSION_0 = "mission_0"
    MISSION_1 = "mission_1"
    MISSION_2 = "mission_2"
    MISSION_3 = "mission_3"


def launch_setup(context) -> list[Action]:
    config = context.launch_configurations  # dict containing declared launch arguments
    debug = config[Args.DEBUG]
    swarm_logger = get_logger(
        "uavs_sitl.launch", logging.DEBUG if is_truthy(debug) else logging.INFO
    )
    world = config[Args.WORLD]
    headless = config[Args.HEADLESS]
    mission_0 = config[Args.MISSION_0]
    mission_1 = config[Args.MISSION_1]
    mission_2 = config[Args.MISSION_2]
    mission_3 = config[Args.MISSION_3]

    actions: list[Action] = [
        include_launch(
            "sim",
            "sim2.launch.py",
            launch_arguments={
                "world": world,
                "headless": headless,
                "debug": debug,
            },
        ),
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
            output="screen",
            name="fleet_micro_xrce_agent",
        ),
        include_launch(
            "uav",
            "uav_sitl.launch.py",
            launch_arguments={
                "ns_id": "0",
                "mission": mission_0,
                "world": world,
                "model_pose": "0,0",
                "standalone": "false",
                "run_mw": "false",
                "launch_sim": "false",
                "headless": headless,
                "debug": debug,
            },
        ),
        include_launch(
            "uav",
            "uav_sitl.launch.py",
            launch_arguments={
                "ns_id": "1",
                "mission": mission_1,
                "world": world,
                "model_pose": "5,0",
                "standalone": "false",
                "run_mw": "false",
                "launch_sim": "false",
                "headless": headless,
                "debug": debug,
            },
        ),
        include_launch(
            "uav",
            "uav_sitl.launch.py",
            launch_arguments={
                "ns_id": "2",
                "mission": mission_2,
                "world": world,
                "model_pose": "10,0",
                "standalone": "false",
                "run_mw": "false",
                "launch_sim": "false",
                "headless": headless,
                "debug": debug,
            },
        ),
        include_launch(
            "uav",
            "uav_sitl.launch.py",
            launch_arguments={
                "ns_id": "3",
                "mission": mission_3,
                "world": world,
                "model_pose": "15,0",
                "standalone": "false",
                "run_mw": "false",
                "launch_sim": "false",
                "headless": headless,
                "debug": debug,
            },
        ),
    ]
    swarm_logger.info(
        "Launching in %s world",
        world,
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                Args.WORLD,
                default_value="default",
                description="Gazebo world shared by all SITL vehicles.",
            ),
            DeclareLaunchArgument(
                Args.HEADLESS,
                default_value="false",
                choices=["true", "false", "t", "f", "0", "1"],
                description="Run Gazebo without a GUI.",
            ),
            DeclareLaunchArgument(
                Args.DEBUG,
                default_value="false",
                choices=["true", "false", "t", "f", "0", "1"],
                description="Enable debug logging.",
            ),
            DeclareLaunchArgument(
                Args.MISSION_0,
                default_value="basic",
                description="Mission for uav_0. LOCAL coordinates are relative to uav_0's origin.",
            ),
            DeclareLaunchArgument(
                Args.MISSION_1,
                default_value="basic_waypoints",
                description="Mission for uav_1. LOCAL coordinates are relative to uav_1's origin.",
            ),
            DeclareLaunchArgument(
                Args.MISSION_2,
                default_value="basic",
                description="Mission for uav_2. LOCAL coordinates are relative to uav_2's origin.",
            ),
            DeclareLaunchArgument(
                Args.MISSION_3,
                default_value="basic_waypoints",
                description="Mission for uav_3. LOCAL coordinates are relative to uav_3's origin.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
