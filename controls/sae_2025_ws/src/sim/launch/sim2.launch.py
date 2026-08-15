import os
from enum import StrEnum
from pathlib import Path

from launch import LaunchDescription, Action
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
)

from vehicle_common.launch_utils import (
    get_logger,
    LaunchError,
    is_truthy,
    format_bullet_list,
    prepend_env_path,
)

from sim.utils import get_available_worlds
from sim.simulation_params import SimulationParams

logger = get_logger("sim.launch")

# TODO: Make these get envs during launch a helper
PENNAIR_GZ_MODELS_PATH = os.environ.get("PENNAIR_GZ_MODELS_PATH", "")
if not PENNAIR_GZ_MODELS_PATH:
    raise LaunchError(
        "PENNAIR_GZ_MODELS_PATH is not set. Please source dev_env.sh or manually set it"
    )

PENNAIR_PX4_PATH = os.environ.get("PENNAIR_PX4_PATH", "")
if not PENNAIR_PX4_PATH:
    raise LaunchError("PENNAIR_PX4_PATH is not set. Please source dev_env.sh or manually set it")

GZ_SIM_RESOURCE_PATH = Path(PENNAIR_GZ_MODELS_PATH) / "models"
# Prepends value to existing value, bc we don't want to overwrite any user-set paths
GZ_SIM_RESOURCE_PATH = prepend_env_path("GZ_SIM_RESOURCE_PATH", str(GZ_SIM_RESOURCE_PATH))

GZ_SIM_SERVER_CONFIG_PATH = Path(PENNAIR_GZ_MODELS_PATH) / "server.config"

# .so plugins include: libOpticalFlowSystem.so, libGstCameraSystem.so, etc.
GZ_SIM_SYSTEM_PLUGIN_PATH = Path(PENNAIR_PX4_PATH) / "build" / "px4_sitl_default" / "src" / "modules" / "simulation" / "gz_plugins"  # fmt: skip
GZ_SIM_SYSTEM_PLUGIN_PATH = prepend_env_path("GZ_SIM_SYSTEM_PLUGIN_PATH", str(GZ_SIM_SYSTEM_PLUGIN_PATH))  # fmt: skip

# not a default GZ_SIM* env var, so no need to prepend
GZ_WORLDS_PATH = Path(PENNAIR_GZ_MODELS_PATH) / "worlds"

logger.debug(f"ENV VAR DETECTED: PENNAIR_GZ_MODELS_PATH={PENNAIR_GZ_MODELS_PATH}")
logger.debug(f"ENV VAR DETECTED: PENNAIR_PX4_PATH={PENNAIR_PX4_PATH}")
logger.debug(f"GZ_SIM_RESOURCE_PATH={GZ_SIM_RESOURCE_PATH}")
logger.debug(f"GZ_SIM_SERVER_CONFIG_PATH={GZ_SIM_SERVER_CONFIG_PATH}")
logger.debug(f"GZ_SIM_SYSTEM_PLUGIN_PATH={GZ_SIM_SYSTEM_PLUGIN_PATH}")


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    WORLD = "world"
    STAGE = "stage"
    HEADLESS = "headless"


def gz_sim_command(world: str, headless: bool) -> list[str]:
    world_path = GZ_WORLDS_PATH / f"{world}.sdf"
    cmd = ["gz", "sim", "--render-engine", "ogre", "-r", str(world_path)]
    if headless:
        cmd.extend(["-s", "--headless-rendering"])  # add flags for gz headless mode
    return cmd


def launch_setup(context) -> list[Action]:
    config = context.launch_configurations  # dict containing declared launch arguments

    headless: bool = is_truthy(config[Args.HEADLESS])
    world: str = config[Args.WORLD]
    stage: str = config[Args.STAGE]

    gz_env = {
        "GZ_SIM_RESOURCE_PATH": GZ_SIM_RESOURCE_PATH,
        "GZ_SIM_SERVER_CONFIG_PATH": GZ_SIM_SERVER_CONFIG_PATH,
        "GZ_SIM_SYSTEM_PLUGIN_PATH": GZ_SIM_SYSTEM_PLUGIN_PATH,
    }
    # TODO: These need to be documented
    if headless:  # what do these env vars do?
        gz_env["LIBGL_ALWAYS_SOFTWARE"] = "1"
        gz_env["QT_QPA_PLATFORM"] = "offscreen"
    else:
        gz_env["QT_QPA_PLATFORM_PLUGIN_PATH"] = ""
        gz_env["QT_QPA_FONTDIR"] = ""

    actions = []

    gz_sim = ExecuteProcess(
        cmd=gz_sim_command(world, headless),
        output="log",
        name="gz_sim",
        additional_env=gz_env,  # type: ignore (dict[str, str] works instead of SomeSubstitutionsType)
    )

    actions.append(gz_sim)
    logger.info(f"Launching world: {world}")

    try:
        simulation_params = SimulationParams.load_from_stage(world, stage)
        node_name = simulation_params.world.node
        world_node = Node(
            package="sim",
            executable=node_name,
            parameters=[
                {
                    "world": world,
                    "stage": stage,
                }
            ],
            output="screen",
            name=node_name,
        )
        actions.append(world_node)
        logger.info(f"Launching world node: {node_name}")

        spawn_entity_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[f"/world/{world}/create@ros_gz_interfaces/srv/SpawnEntity"],
            output="screen",
            name="spawn_entity_bridge",
        )
        actions.append(spawn_entity_bridge)
        logger.info("Launching entity bridge")
    except FileNotFoundError:
        # no configuration exists, so don't launch world node or any gz world bridges
        logger.warning(f"No configuration found for {{ world: {world}, stage: {stage} }} Not launching a world node")  # fmt: skip

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                Args.WORLD,
                default_value="default",
                description=format_bullet_list(
                    "simulation world for gz to load. Looks under {PENNAIR_GZ_MODELS_PATH}/worlds/{world}.sdf\n\tAvailable Worlds:",
                    options=get_available_worlds(GZ_WORLDS_PATH),
                ),
            ),
            DeclareLaunchArgument(
                Args.STAGE,
                default_value="base",
                description="Stage name for the specified world. Only applies if there is a world node for the world and if a world <stage> configuration is defined",
            ),
            DeclareLaunchArgument(
                Args.HEADLESS,
                default_value="false",
                description="If true, runs gz server in headless mode (no GUI)",
                choices=["true", "false", "t", "f", "0", "1"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
