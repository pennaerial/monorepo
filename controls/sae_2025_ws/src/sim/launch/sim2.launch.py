import os
from enum import StrEnum
from pathlib import Path

from launch import LaunchDescription, Action
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

logger = get_logger("sim.launch")

# TODO: Make these get envs during launch a helper
PENNAIR_GZ_MODELS_PATH = os.environ.get("PENNAIR_GZ_MODELS_PATH", "")
if not PENNAIR_GZ_MODELS_PATH:
    raise LaunchError(
        "PENNAIR_GZ_MODELS_PATH is not set. Please source dev_env.sh or manually set it"
    )

PENNAIR_PX4_PATH = os.environ.get("PENNAIR_PX4_PATH", "")
if not PENNAIR_PX4_PATH:
    raise LaunchError(
        "PENNAIR_PX4_PATH is not set. Please source dev_env.sh or manually set it"
    )

GZ_SIM_RESOURCE_PATH = Path(PENNAIR_GZ_MODELS_PATH) / "models"
# Prepends value to existing value, bc we don't want to overwrite any user-set paths
GZ_SIM_RESOURCE_PATH = prepend_env_path(
    "GZ_SIM_RESOURCE_PATH", str(GZ_SIM_RESOURCE_PATH)
)

GZ_SIM_SERVER_CONFIG_PATH = (
    Path(PENNAIR_GZ_MODELS_PATH) / "server.config"
)  # no need to prepend because points to file

# .so plugins include: libOpticalFlowSystem.so, libGstCameraSystem.so, etc.
GZ_SIM_SYSTEM_PLUGIN_PATH = (
    Path(PENNAIR_PX4_PATH)
    / "build"
    / "px4_sitl_default"
    / "src"
    / "modules"
    / "simulation"
    / "gz_plugins"
)  # noqa: F401
GZ_SIM_SYSTEM_PLUGIN_PATH = prepend_env_path(
    "GZ_SIM_SYSTEM_PLUGIN_PATH", str(GZ_SIM_SYSTEM_PLUGIN_PATH)
)  # noqa: F401

GZ_WORLDS_PATH = (
    Path(PENNAIR_GZ_MODELS_PATH) / "worlds"
)  # not a default GZ_SIM* env var, so no need to prepend

logger.debug(f"ENV VAR DETECTED: PENNAIR_GZ_MODELS_PATH={PENNAIR_GZ_MODELS_PATH}")
logger.debug(f"ENV VAR DETECTED: PENNAIR_PX4_PATH={PENNAIR_PX4_PATH}")
logger.debug(f"GZ_SIM_RESOURCE_PATH={GZ_SIM_RESOURCE_PATH}")
logger.debug(f"GZ_SIM_SERVER_CONFIG_PATH={GZ_SIM_SERVER_CONFIG_PATH}")
logger.debug(f"GZ_SIM_SYSTEM_PLUGIN_PATH={GZ_SIM_SYSTEM_PLUGIN_PATH}")


class Args(StrEnum):
    """Maps constants to launch argument keyords"""

    WORLD = "world"
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

    gz = ExecuteProcess(
        cmd=gz_sim_command(world, headless),
        output="log",
        name=f"gz_{world}",
        additional_env=gz_env,  # type: ignore (dict[str, str] works instead of SomeSubstitutionsType)
    )

    logger.info(f"Creating world: {world}")
    return [gz]


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
                Args.HEADLESS,
                default_value="false",
                description="If true, runs gz server in headless mode (no GUI)",
                choices=["true", "false", "t", "f", "0", "1"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
