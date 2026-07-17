import logging
import os
from enum import StrEnum
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

PA_LAUNCH_DEBUG = os.getenv("PA_LAUNCH_DEBUG", "0").lower() == "1"

RED = "\033[31m"
CYAN = "\033[36m"
RESET = "\033[0m"


class PennairFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        msg = super().format(record)

        if record.levelno == logging.DEBUG:
            return f"{CYAN}{msg}{RESET}"

        return msg


def get_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    handler = logging.StreamHandler()

    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(
            PennairFormatter("[PENNAIR] [%(levelname)s] [%(name)s] %(message)s")
        )
        logger.addHandler(handler)
        logger.propagate = False
    logger.setLevel(logging.DEBUG if PA_LAUNCH_DEBUG else logging.INFO)

    return logger


class LaunchError(Exception):
    def __init__(self, msg: str):
        super().__init__(f"{RED}{msg}{RESET}")


def check_unknown_launch_args(
    arg_enum: type[StrEnum], launch_configurations: dict, logger: logging.Logger
):
    """Displays a warning if any undeclared launch arguments are present.

    Args:
        arg_enum: StrEnum containing the set of valid launch argument names.
        launch_configurations: Launch configuration mapping from the launch context.
    """

    valid_args = {arg.value for arg in arg_enum}
    unknown = set(launch_configurations) - valid_args
    if unknown:
        logger.warning(
            f"Unknown launch argument(s): {', '.join(sorted(unknown))}\n"
            "Run `ros2 launch <package> <launch_file> --show-args` to see valid arguments."
        )


def include_launch(pkg: str, launch_file: str) -> IncludeLaunchDescription:
    path = Path(get_package_share_directory(pkg)) / "launch" / launch_file
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))


def format_bullet_list(title: str, options: list[str]) -> str:
    """Formats a list of strings as a bullet list in the description section of a declared launch arguments
    Because ROS launch matches strings for the choices argumnet, you must leave choices empty and validate
    yourself
    """

    return f"{title}\n" + "\n".join(f"\t  • {option}" for option in options)
