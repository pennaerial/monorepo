from launch import Action, LaunchDescription
from launch.actions import OpaqueFunction
from vehicle_common.launch_utils import get_logger


def launch_setup(context) -> list[Action]:
    logger = get_logger("vision_launch")

    logger.debug("LAUNCH PARAMS")
    logger.warning("This launch file is unimplemented!")

    ## create actions
    actions = []
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
