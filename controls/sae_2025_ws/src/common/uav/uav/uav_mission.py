from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException

from uav.UAVModeManager import UAVModeManager


def main(args=None) -> None:
    rclpy.init(args=args)
    mission_node = None

    try:
        mission_node = UAVModeManager()
        rclpy.spin(mission_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if mission_node is not None:
            mission_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
