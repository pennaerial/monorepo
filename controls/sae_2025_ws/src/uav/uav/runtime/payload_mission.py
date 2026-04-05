from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .PayloadModeManager import PayloadModeManager
from .mission_spec import MissionSpec, mission_path_for_name


class PayloadMissionBootstrap(Node):
    def __init__(self) -> None:
        super().__init__("payload_mission_bootstrap")
        self.declare_parameter("mode_map", mission_path_for_name("basic"))
        self.declare_parameter("payload_name", "")

    def manager_kwargs(self) -> dict:
        mission_path = str(self.get_parameter("mode_map").value)
        if not mission_path:
            raise ValueError("payload_mission requires a non-empty 'mode_map'.")

        payload_name = str(self.get_parameter("payload_name").value).strip()
        if not payload_name:
            raise ValueError("payload_mission requires a non-empty 'payload_name'.")

        mission_spec = MissionSpec.load(mission_path)
        if not mission_spec.is_payload:
            raise ValueError(
                f"payload_mission requires a payload mission spec, received target '{mission_spec.target}'."
            )

        return {
            "mission_spec": mission_spec,
            "payload_name": payload_name,
            "node_name": "mission",
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    bootstrap = PayloadMissionBootstrap()
    mission_node = None

    try:
        manager_kwargs = bootstrap.manager_kwargs()
        bootstrap.destroy_node()
        bootstrap = None
        mission_node = PayloadModeManager(**manager_kwargs)
        rclpy.spin(mission_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if bootstrap is not None:
            bootstrap.destroy_node()
        if mission_node is not None:
            if rclpy.ok():
                mission_node._stop_vehicle()
            mission_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
