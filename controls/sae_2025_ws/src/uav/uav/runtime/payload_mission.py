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
        self.declare_parameter("auto_launch", True)
        self.declare_parameter("vehicle_name", "")

    def _bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if not isinstance(value, bool):
            raise ValueError(
                f"payload_mission requires boolean parameter '{name}', received {value!r}."
            )
        return value

    def manager_kwargs(self) -> dict:
        mission_path = str(self.get_parameter("mode_map").value)
        if not mission_path:
            raise ValueError("payload_mission requires a non-empty 'mode_map'.")

        vehicle_name = str(self.get_parameter("vehicle_name").value).strip()
        if not vehicle_name:
            raise ValueError("payload_mission requires a non-empty 'vehicle_name'.")

        mission_spec = MissionSpec.load(mission_path)
        if not mission_spec.is_payload:
            raise ValueError(
                f"payload_mission requires a payload mission spec, received target '{mission_spec.target}'."
            )

        return {
            "mission_spec": mission_spec,
            "auto_launch": self._bool_parameter("auto_launch"),
            "vehicle_name": vehicle_name,
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
            mission_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
