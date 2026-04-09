from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from uav.vehicles.AirframeClass import AirframeClass
from .UAVModeManager import UAVModeManager
from .mission_spec import MissionSpec, mission_path_for_name


class UAVMissionBootstrap(Node):
    def __init__(self) -> None:
        super().__init__("uav_mission_bootstrap")
        self.declare_parameter("mode_map", mission_path_for_name("basic"))
        self.declare_parameter("auto_launch", True)
        self.declare_parameter("debug", False)
        self.declare_parameter("servo_only", False)
        self.declare_parameter("vehicle_name", "uav")
        self.declare_parameter("vehicle_class", AirframeClass.MULTICOPTER.name)
        self.declare_parameter("uav_camera_offsets", [0.0, 0.0, 0.0])

    def _bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if not isinstance(value, bool):
            raise ValueError(
                f"uav_mission requires boolean parameter '{name}', received {value!r}."
            )
        return value

    def manager_kwargs(self) -> dict:
        mission_path = str(self.get_parameter("mode_map").value)
        if not mission_path:
            raise ValueError("uav_mission requires a non-empty 'mode_map'.")

        mission_spec = MissionSpec.load(mission_path)
        if not mission_spec.is_uav:
            raise ValueError(
                f"uav_mission requires a UAV mission spec, received target '{mission_spec.target}'."
            )

        return {
            "mission_spec": mission_spec,
            "auto_launch": self._bool_parameter("auto_launch"),
            "debug": bool(self.get_parameter("debug").value),
            "servo_only": bool(self.get_parameter("servo_only").value),
            "vehicle_name": str(self.get_parameter("vehicle_name").value).strip()
            or "uav",
            "vehicle_class": AirframeClass.parse(
                self.get_parameter("vehicle_class").value
            ),
            "camera_offsets": list(self.get_parameter("uav_camera_offsets").value),
            "node_name": "mission",
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    bootstrap = UAVMissionBootstrap()
    mission_node = None

    try:
        manager_kwargs = bootstrap.manager_kwargs()
        bootstrap.destroy_node()
        bootstrap = None
        mission_node = UAVModeManager(**manager_kwargs)
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
