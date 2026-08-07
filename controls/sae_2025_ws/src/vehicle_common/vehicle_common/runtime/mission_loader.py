import os
from typing import Any
from pathlib import Path

from rclpy.node import Node
from pydantic import BaseModel, ConfigDict, PrivateAttr
from ament_index_python.packages import get_package_share_directory

import yaml

from vehicle_common.vehicle import Vehicle
from vehicle_common.mode import Mode
from vehicle_common.base import VisionNode
from vehicle_common.mode_loader import RegisteredMode, ModeRegistry


class RuntimeMode(BaseModel):
    model_config = ConfigDict(extra="forbid")

    mode: str
    transitions: dict[str, str] = {}
    params: dict[str, Any] = {}
    _validated_params: BaseModel = PrivateAttr()
    _registered: RegisteredMode = PrivateAttr()

    def model_post_init(self, context: Any, /) -> None:
        mode_registry = ModeRegistry.get()
        self._registered = mode_registry.get_registered_mode(self.mode)
        self._validated_params = self._registered.params_cls.model_validate(self.params)

    def instantiate_mode(self, node: Node, vehicle: Vehicle) -> Mode:
        mode = self._registered.mode_cls()
        mode.initialize(node, vehicle, self._validated_params)
        return mode


class RuntimeMission(BaseModel):
    model_config = ConfigDict(extra="forbid")

    modes: dict[str, RuntimeMode]

    _targets: set[type[Vehicle]] = PrivateAttr()
    _vision_nodes: set[type[VisionNode]] = PrivateAttr()
    _peer_vehicle_names: set[str] = PrivateAttr()
    _requires_camera: bool = PrivateAttr()

    def model_post_init(self, context: Any, /) -> None:
        target_sets = []
        vision_sets = []
        peer_vehicle_sets = []
        requires_camera = False

        for m in self.modes.values():
            target_sets.append(set(m._registered.targets))
            vision_sets.append(set(m._registered.required_vision_nodes))
            peer_vehicle_sets.append(set(m._registered.peer_vehicle_names))
            requires_camera |= m._registered.requires_camera

        self._targets = set.intersection(*target_sets)
        self._vision_nodes = set.union(*vision_sets)
        self._peer_vehicle_names = set.union(*peer_vehicle_sets)
        self._requires_camera = requires_camera

    @classmethod
    def load_from_path(cls, path: Path | str):
        with open(path, "r") as f:
            return cls.model_validate(yaml.safe_load(f))


def get_mission_path(mission_name: str, package: str) -> str:
    # gets path in package's install directory where missions are installed to
    mission_path = Path(get_package_share_directory(package)) / "missions" / f"{mission_name}.yaml"
    if not os.path.isfile(mission_path):
        raise FileNotFoundError(f"Mission {mission_name} was not found at {mission_path}")

    return str(mission_path)
