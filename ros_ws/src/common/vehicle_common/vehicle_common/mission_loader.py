import os
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from pydantic import BaseModel, ConfigDict, PrivateAttr

from vehicle_common.mode_loader import ModeRegistry, RegisteredMode
from vehicle_common.vehicle import Vehicle


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


class RuntimeMission(BaseModel):
    model_config = ConfigDict(extra="forbid")

    modes: dict[str, RuntimeMode]

    _targets: set[type[Vehicle]] = PrivateAttr()

    def model_post_init(self, context: Any, /) -> None:
        target_sets = []

        for m in self.modes.values():
            target_sets.append(set(m._registered.targets))

        self._targets = set.intersection(*target_sets)

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
