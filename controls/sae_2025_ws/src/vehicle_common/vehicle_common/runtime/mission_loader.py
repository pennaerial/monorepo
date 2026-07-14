from pydantic import BaseModel, ConfigDict, PrivateAttr
from vehicle_common.mode import Vehicle
from vehicle_common.mode_loader import RegisteredMode, get_registered_mode
from typing import Any


class RuntimeMode(BaseModel):
    model_config = ConfigDict(extra='forbid')

    mode: str
    transitions: dict[str, str] = {}
    params: dict[str, Any] = {}
    _validated_params: BaseModel = PrivateAttr()
    _registered: RegisteredMode = PrivateAttr()

    def model_post_init(self, context: Any, /) -> None:
        self._registered = get_registered_mode(self.id)
        mode_cls = self._registered.mode_cls
        self._validated_params = mode_cls.Params.model_validate(self.params)


class RuntimeMission(BaseModel):
    model_config = ConfigDict(extra='forbid')

    modes: dict[str, RuntimeMode]

    _targets: set[type[Vehicle]] = PrivateAttr()
    _vision_nodes: set[str] = PrivateAttr()
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
        self._vision_nodes = set.intersection(*vision_sets)
        self._peer_vehicle_names = set.intersection(*peer_vehicle_sets)
        self._requires_camera = requires_camera

