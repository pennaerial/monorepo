from pydantic import BaseModel

from vehicle_common.vehicle import Vehicle
from vehicle_common.mode import Mode
from vehicle_common.base import VisionNode
from vehicle_common.mode_loader import (
    register_mode,
)


class MockVehicle(Vehicle):
    pass


class MockVisionNode(VisionNode):
    pass


class MockParams(BaseModel):
    pass


@register_mode(id="mock", params_cls=MockParams, targets=[MockVehicle])
class MockMode(Mode):
    pass

@register_mode(id="no_params", targets=[MockVehicle])
class NoParamsMock(Mode):
    pass

class MockVerticalTakeoffParams(BaseModel):
    takeoff_height: float = 5.0
    takeoff_method: str = "PX4_AUTO"


@register_mode(
    id="mock.VerticalTakeoffMode",
    params_cls=MockVerticalTakeoffParams,
    targets=[MockVehicle],
    required_vision_nodes=[MockVisionNode],
    peer_vehicle_names=["peer1"],
    requires_camera=True,
    transition_labels=["complete"],
)
class MockVerticalTakeoffMode(Mode):
    """Copy of uav.modes.VerticalTakeoffMode.VerticalTakeoffMode, stripped of its
    PX4/ROS dependencies so it can be registered and validated in plain-Python tests."""


@register_mode(
    id="mock.loiter",
    params_cls=MockParams,
    targets=[MockVehicle],
    required_vision_nodes=[MockVisionNode],
    peer_vehicle_names=["peer1"],
)
class MockLoiterMode(Mode):
    """Second mock mode sharing vision/peer requirements with MockVerticalTakeoffMode
    but not requiring a camera, so RuntimeMission's intersection/union aggregation
    across modes can be exercised."""
