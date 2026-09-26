from vehicle_common.mode import Mode
from vehicle_common.mode_loader import (
    ParamsBase,
    register_mode,
)
from vehicle_common.vehicle import Vehicle


class MockVehicle(Vehicle):
    pass


class MockParams(ParamsBase):
    pass


@register_mode(id="mock", params_cls=MockParams, targets=[MockVehicle])
class MockMode(Mode):
    pass


@register_mode(id="no_params", targets=[MockVehicle])
class NoParamsMock(Mode):
    pass


class MockVerticalTakeoffParams(ParamsBase):
    takeoff_height: float = 5.0
    takeoff_method: str = "PX4_AUTO"


class MockRequiredParams(ParamsBase):
    required_field: float


@register_mode(
    id="mock.VerticalTakeoffMode",
    params_cls=MockVerticalTakeoffParams,
    targets=[MockVehicle],
    peer_vehicle_names=["peer1"],
    transition_labels=["complete"],
)
class MockVerticalTakeoffMode(Mode):
    """Copy of uav.modes.VerticalTakeoffMode.VerticalTakeoffMode, stripped of its
    PX4/ROS dependencies so it can be registered and validated in plain-Python tests."""


@register_mode(
    id="mock.loiter",
    params_cls=MockParams,
    targets=[MockVehicle],
    peer_vehicle_names=["peer1"],
)
class MockLoiterMode(Mode):
    """Second mock mode sharing the vehicle target and peer metadata."""


@register_mode(
    id="mock.required_params",
    params_cls=MockRequiredParams,
    targets=[MockVehicle],
)
class MockRequiredParamsMode(Mode):
    pass
