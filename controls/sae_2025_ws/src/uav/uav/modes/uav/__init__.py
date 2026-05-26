from importlib import import_module
from typing import TYPE_CHECKING

__all__ = [
    "LandingMode",
    "NavGPSMode",
    "PayloadDropoffMode",
    "PayloadPickupMode",
    "ServoDropoffMode",
    "TransitionMode",
    "WaypointMission",
]

if TYPE_CHECKING:
    from .LandingMode import LandingMode
    from .NavGPSMode import NavGPSMode
    from .PayloadDropoffMode import PayloadDropoffMode
    from .PayloadPickupMode import PayloadPickupMode
    from .ServoDropoffMode import ServoDropoffMode
    from .TransitionMode import TransitionMode
    from .WaypointMission import WaypointMission


def __getattr__(name: str):
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(f".{name}", __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value
