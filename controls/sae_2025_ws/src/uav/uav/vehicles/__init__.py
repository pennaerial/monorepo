from importlib import import_module
from typing import TYPE_CHECKING

__all__ = [
    "AirframeClass",
    "PX4Airframe",
    "UAV",
    "VTOL",
    "Multicopter",
]

if TYPE_CHECKING:
    from .AirframeClass import AirframeClass, PX4Airframe
    from .Multicopter import Multicopter
    from .UAV import UAV
    from .VTOL import VTOL


def __getattr__(name: str):
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(f".{name}", __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value

# TOOD:  replace eventually with this after fixing integration
# from .AirframeClass import AirframeClass, PX4Airframe
# from .Multicopter import Multicopter
# from .UAV import UAV
# from .VTOL import VTOL
#
# __all__ = [
#     "AirframeClass",
#     "PX4Airframe",
#     "UAV",
#     "VTOL",
#     "Multicopter",
# ]
