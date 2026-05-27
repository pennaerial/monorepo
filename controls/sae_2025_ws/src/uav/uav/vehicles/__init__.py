from importlib import import_module
from typing import TYPE_CHECKING

__all__ = [
    "AirframeClass",
    "Payload",
    "Vehicle",
    "UAV",
    "VTOL",
    "Multicopter",
]

if TYPE_CHECKING:
    from .AirframeClass import AirframeClass
    from .Multicopter import Multicopter
    from .Payload import Payload
    from .UAV import UAV
    from .VTOL import VTOL
    from .Vehicle import Vehicle


def __getattr__(name: str):
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(f".{name}", __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value
