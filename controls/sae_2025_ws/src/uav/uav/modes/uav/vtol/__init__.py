from importlib import import_module
from typing import TYPE_CHECKING

__all__ = ["TakeoffMode", "TransitionMode"]

if TYPE_CHECKING:
    from .TakeoffMode import TakeoffMode
    from .TransitionMode import TransitionMode


def __getattr__(name: str):
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(f".{name}", __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value
