from importlib import import_module
from typing import TYPE_CHECKING

__all__ = [
    "PayloadAprilTagApproachMode",
    "PayloadColorStringApproachMode",
    "PayloadDualApproachMode",
    "PayloadRetreatMode",
]

if TYPE_CHECKING:
    from .PayloadAprilTagApproachMode import PayloadAprilTagApproachMode
    from .PayloadColorStringApproachMode import PayloadColorStringApproachMode
    from .PayloadDualApproachMode import PayloadDualApproachMode
    from .PayloadRetreatMode import PayloadRetreatMode


def __getattr__(name: str):
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(f".{name}", __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value
