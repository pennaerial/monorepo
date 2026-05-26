from importlib import import_module
from typing import TYPE_CHECKING

__all__ = ["Mode"]

_EXPORTS = {
    "Mode": ".Mode",
}

if TYPE_CHECKING:
    from .Mode import Mode


def __getattr__(name: str):
    module_path = _EXPORTS.get(name)
    if module_path is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(module_path, __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value
