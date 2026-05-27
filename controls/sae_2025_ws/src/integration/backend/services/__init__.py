from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from . import deploy as deploy
    from . import fleet as fleet
    from . import mission as mission
    from . import wifi as wifi

__all__ = ["wifi", "deploy", "mission", "fleet"]

_EXPORTS = {
    "deploy": ".deploy",
    "fleet": ".fleet",
    "mission": ".mission",
    "wifi": ".wifi",
}


def __getattr__(name: str) -> Any:
    module_path = _EXPORTS.get(name)
    if module_path is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(module_path, __name__)
    globals()[name] = module
    return module
