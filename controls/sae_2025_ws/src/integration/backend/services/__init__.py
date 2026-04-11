from importlib import import_module

__all__ = ["wifi", "deploy", "mission"]

_EXPORTS = {
    "deploy": ".deploy",
    "mission": ".mission",
    "wifi": ".wifi",
}


def __getattr__(name: str):
    module_path = _EXPORTS.get(name)
    if module_path is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(module_path, __name__)
    globals()[name] = module
    return module
