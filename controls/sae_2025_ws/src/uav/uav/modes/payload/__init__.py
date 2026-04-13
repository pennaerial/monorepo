from importlib import import_module

__all__ = [
    "PayloadAprilTagApproachMode",
    "PayloadColorStringApproachMode",
    "PayloadRetreatMode",
]


def __getattr__(name: str):
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(f".{name}", __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value
