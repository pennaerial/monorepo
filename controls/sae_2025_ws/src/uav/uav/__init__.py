from importlib import import_module

__all__ = ["AirframeClass", "Payload", "Vehicle", "UAV", "VTOL", "Multicopter"]

_EXPORTS = {
    "AirframeClass": "AirframeClass",
    "Payload": "Payload",
    "Vehicle": "Vehicle",
    "UAV": "UAV",
    "VTOL": "VTOL",
    "Multicopter": "Multicopter",
}


def __getattr__(name: str):
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(f".{module_name}", __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value
