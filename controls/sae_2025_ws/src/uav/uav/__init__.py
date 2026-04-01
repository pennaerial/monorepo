from importlib import import_module

__all__ = ["AirframeClass", "Payload", "Vehicle", "UAV", "VTOL", "Multicopter"]

_EXPORTS = {
    "AirframeClass": ".vehicles.AirframeClass",
    "Payload": ".vehicles.Payload",
    "Vehicle": ".vehicles.Vehicle",
    "UAV": ".vehicles.UAV",
    "VTOL": ".vehicles.VTOL",
    "Multicopter": ".vehicles.Multicopter",
}


def __getattr__(name: str):
    module_path = _EXPORTS.get(name)
    if module_path is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(module_path, __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value
