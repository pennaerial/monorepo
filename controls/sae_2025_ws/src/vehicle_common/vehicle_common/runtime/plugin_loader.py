from collections import defaultdict
from typing import TypeVar
import importlib
import json
import pkgutil

T = TypeVar("T")

# BaseClass --> plugin_name --> PluginClass
_registry: defaultdict[type, dict[str, type]] = defaultdict(dict)


def register_plugin(name: str, base_cls: type):
    def decorator(plugin_cls: type):
        if not issubclass(plugin_cls, base_cls):
            raise TypeError(
                f"{plugin_cls.__name__} must inherit from {base_cls.__name__}"
            )
        name_to_plugin = _registry[base_cls]
        if name in name_to_plugin:
            raise ValueError(
                f"plugin with name: {name} already existsfor base class {base_cls}"
            )
        name_to_plugin[name] = plugin_cls
        return plugin_cls

    return decorator


def cls_to_str(cls: type):
    return f"{cls.__module__}.{cls.__qualname__}"


def get_plugins(base_cls: type[T]) -> dict[str, type[T]]:
    return _registry[base_cls]


def import_plugins(modules: tuple[str, ...] = ("uav", "payload")):
    for module in modules:
        try:
            pkg = importlib.import_module(module)
        except ImportError as e:
            print(e)
            continue

        if not hasattr(pkg, "__path__"):
            continue

        for _, name, _ in pkgutil.walk_packages(pkg.__path__, prefix=module + "."):
            try:
                importlib.import_module(name)
            except Exception as e:
                print(e)
                continue


def load_plugin_cls(base_cls: type[T], plugin_name: str) -> type[T]:
    try:
        return _registry[base_cls][plugin_name]
    except KeyError:
        available = ", ".join(_registry[base_cls].keys())

        raise KeyError(
            f"Plugin '{plugin_name}' not found for {base_cls.__name__}. "
            f"Available plugins for this base_class are {available}"
        )


def plugins_to_json() -> str:
    return json.dumps(
        {
            cls_to_str(base_cls): [
                {"name": name, "cls": cls_to_str(plugin_cls)}
                for name, plugin_cls in plugins.items()
            ]
            for base_cls, plugins in _registry.items()
        },
        indent=2,
    )


# import time as _time
# _t0 = _time.monotonic()

## import_plugins gets called when any module imports plugin_loader for the first time
import_plugins(("uav.modes", "payload.modes"))
# print(f"[plugin_loader] discovery took {(_time.monotonic() - _t0) * 1000:.1f}ms", flush=True)


def main():
    print(plugins_to_json())
