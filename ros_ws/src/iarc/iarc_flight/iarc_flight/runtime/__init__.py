"""Pieces shared by the modes of one mission: one instance per mode-manager node."""

from collections.abc import Callable
from typing import Any
from weakref import WeakKeyDictionary

_instances: WeakKeyDictionary[Any, dict[str, Any]] = WeakKeyDictionary()


def per_node(node: Any, key: str, make: Callable[[], Any] | None = None) -> Any:
    """The node's instance under `key`, made with `make` if missing (None without `make`)."""
    shared = _instances.setdefault(node, {})
    if key not in shared and make is not None:
        shared[key] = make()
    return shared.get(key)
