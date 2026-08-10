from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from typing import cast
import importlib


def serialize_type(cls: type) -> str:
    """Returns the full import path of the provided type"""

    return f"{cls.__module__}:{cls.__qualname__}"


def deserialize_type(path: str, base_cls: type | None = None) -> type:
    """Resolve a serialized "module:QualName" path back into a class object.

    Tries to instantiate the class after importing the module.

    Args:
        path: Serialized type reference in "module.path:QualName" form,
            as produced by `serialize_type`.
        base_cls: If provided, validates that the type is a subclass of base_cls

    Returns:
        The resolved class object.

    Raises:
        ValueError: If `path` isn't in "module:QualName" form, or the
            resolved object is not a subclass of `base_cls`.
    """

    try:
        module_path, class_name = path.split(":", 1)
    except ValueError:
        raise ValueError(f"Invalid type path '{path}'. Expected 'module:ClassName'")

    module = importlib.import_module(module_path)
    obj = module
    for part in class_name.split("."):
        obj = getattr(obj, part)

    obj = cast(type, obj)
    if base_cls:
        if not issubclass(obj, base_cls):
            raise ValueError(f"{path} must be a {base_cls.__name__}")

    return obj


def get_available_missions(package: str) -> list[str]:
    path = Path(get_package_share_directory(package)) / "missions"
    return [mission.stem for mission in path.glob("*.yaml")]
