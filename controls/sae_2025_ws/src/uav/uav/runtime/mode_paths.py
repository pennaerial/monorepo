from __future__ import annotations

MODE_PACKAGE_PREFIX = "uav.modes."


def implementation_mode_path(mode_or_class: object) -> str:
    if isinstance(mode_or_class, str):
        return str(mode_or_class).strip()

    mode_class = (
        mode_or_class if isinstance(mode_or_class, type) else type(mode_or_class)
    )
    module_path = str(mode_class.__module__).strip()
    class_name = str(mode_class.__name__).strip()
    if module_path.rsplit(".", 1)[-1] == class_name:
        return module_path
    return f"{module_path}.{class_name}"


def mode_id_from_class_path(class_path: str) -> str:
    normalized = str(class_path).strip()
    if not normalized:
        raise ValueError("Mode path cannot be empty.")
    if not normalized.startswith(MODE_PACKAGE_PREFIX):
        return normalized

    public_path = normalized[len(MODE_PACKAGE_PREFIX) :]
    module_path, _, class_name = public_path.rpartition(".")
    if module_path and module_path.rsplit(".", 1)[-1] == class_name:
        return module_path
    return public_path


def mode_id_for(mode_or_class: object) -> str:
    return mode_id_from_class_path(implementation_mode_path(mode_or_class))


def normalize_public_mode_id(mode_id: str) -> str:
    normalized = str(mode_id).strip()
    if not normalized:
        raise ValueError("Mode identifier cannot be empty.")
    if normalized.startswith(MODE_PACKAGE_PREFIX):
        suggested = mode_id_from_class_path(normalized)
        raise ValueError(
            f"Mode identifier '{normalized}' uses the internal Python path. Use '{suggested}' instead."
        )
    return normalized


def mission_target_from_mode_id(mode_id: str) -> str:
    normalized = normalize_public_mode_id(mode_id)
    prefix, _, _ = normalized.partition(".")
    return prefix


def canonical_mode_path(mode_or_class: object) -> str:
    return mode_id_for(mode_or_class)
