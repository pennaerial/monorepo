from __future__ import annotations


def canonical_mode_path(mode_or_class: object) -> str:
    if isinstance(mode_or_class, str):
        return mode_or_class
    mode_class = (
        mode_or_class if isinstance(mode_or_class, type) else type(mode_or_class)
    )
    return f"{mode_class.__module__}.{mode_class.__name__}"
