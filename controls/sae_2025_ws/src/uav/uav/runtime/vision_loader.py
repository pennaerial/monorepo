from __future__ import annotations

import importlib


def load_vision_class(class_path: str):
    class_name = class_path.rsplit(".", 1)[-1]
    try:
        module = importlib.import_module(class_path)
    except ModuleNotFoundError as exc:
        if exc.name != class_path:
            raise
        module = importlib.import_module(class_path.rsplit(".", 1)[0])

    vision_class = getattr(module, class_name)

    from uav.vision_nodes.VisionNode import VisionNode

    if not isinstance(vision_class, type) or not issubclass(vision_class, VisionNode):
        raise TypeError(
            f"Vision node path '{class_path}' did not resolve to a VisionNode subclass."
        )
    return vision_class


def canonical_vision_node_path(vision_node: object | str) -> str:
    if isinstance(vision_node, str):
        text = vision_node.strip()
        if not text:
            raise ValueError("Vision node path cannot be empty.")
        if "." not in text:
            raise ValueError(
                f"Vision node path '{text}' is not canonical. Use a full import path."
            )
        vision_class = load_vision_class(text)
        canonical = canonical_vision_node_path(vision_class)
        if canonical != text:
            raise ValueError(
                f"Vision node path '{text}' is not canonical. Use '{canonical}'."
            )
        return canonical

    module_path = str(getattr(vision_node, "__module__", "")).strip()
    class_name = str(getattr(vision_node, "__name__", "")).strip()
    if not module_path or not class_name:
        raise TypeError(
            f"Expected a VisionNode subclass or canonical import path, received {vision_node!r}."
        )
    if module_path.rsplit(".", 1)[-1] == class_name:
        return module_path
    return f"{module_path}.{class_name}"
