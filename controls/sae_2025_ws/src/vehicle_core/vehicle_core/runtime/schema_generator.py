from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from enum import Enum
import importlib
import inspect
import json
import math
import pkgutil
from pathlib import Path
import types
from typing import Any, Literal, Union, get_args, get_origin, get_type_hints

from pydantic import BaseModel, ConfigDict, create_model

from vehicle_core.mode import Mode

from .mode_paths import (
    implementation_mode_path,
    mission_target_from_mode_id,
    mode_id_for,
)
from .schema_registry import (
    ModeParamFieldSpec,
    ModeRegistryDocument,
    ModeRegistryEntry,
    dump_mode_registry_document,
    registry_path,
)


def _doc_summary(obj: object) -> str:
    doc = inspect.getdoc(obj) or ""
    return doc.strip().split("\n", 1)[0] if doc.strip() else ""


def _is_typed_dict(annotation: object) -> bool:
    return (
        isinstance(annotation, type)
        and issubclass(annotation, dict)
        and hasattr(annotation, "__required_keys__")
        and hasattr(annotation, "__optional_keys__")
    )


def _iter_mode_classes() -> list[type[Mode[Any]]]:
    import uav.modes as mode_package

    discovered: dict[str, type[Mode[Any]]] = {}
    for module_info in pkgutil.walk_packages(
        mode_package.__path__, prefix=f"{mode_package.__name__}."
    ):
        try:
            module = importlib.import_module(module_info.name)
        except ImportError as exc:
            print(
                f"WARNING: skipping mode module '{module_info.name}' "
                f"(missing dependency: {exc})",
                file=sys.stderr,
            )
            continue
        for value in vars(module).values():
            if (
                isinstance(value, type)
                and issubclass(value, Mode)
                and value is not Mode
                and not inspect.isabstract(value)
                and value.__module__ == module.__name__
            ):
                discovered[mode_id_for(value)] = value
    return [discovered[key] for key in sorted(discovered)]


def _normalized_annotation(
    mode_class: type[Mode[Any]], *, name: str, annotation: object, default: object
) -> object:
    if annotation in (inspect.Parameter.empty, None, Any, object):
        if default not in (inspect.Parameter.empty, None) and type(default) in {
            bool,
            int,
            float,
            str,
        }:
            return type(default)
        raise TypeError(
            f"Mode '{mode_id_for(mode_class)}' parameter '{name}' requires an explicit schema-grade type annotation."
        )

    if annotation in {dict, list, tuple, set}:
        raise TypeError(
            f"Mode '{mode_id_for(mode_class)}' parameter '{name}' must use a typed collection annotation, not bare '{annotation.__name__}'."
        )

    origin = get_origin(annotation)
    args = get_args(annotation)
    if origin in {dict, list, tuple, set} and not args:
        raise TypeError(
            f"Mode '{mode_id_for(mode_class)}' parameter '{name}' must declare collection element types."
        )
    return annotation


def mode_params_model(mode_class: type[Mode[Any]]) -> type[BaseModel]:
    signature = inspect.signature(mode_class.__init__)
    type_hints = get_type_hints(mode_class.__init__)
    field_definitions: dict[str, tuple[object, object]] = {}

    for name, parameter in signature.parameters.items():
        if name in {"self", "node", "vehicle"}:
            continue
        if parameter.kind in {
            inspect.Parameter.VAR_POSITIONAL,
            inspect.Parameter.VAR_KEYWORD,
        }:
            continue
        annotation = _normalized_annotation(
            mode_class,
            name=name,
            annotation=type_hints.get(name, parameter.annotation),
            default=parameter.default,
        )
        default = (
            ... if parameter.default is inspect.Parameter.empty else parameter.default
        )
        field_definitions[name] = (annotation, default)

    return create_model(
        f"{mode_class.__name__}Params",
        __config__=ConfigDict(extra="forbid"),
        __module__=mode_class.__module__,
        **field_definitions,
    )


def _normalize_default_value(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, tuple):
        return [_normalize_default_value(item) for item in value]
    if isinstance(value, list):
        return [_normalize_default_value(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _normalize_default_value(item) for key, item in value.items()}
    return value


def _sanitize_json_schema(value: Any) -> Any:
    if isinstance(value, dict):
        next_value: dict[str, Any] = {}
        for key, item in value.items():
            if key == "default" and isinstance(item, float) and math.isnan(item):
                continue
            next_value[key] = _sanitize_json_schema(item)
        return next_value
    if isinstance(value, list):
        return [_sanitize_json_schema(item) for item in value]
    return value


def _default_payload(default: object) -> tuple[str, Any | None]:
    if default is inspect.Parameter.empty:
        return "missing", None
    if default is None:
        return "none", None
    if isinstance(default, float) and math.isnan(default):
        return "nan", None
    return "value", _normalize_default_value(default)


def _normalize_annotation_spec(annotation: object) -> dict[str, Any]:
    if annotation is bool:
        return {"kind": "bool"}
    if annotation is int:
        return {"kind": "int"}
    if annotation is float:
        return {"kind": "float"}
    if annotation is str:
        return {"kind": "str"}
    if annotation is type(None):
        return {"kind": "none"}

    if _is_typed_dict(annotation):
        type_hints = get_type_hints(annotation)
        required_keys = set(annotation.__required_keys__)
        optional_keys = set(annotation.__optional_keys__)
        fields = []
        for name in sorted(required_keys | optional_keys):
            fields.append(
                {
                    "name": name,
                    "required": name in required_keys,
                    "annotation": _normalize_annotation_spec(type_hints[name]),
                }
            )
        return {
            "kind": "typed_dict",
            "name": annotation.__name__,
            "fields": fields,
        }

    if inspect.isclass(annotation) and issubclass(annotation, Enum):
        return {
            "kind": "literal",
            "choices": [
                _normalize_default_value(member.value) for member in annotation
            ],
        }

    origin = get_origin(annotation)
    args = get_args(annotation)

    if origin is Literal:
        return {
            "kind": "literal",
            "choices": [_normalize_default_value(choice) for choice in args],
        }
    if origin is list:
        return {"kind": "list", "item": _normalize_annotation_spec(args[0])}
    if origin is dict:
        return {
            "kind": "dict",
            "key": _normalize_annotation_spec(args[0]),
            "value": _normalize_annotation_spec(args[1]),
        }
    if origin is tuple:
        items = list(args)
        variadic = len(items) == 2 and items[1] is Ellipsis
        if variadic:
            items = items[:1]
        return {
            "kind": "tuple",
            "items": [_normalize_annotation_spec(item) for item in items],
            "variadic": variadic,
        }
    if origin in {Union, types.UnionType}:
        return {
            "kind": "union",
            "options": [_normalize_annotation_spec(option) for option in args],
        }

    raise TypeError(f"Unsupported schema annotation: {annotation!r}")


def _field_specs_for_mode(
    mode_class: type[Mode[Any]],
) -> tuple[ModeParamFieldSpec, ...]:
    signature = inspect.signature(mode_class.__init__)
    type_hints = get_type_hints(mode_class.__init__)
    fields: list[ModeParamFieldSpec] = []

    for name, parameter in signature.parameters.items():
        if name in {"self", "node", "vehicle"}:
            continue
        if parameter.kind in {
            inspect.Parameter.VAR_POSITIONAL,
            inspect.Parameter.VAR_KEYWORD,
        }:
            continue

        annotation = _normalized_annotation(
            mode_class,
            name=name,
            annotation=type_hints.get(name, parameter.annotation),
            default=parameter.default,
        )
        default_kind, default_value = _default_payload(parameter.default)
        fields.append(
            ModeParamFieldSpec(
                name=name,
                annotation=_normalize_annotation_spec(annotation),
                required=parameter.default is inspect.Parameter.empty,
                default_kind=default_kind,
                default=default_value,
            )
        )

    return tuple(fields)


def build_mode_registry_entry(mode_class: type[Mode[Any]]) -> ModeRegistryEntry:
    mode_id = mode_id_for(mode_class)
    mission_target = mission_target_from_mode_id(mode_id)
    if mission_target not in {"uav", "payload"}:
        raise ValueError(
            f"Mode '{mode_id}' must live under the 'uav' or 'payload' public mode namespace."
        )
    peer_vehicle_names = tuple(
        sorted(
            {
                str(peer_name).strip()
                for peer_name in getattr(mode_class, "peer_vehicle_names", ())
                if str(peer_name).strip()
            }
        )
    )

    return ModeRegistryEntry(
        mode_id=mode_id,
        class_path=implementation_mode_path(mode_class),
        module_path=mode_class.__module__,
        class_name=mode_class.__name__,
        display_name=mode_class.__name__,
        description=_doc_summary(mode_class),
        mission_target=mission_target,
        required_vision_nodes=mode_class.required_vision_node_paths(),
        peer_vehicle_names=peer_vehicle_names,
        requires_camera=bool(getattr(mode_class, "requires_camera", False)),
        transition_labels=mode_class.declared_transition_labels(),
        params_schema=_sanitize_json_schema(
            mode_params_model(mode_class).model_json_schema()
        ),
        param_fields=_field_specs_for_mode(mode_class),
    )


def build_mode_registry_document() -> ModeRegistryDocument:
    entries = [
        build_mode_registry_entry(mode_class) for mode_class in _iter_mode_classes()
    ]
    return ModeRegistryDocument(
        modes={
            entry.mode_id: entry for entry in sorted(entries, key=lambda e: e.mode_id)
        }
    )


def refresh_schema_registry(*, path: Path | None = None) -> Path:
    return dump_mode_registry_document(build_mode_registry_document(), path=path)


def _registry_payload(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def check_schema_registry(*, path: Path | None = None) -> bool:
    expected = build_mode_registry_document().model_dump(mode="json")
    actual = _registry_payload(path or registry_path())
    return actual == expected


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate the committed UAV mode schema registry."
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Fail if the committed registry does not match the generated registry.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=registry_path(),
        help="Registry output path.",
    )
    args = parser.parse_args(argv)

    if args.check:
        if not check_schema_registry(path=args.output):
            print(
                f"Committed schema registry at '{args.output}' is stale. "
                "Regenerate it with `python3 -m uav.runtime.schema_generator`."
            )
            return 1
        print(f"Committed schema registry at '{args.output}' is up to date.")
        return 0

    output = refresh_schema_registry(path=args.output)
    print(f"Wrote mode schema registry to '{output}'.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
