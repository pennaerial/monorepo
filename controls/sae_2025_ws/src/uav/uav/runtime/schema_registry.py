from __future__ import annotations

from functools import lru_cache, reduce
import json
import operator
from pathlib import Path
from typing import Any, Literal, cast

from pydantic import BaseModel, ConfigDict, create_model
from typing_extensions import NotRequired, Required, TypedDict


REGISTRY_FILENAME = "mode_registry.json"
SCHEMA_REGISTRY_VERSION = 4


class ModeParamFieldSpec(BaseModel):
    model_config = ConfigDict(extra="forbid")

    name: str
    annotation: dict[str, Any]
    required: bool
    default_kind: Literal["missing", "none", "value", "nan"] = "missing"
    default: Any | None = None


class ModeRegistryEntry(BaseModel):
    model_config = ConfigDict(extra="forbid")

    mode_id: str
    class_path: str
    module_path: str
    class_name: str
    display_name: str
    description: str
    mission_target: str
    required_vision_nodes: tuple[str, ...]
    peer_vehicle_names: tuple[str, ...] = ()
    requires_camera: bool = False
    transition_labels: tuple[str, ...]
    params_schema: dict[str, Any]
    param_fields: tuple[ModeParamFieldSpec, ...] = ()


class ModeRegistryDocument(BaseModel):
    model_config = ConfigDict(extra="forbid")

    schema_version: int = SCHEMA_REGISTRY_VERSION
    modes: dict[str, ModeRegistryEntry]


def registry_path() -> Path:
    return Path(__file__).with_name(REGISTRY_FILENAME)


def _typed_dict_name(name_hint: str) -> str:
    return "".join(part.capitalize() for part in name_hint.replace(".", "_").split("_"))


def _restore_default(annotation: dict[str, Any], value: Any) -> Any:
    kind = annotation["kind"]
    if value is None:
        return None
    if kind == "tuple":
        items = annotation.get("items", [])
        if annotation.get("variadic"):
            item_spec = items[0]
            return tuple(_restore_default(item_spec, item) for item in value)
        return tuple(
            _restore_default(item_spec, item)
            for item_spec, item in zip(items, value, strict=False)
        )
    if kind == "list":
        return [_restore_default(annotation["item"], item) for item in value]
    if kind == "dict":
        return {
            key: _restore_default(annotation["value"], item)
            for key, item in value.items()
        }
    if kind == "typed_dict":
        fields = {
            field["name"]: field["annotation"] for field in annotation.get("fields", [])
        }
        return {
            key: _restore_default(fields.get(key, {"kind": "str"}), item)
            for key, item in value.items()
        }
    if kind == "union":
        options = annotation.get("options", [])
        for option in options:
            if option["kind"] == "none" and value is None:
                return None
            if option["kind"] == "none":
                continue
            return _restore_default(option, value)
    return value


def _annotation_from_spec(annotation: dict[str, Any], *, name_hint: str) -> object:
    kind = annotation["kind"]
    if kind == "bool":
        return bool
    if kind == "int":
        return int
    if kind == "float":
        return float
    if kind == "str":
        return str
    if kind == "none":
        return type(None)
    if kind == "literal":
        choices = tuple(annotation.get("choices", ()))
        return cast(Any, Literal).__getitem__(choices)
    if kind == "list":
        return list[
            _annotation_from_spec(annotation["item"], name_hint=f"{name_hint}_item")
        ]
    if kind == "tuple":
        items = tuple(
            _annotation_from_spec(item, name_hint=f"{name_hint}_{index}")
            for index, item in enumerate(annotation.get("items", ()))
        )
        if annotation.get("variadic"):
            return tuple[(items[0], ...)]
        return tuple[items]
    if kind == "dict":
        key_type = _annotation_from_spec(
            annotation["key"], name_hint=f"{name_hint}_key"
        )
        value_type = _annotation_from_spec(
            annotation["value"], name_hint=f"{name_hint}_value"
        )
        return dict[key_type, value_type]
    if kind == "typed_dict":
        fields = {}
        for field in annotation.get("fields", ()):
            field_type = _annotation_from_spec(
                field["annotation"], name_hint=f"{name_hint}_{field['name']}"
            )
            if field.get("required", True):
                fields[field["name"]] = Required[field_type]
            else:
                fields[field["name"]] = NotRequired[field_type]
        return cast(Any, TypedDict)(_typed_dict_name(name_hint), fields)
    if kind == "union":
        options = [
            _annotation_from_spec(option, name_hint=f"{name_hint}_{index}")
            for index, option in enumerate(annotation.get("options", ()))
        ]
        if not options:
            raise TypeError(f"Union annotation for '{name_hint}' must define options.")
        return reduce(operator.or_, options)

    raise TypeError(f"Unsupported annotation kind '{kind}' for '{name_hint}'.")


@lru_cache
def load_mode_registry_document() -> ModeRegistryDocument:
    path = registry_path()
    if not path.exists():
        raise RuntimeError(
            f"Committed mode schema registry is missing at '{path}'. "
            "Regenerate it with `python3 -m uav.runtime.schema_generator`."
        )
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    document = ModeRegistryDocument.model_validate(payload)
    if document.schema_version != SCHEMA_REGISTRY_VERSION:
        raise RuntimeError(
            f"Unsupported mode schema registry version '{document.schema_version}'."
        )
    return document


def dump_mode_registry_document(
    document: ModeRegistryDocument, *, path: Path | None = None
) -> Path:
    output_path = path or registry_path()
    output_path.write_text(
        json.dumps(
            document.model_dump(mode="json"),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    return output_path


@lru_cache
def params_model_for_entry(mode_id: str) -> type[BaseModel]:
    entry = load_mode_registry_document().modes.get(mode_id)
    if entry is None:
        raise ValueError(
            f"Mode '{mode_id}' is not present in the committed schema registry."
        )

    field_definitions: dict[str, tuple[object, object]] = {}
    for field in entry.param_fields:
        annotation = _annotation_from_spec(
            field.annotation,
            name_hint=f"{entry.class_name}_{field.name}",
        )
        default = ...
        if field.default_kind == "none":
            default = None
        elif field.default_kind == "value":
            default = _restore_default(field.annotation, field.default)
        elif field.default_kind == "nan":
            default = float("nan")
        field_definitions[field.name] = (annotation, default)

    return create_model(
        f"{entry.class_name}Params",
        __config__=ConfigDict(extra="forbid"),
        __module__=__name__,
        **cast(Any, field_definitions),
    )
