from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
import json
from pathlib import Path
from typing import Any

import yaml

_PUBLIC_MODE_PREFIX = "uav.modes."
_MODE_REGISTRY_PATH = (
    Path(__file__).resolve().parents[2]
    / "uav"
    / "uav"
    / "runtime"
    / "mode_registry.json"
)


@dataclass(frozen=True)
class ModeRegistryEntryCompat:
    mode_id: str
    class_path: str
    module_path: str
    class_name: str
    display_name: str
    description: str
    mission_target: str
    required_vision_nodes: tuple[str, ...]
    transition_labels: tuple[str, ...]
    params_schema: dict[str, Any] = field(default_factory=dict)
    peer_vehicle_names: tuple[str, ...] = ()
    requires_camera: bool = False


@dataclass(frozen=True)
class ModeSpecCompat:
    mode_id: str
    class_path: str
    params: dict[str, Any] = field(default_factory=dict)
    transitions: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class MissionSpecCompat:
    target: str
    modes: dict[str, ModeSpecCompat]
    vision_nodes: tuple[str, ...] = ()
    peer_vehicle_names: tuple[str, ...] = ()
    requires_camera: bool = False
    path: Path | None = None


def public_mode_id(mode_ref: str | None) -> str:
    mode = (mode_ref or "").strip()
    if mode.startswith(_PUBLIC_MODE_PREFIX):
        mode = mode[len(_PUBLIC_MODE_PREFIX) :]
    parts = mode.split(".")
    if len(parts) > 1 and parts[-1] == parts[-2]:
        mode = ".".join(parts[:-1])
    return mode


def internal_mode_path(mode_ref: str | None) -> str:
    mode = public_mode_id(mode_ref)
    if mode.startswith(("uav.", "payload.")):
        module_path = f"{_PUBLIC_MODE_PREFIX}{mode}"
        class_name = mode.rsplit(".", 1)[-1]
        if module_path.rsplit(".", 1)[-1] == class_name:
            return module_path
        return f"{module_path}.{class_name}"
    return mode


@lru_cache(maxsize=1)
def _load_raw_mode_registry() -> dict[str, Any]:
    with _MODE_REGISTRY_PATH.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _mode_entry_from_payload(
    key: str, payload: dict[str, Any]
) -> ModeRegistryEntryCompat:
    class_path = str(payload.get("class_path") or key).strip()
    mode_id = public_mode_id(str(payload.get("mode_id") or class_path))
    module_path = str(
        payload.get("module_path") or class_path.rsplit(".", 1)[0]
    ).strip()
    class_name = str(payload.get("class_name") or class_path.rsplit(".", 1)[-1]).strip()
    return ModeRegistryEntryCompat(
        mode_id=mode_id,
        class_path=class_path,
        module_path=module_path,
        class_name=class_name,
        display_name=str(payload.get("display_name") or class_name).strip(),
        description=str(payload.get("description") or "").strip(),
        mission_target=str(payload.get("mission_target") or "").strip(),
        required_vision_nodes=tuple(payload.get("required_vision_nodes") or ()),
        transition_labels=tuple(payload.get("transition_labels") or ()),
        params_schema=dict(payload.get("params_schema") or {}),
        peer_vehicle_names=tuple(payload.get("peer_vehicle_names") or ()),
        requires_camera=bool(payload.get("requires_camera", False)),
    )


@lru_cache(maxsize=1)
def load_mode_registry_entries() -> tuple[ModeRegistryEntryCompat, ...]:
    payload = _load_raw_mode_registry()
    modes = payload.get("modes", {})
    if not isinstance(modes, dict):
        raise ValueError("Committed mode registry must define a modes mapping.")
    return tuple(
        _mode_entry_from_payload(key, value)
        for key, value in modes.items()
        if isinstance(value, dict)
    )


def mode_registry_entries(
    mission_target: str | None = None,
) -> list[ModeRegistryEntryCompat]:
    entries = list(load_mode_registry_entries())
    if mission_target:
        entries = [entry for entry in entries if entry.mission_target == mission_target]
    return entries


def mode_entry_for_class_path(mode_ref: str) -> ModeRegistryEntryCompat:
    mode = public_mode_id(mode_ref)
    candidates = {
        mode_ref.strip(),
        mode,
        internal_mode_path(mode_ref),
        internal_mode_path(mode),
    }
    for entry in load_mode_registry_entries():
        if entry.class_path in candidates or entry.mode_id in candidates:
            return entry
    raise ValueError(
        f"Mode '{mode_ref}' is not present in the committed schema registry."
    )


def _mode_spec_from_entry(
    entry: ModeRegistryEntryCompat, mode_info: dict[str, Any]
) -> ModeSpecCompat:
    params = mode_info.get("params", {})
    if not isinstance(params, dict):
        raise ValueError("Mode params must be a mapping.")
    transitions = mode_info.get("transitions", {})
    if not isinstance(transitions, dict) or not all(
        isinstance(state, str) and isinstance(next_mode, str)
        for state, next_mode in transitions.items()
    ):
        raise ValueError("Mode transitions must be a string-to-string mapping.")
    return ModeSpecCompat(
        mode_id=entry.mode_id,
        class_path=entry.class_path,
        params=dict(params),
        transitions=dict(transitions),
    )


def load_mission_spec_compat(path: str | Path | dict[str, Any]) -> MissionSpecCompat:
    mission_path: Path | None = None
    if isinstance(path, dict):
        raw = path
        mission_label = "<mission>"
    else:
        mission_path = Path(path)
        with mission_path.open("r", encoding="utf-8") as mission_file:
            raw = yaml.safe_load(mission_file) or {}
        mission_label = str(mission_path)

    if not isinstance(raw, dict):
        raise ValueError(
            f"Mission file '{mission_label}' must define a mapping at the top level."
        )

    raw_modes = raw.get("modes")
    if not isinstance(raw_modes, dict) or not raw_modes:
        raise ValueError(
            f"Mission file '{mission_label}' must define a non-empty modes mapping."
        )
    if "start" not in raw_modes:
        raise ValueError(f"Mission file '{mission_label}' must define a start mode.")

    modes: dict[str, ModeSpecCompat] = {}
    target: str | None = None
    vision_nodes: set[str] = set()
    peer_vehicle_names: set[str] = set()
    requires_camera = False

    for mode_name, mode_info in raw_modes.items():
        if not isinstance(mode_name, str) or not mode_name:
            raise ValueError(
                f"Mission file '{mission_label}' contains an invalid mode name: {mode_name!r}"
            )
        if not isinstance(mode_info, dict):
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' must be a mapping."
            )

        unknown_mode_keys = set(mode_info) - {"mode", "class", "params", "transitions"}
        if unknown_mode_keys:
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' contains unsupported keys: {sorted(unknown_mode_keys)}"
            )

        mode_ref = mode_info.get("mode") or mode_info.get("class")
        if not isinstance(mode_ref, str) or not mode_ref.strip():
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' must define mode or class."
            )

        entry = mode_entry_for_class_path(mode_ref)
        if entry.mission_target not in {"uav", "payload"}:
            raise ValueError(
                f"Mode '{mode_ref}' must declare mission_target as one of ['payload', 'uav']."
            )

        if target is None:
            target = entry.mission_target
        elif target != entry.mission_target:
            raise ValueError(
                f"Mission file '{mission_label}' mixes incompatible mode targets: {[target, entry.mission_target]}."
            )

        vision_nodes.update(entry.required_vision_nodes)
        peer_vehicle_names.update(
            str(peer_name).strip()
            for peer_name in entry.peer_vehicle_names
            if str(peer_name).strip()
        )
        requires_camera = (
            requires_camera
            or bool(entry.required_vision_nodes)
            or entry.requires_camera
        )

        modes[mode_name] = _mode_spec_from_entry(entry, mode_info)

    for mode_name, mode_spec in modes.items():
        unknown_targets = sorted(
            next_mode
            for next_mode in mode_spec.transitions.values()
            if next_mode not in modes
        )
        if unknown_targets:
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' transitions to undefined mode(s): {unknown_targets}"
            )

    return MissionSpecCompat(
        target=target or "",
        modes=modes,
        vision_nodes=tuple(sorted(vision_nodes)),
        peer_vehicle_names=tuple(sorted(peer_vehicle_names)),
        requires_camera=requires_camera,
        path=mission_path,
    )
