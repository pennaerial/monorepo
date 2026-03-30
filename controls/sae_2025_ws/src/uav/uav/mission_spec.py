from __future__ import annotations

from dataclasses import dataclass, field
import importlib
from pathlib import Path
from typing import Any

import yaml

VALID_MISSION_TARGETS = {"uav", "payload"}
MISSION_ROOT = Path(__file__).resolve().parent / "missions"
_TOP_LEVEL_KEYS = {"modes"}
_MODE_KEYS = {"class", "params", "transitions"}


def mission_path_for_name(mission_name: str) -> str:
    return str(MISSION_ROOT / f"{mission_name}.yaml")


def load_mode_class(class_path: str):
    class_name = class_path.rsplit(".", 1)[-1]
    try:
        module = importlib.import_module(class_path)
    except ModuleNotFoundError as exc:
        if exc.name != class_path:
            raise
        module = importlib.import_module(class_path.rsplit(".", 1)[0])

    mode_class = getattr(module, class_name)
    from uav.autonomous_modes.Mode import Mode

    if not isinstance(mode_class, type) or not issubclass(mode_class, Mode):
        raise TypeError(
            f"Mode path '{class_path}' did not resolve to a Mode subclass."
        )
    return mode_class


@dataclass(frozen=True)
class ModeSpec:
    class_path: str
    params: dict[str, Any] = field(default_factory=dict)
    transitions: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class MissionSpec:
    target: str
    modes: dict[str, ModeSpec]
    vision_nodes: tuple[str, ...] = ()
    path: Path | None = None

    @property
    def is_uav(self) -> bool:
        return self.target == "uav"

    @property
    def is_payload(self) -> bool:
        return self.target == "payload"

    @classmethod
    def load(cls, path: str | Path) -> "MissionSpec":
        return load_mission_spec(path)

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "MissionSpec":
        return load_mission_spec(raw)


def load_mission_spec(path: str | Path | dict[str, Any]) -> MissionSpec:
    mission_path: Path | None = None
    if isinstance(path, dict):
        raw = path
    else:
        mission_path = Path(path)
        with mission_path.open("r", encoding="utf-8") as mission_file:
            raw = yaml.safe_load(mission_file) or {}

    mission_label = str(mission_path) if mission_path is not None else "<mission>"

    if not isinstance(raw, dict):
        raise ValueError(
            f"Mission file '{mission_label}' must define a mapping at the top level."
        )

    unknown_top_level = set(raw) - _TOP_LEVEL_KEYS
    if unknown_top_level:
        raise ValueError(
            f"Mission file '{mission_label}' contains unsupported top-level keys: {sorted(unknown_top_level)}"
        )

    raw_modes = raw.get("modes")
    if not isinstance(raw_modes, dict) or not raw_modes:
        raise ValueError(
            f"Mission file '{mission_label}' must define a non-empty modes mapping."
        )
    if "start" not in raw_modes:
        raise ValueError(f"Mission file '{mission_label}' must define a start mode.")

    modes: dict[str, ModeSpec] = {}
    mode_targets: set[str] = set()
    vision_nodes: set[str] = set()

    for mode_name, mode_info in raw_modes.items():
        if not isinstance(mode_name, str) or not mode_name:
            raise ValueError(
                f"Mission file '{mission_label}' contains an invalid mode name: {mode_name!r}"
            )
        if not isinstance(mode_info, dict):
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' must be a mapping."
            )

        unknown_mode_keys = set(mode_info) - _MODE_KEYS
        if unknown_mode_keys:
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' contains unsupported keys: {sorted(unknown_mode_keys)}"
            )

        class_path = mode_info.get("class")
        if not isinstance(class_path, str) or not class_path:
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' must define a non-empty class path."
            )

        params = mode_info.get("params", {})
        if not isinstance(params, dict):
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' must define params as a mapping."
            )

        transitions = mode_info.get("transitions", {})
        if not isinstance(transitions, dict) or not all(
            isinstance(state, str) and isinstance(next_mode, str)
            for state, next_mode in transitions.items()
        ):
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' must define transitions as a string-to-string mapping."
            )

        mode_class = load_mode_class(class_path)
        mission_target = getattr(mode_class, "mission_target", None)
        if mission_target not in VALID_MISSION_TARGETS:
            raise ValueError(
                f"Mode '{class_path}' must declare mission_target as one of {sorted(VALID_MISSION_TARGETS)}."
            )
        mode_targets.add(mission_target)
        vision_nodes.update(mode_class.required_vision_node_names())

        modes[mode_name] = ModeSpec(
            class_path=class_path,
            params=dict(params),
            transitions=dict(transitions),
        )

    if len(mode_targets) != 1:
        raise ValueError(
            f"Mission file '{mission_label}' mixes incompatible mode targets: {sorted(mode_targets)}."
        )

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

    return MissionSpec(
        target=next(iter(mode_targets)),
        modes=modes,
        vision_nodes=tuple(sorted(vision_nodes)),
        path=mission_path,
    )
