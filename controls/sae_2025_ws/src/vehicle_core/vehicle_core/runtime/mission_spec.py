from __future__ import annotations

from dataclasses import dataclass, field
import importlib
from pathlib import Path
from typing import TYPE_CHECKING, Any

import yaml
from pydantic import BaseModel, ConfigDict, Field

from .mode_paths import normalize_public_mode_id
from vehicle_core.mode import Mode

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)


VALID_MISSION_TARGETS = {"uav", "payload"}
_TOP_LEVEL_KEYS = {"modes"}
_MODE_KEYS = {"mode", "params", "transitions"}

if TYPE_CHECKING:
    from uav.modes.Mode import Mode


class MissionModeDocumentModel(BaseModel):
    model_config = ConfigDict(extra="forbid", populate_by_name=True)

    mode_id: str = Field(alias="mode")
    params: dict[str, Any] = Field(default_factory=dict)
    transitions: dict[str, str] = Field(default_factory=dict)


class MissionDocumentModel(BaseModel):
    model_config = ConfigDict(extra="forbid")

    modes: dict[str, MissionModeDocumentModel]


def _mission_roots() -> list[Path]:
    # Installed: check share directories for each package that owns missions.
    roots = []
    for pkg in ("uav", "payload"):
        try:
            roots.append(Path(get_package_share_directory(pkg)) / "missions")
        except PackageNotFoundError:
            pass
    if roots:
        return roots
    # Dev fallback: search relative to the source tree.
    src_root = Path(__file__).resolve().parents[3]
    return [src_root / pkg / "missions" for pkg in ("uav", "payload")]


def mission_root() -> Path:
    """Return the primary (UAV) mission root for backward compatibility."""
    return _mission_roots()[0]


def mission_path_for_name(mission_name: str) -> str:
    for root in _mission_roots():
        candidate = root / f"{mission_name}.yaml"
        if candidate.exists():
            return str(candidate)
    return str(_mission_roots()[0] / f"{mission_name}.yaml")


def load_mode_class(class_path: str) -> type["Mode[Any]"]:
    class_name = class_path.rsplit(".", 1)[-1]
    try:
        module = importlib.import_module(class_path)
    except ModuleNotFoundError as exc:
        if exc.name != class_path:
            raise
        module = importlib.import_module(class_path.rsplit(".", 1)[0])

    mode_class = getattr(module, class_name)

    if not isinstance(mode_class, type) or not issubclass(mode_class, Mode):
        raise TypeError(f"Mode path '{class_path}' did not resolve to a Mode subclass.")
    return mode_class


@dataclass(frozen=True)
class ModeSpec:
    mode_id: str
    params: dict[str, Any] = field(default_factory=dict)
    transitions: dict[str, str] = field(default_factory=dict)

    @property
    def mode(self) -> str:
        return self.mode_id

    @property
    def class_path(self) -> str:
        return self.mode_id


@dataclass(frozen=True)
class MissionSpec:
    target: str
    modes: dict[str, ModeSpec]
    vision_nodes: tuple[str, ...] = ()
    peer_vehicle_names: tuple[str, ...] = ()
    requires_camera: bool = False
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


# TODO: Lowkey this entire file can be moved to a third party validator, like Pydantic
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

        unknown_mode_keys = set(mode_info) - _MODE_KEYS
        if unknown_mode_keys:
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' contains unsupported keys: {sorted(unknown_mode_keys)}"
            )

        if "class" in mode_info and "mode" not in mode_info:
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' uses deprecated key 'class'. Use 'mode' instead."
            )

        mode_id = mode_info.get("mode")
        if not isinstance(mode_id, str) or not mode_id:
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' must define a non-empty mode identifier."
            )
        try:
            mode_id = normalize_public_mode_id(mode_id)
        except ValueError as exc:
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' has an invalid mode identifier: {exc}"
            ) from exc

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

        from .schema import mode_entry_for_mode_id, validate_mode_params

        mode_entry = mode_entry_for_mode_id(mode_id)
        mission_target = mode_entry.mission_target
        if mission_target not in VALID_MISSION_TARGETS:
            raise ValueError(
                f"Mode '{mode_id}' must resolve mission_target as one of {sorted(VALID_MISSION_TARGETS)}."
            )
        mode_targets.add(mission_target)
        vision_nodes.update(mode_entry.required_vision_nodes)
        peer_vehicle_names.update(
            str(peer_name).strip()
            for peer_name in getattr(mode_entry, "peer_vehicle_names", ())
            if str(peer_name).strip()
        )
        requires_camera = requires_camera or bool(mode_entry.required_vision_nodes)
        requires_camera = requires_camera or bool(
            getattr(mode_entry, "requires_camera", False)
        )

        try:
            validated_params = validate_mode_params(mode_id, dict(params))
        except Exception as exc:
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' has invalid params for '{mode_id}': {exc}"
            ) from exc

        unknown_transition_labels = sorted(
            state
            for state in transitions
            if state not in set(mode_entry.transition_labels)
        )
        if unknown_transition_labels:
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' uses unsupported transition label(s) for '{mode_id}': {unknown_transition_labels}"
            )

        modes[mode_name] = ModeSpec(
            mode_id=mode_id,
            params=validated_params,
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
            if next_mode not in modes and next_mode != "terminate"
        )
        if unknown_targets:
            raise ValueError(
                f"Mode '{mode_name}' in '{mission_label}' transitions to undefined mode(s): {unknown_targets}"
            )

    return MissionSpec(
        target=next(iter(mode_targets)),
        modes=modes,
        vision_nodes=tuple(sorted(vision_nodes)),
        peer_vehicle_names=tuple(sorted(peer_vehicle_names)),
        requires_camera=requires_camera,
        path=mission_path,
    )
