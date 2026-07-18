from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
from typing import Any

from pydantic import BaseModel, ConfigDict, Field


VALID_MISSION_TARGETS = {"uav", "payload"}


class MissionModeDocumentModel(BaseModel):
    model_config = ConfigDict(extra="forbid", populate_by_name=True)

    mode_id: str = Field(alias="mode")
    params: dict[str, Any] = Field(default_factory=dict)
    transitions: dict[str, str] = Field(default_factory=dict)


class MissionDocumentModel(BaseModel):
    model_config = ConfigDict(extra="forbid")

    modes: dict[str, MissionModeDocumentModel]


def _mission_roots() -> list[Path]:
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_share_directory,
    )

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


def get_mission_path(mission_name: str, package: str) -> str:
    # Lazy import: ament_index_python ships only with ROS, so importing it at
    # module top level would force ROS onto pure-Python consumers
    from ament_index_python.packages import (
        get_package_share_directory,
    )

    # gets path in package's install directory where missions are installed to
    mission_path = (
        Path(get_package_share_directory(package)) / "missions" / f"{mission_name}.yaml"
    )
    if not os.path.isfile(mission_path):
        raise FileNotFoundError(
            f"Mission {mission_name} was not found at {mission_path}"
        )

    return str(mission_path)


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
