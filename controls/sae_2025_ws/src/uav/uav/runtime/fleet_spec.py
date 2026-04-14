from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    ValidationError,
    field_validator,
    model_validator,
)


class FleetDefaultsModel(BaseModel):
    model_config = ConfigDict(extra="forbid")

    auto_launch: bool | None = None
    debug: bool | None = None
    vision_debug: bool | None = None
    force_camera: bool | None = None
    save_vision_milliseconds: int | None = None
    servo_only: bool | None = None
    camera_mount_offsets: tuple[float, float, float] | None = None
    camera_input_transport: str | None = None
    camera_rotate_degrees: float | None = None
    camera_preprocess_hook: str | None = None


class FleetVehicleModel(FleetDefaultsModel):
    model_config = ConfigDict(extra="forbid")

    name: str
    controllable: str | None = None
    mission: str | None = None
    mission_path: str | None = None
    kind: Literal["uav", "payload"] | None = None
    px4_airframe_id: int | None = None
    px4_instance: int | None = None
    payload_controller: str | None = None
    airframe: str | None = None
    custom_airframe_model: str | None = None
    model: str | None = None
    sim: bool | None = None

    @model_validator(mode="after")
    def _require_name(self) -> "FleetVehicleModel":
        if not self.name.strip():
            raise ValueError("Fleet vehicles require a non-empty name.")
        return self


class HardwareBackendModel(BaseModel):
    model_config = ConfigDict(extra="forbid")

    kind: Literal["hardware", "real"]
    px4_path: str | None = "~/PX4-Autopilot"

    @field_validator("kind")
    @classmethod
    def _normalize_kind(cls, value: str) -> str:
        return "hardware" if value == "real" else value


class SimBackendModel(BaseModel):
    model_config = ConfigDict(extra="forbid")

    kind: Literal["sim"]
    px4_path: str | None = "~/PX4-Autopilot"
    world_name: str
    mission_stage: str | None = None
    middleware_port: int | None = None
    world_overrides: dict[str, Any] | None = None

    @model_validator(mode="after")
    def _require_world_name(self) -> "SimBackendModel":
        if not self.world_name.strip():
            raise ValueError("Sim backend requires a non-empty world_name.")
        return self


FleetBackendModel = HardwareBackendModel | SimBackendModel


class FleetDocumentModel(BaseModel):
    model_config = ConfigDict(extra="forbid")

    backend: FleetBackendModel
    defaults: FleetDefaultsModel = Field(default_factory=FleetDefaultsModel)
    vehicles: list[FleetVehicleModel]

    @model_validator(mode="after")
    def _require_vehicles(self) -> "FleetDocumentModel":
        if not self.vehicles:
            raise ValueError("Fleet file requires a non-empty vehicles list.")
        return self


def load_fleet_document(path: str | Path | dict[str, Any]) -> FleetDocumentModel:
    if isinstance(path, dict):
        raw = path
        label = "<fleet>"
    else:
        fleet_path = Path(path)
        label = str(fleet_path)
        with fleet_path.open("r", encoding="utf-8") as fleet_file:
            raw = yaml.safe_load(fleet_file) or {}

    if not isinstance(raw, dict):
        raise ValueError(f"Fleet file '{label}' must define a mapping.")

    try:
        return FleetDocumentModel.model_validate(raw)
    except ValidationError as exc:
        raise ValueError(f"Fleet file '{label}' is invalid: {exc}") from exc
