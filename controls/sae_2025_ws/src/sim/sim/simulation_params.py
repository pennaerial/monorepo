from __future__ import annotations

import yaml
from ament_index_python import get_package_share_path
from pydantic import BaseModel, Field, model_validator

from sim.world_gen.entity import Entity


class Physics(BaseModel):
    friction: float

class WorldParams(BaseModel):
    node: str
    name: str
    physics: Physics | None = None

    # What are the differences between controllables and entities sim wise?
    controllables: list[Entity] = Field(default_factory=list)
    entities: list[Entity] = Field(default_factory=list)

    @model_validator(mode="before")
    @classmethod
    def inject_world(cls, data: dict) -> dict:
        world_name = data["name"]

        if not world_name:
            return data

        for field in ("controllables", "entities"):
            for entity in data.get(field, []):
                entity["world"] = world_name

        return data

class ScoringParams(BaseModel):
    node: str

    # TODO: Finish later


class SimulationParams(BaseModel):
    world: WorldParams
    scoring: ScoringParams | None = None


    @classmethod
    def load_from_stage(cls, world: str, stage: str = "base") -> SimulationParams:
        simulation_file = get_package_share_path("sim") / "simulations" / world / f"{stage}.yaml"
        with open(simulation_file) as f:
            data = yaml.safe_load(f)
            return SimulationParams.model_validate(data)

