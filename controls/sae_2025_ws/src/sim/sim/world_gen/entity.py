from __future__ import annotations
from pathlib import Path
import os
from math import cos, sin

from pydantic import BaseModel, model_validator

from ros_gz_interfaces.msg import EntityFactory
from geometry_msgs.msg import Pose


def quaternion_from_euler(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = cos(half_roll)
    sr = sin(half_roll)
    cp = cos(half_pitch)
    sp = sin(half_pitch)
    cy = cos(half_yaw)
    sy = sin(half_yaw)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )



class Entity(BaseModel):
    """
    Entity class for dynamically spawning world objects via /world/{competition}/create service
    """

    name: str
    path_to_sdf: str = ""
    model: str = ""
    position: tuple[float, float, float]
    rpy: tuple[float, float, float]
    world: str

    @model_validator(mode="after")
    def post_validate(self) -> Entity:
        if not self.model and not self.path_to_sdf:
            raise ValueError("Error: must provide either model or path_to_sdf field to entity")

        # if only path_to_sdf is defined, derive model from it
        if not self.model: # /path/to/model/model.sdf --> "model"
            self.model = str(Path(self.path_to_sdf).parent)
            self.validate_path_to_sdf()

        # if only model is defined, or both, path_to_sdf gets overridden by path derived from self.model
        else:
            path = Path(os.environ["PENNAIR_GZ_MODELS_PATH"]) / "models" / self.model / "model.sdf"
            self.path_to_sdf = str(path)
            self.validate_path_to_sdf()

        return self

    # validates path_to_sdf is .sdf file and exists
    def validate_path_to_sdf(self):
        sdf_path = Path(self.path_to_sdf).expanduser()
        if sdf_path.suffix.lower() != ".sdf":
            raise ValueError("path_to_sdf must point to an .sdf file")
        if not sdf_path.is_file():
            raise ValueError(f"SDF file does not exist: {sdf_path}")


    def to_entity_factory_msg(self):
        pose = Pose()
        pose.position.x = float(self.position[0])
        pose.position.y = float(self.position[1])
        pose.position.z = float(self.position[2])
        q = quaternion_from_euler(self.rpy[0], self.rpy[1], self.rpy[2])
        pose.orientation.x = float(q[0])
        pose.orientation.y = float(q[1])
        pose.orientation.z = float(q[2])
        pose.orientation.w = float(q[3])

        ent_fact = EntityFactory()
        ent_fact.name = self.name
        ent_fact.sdf_filename = self.path_to_sdf
        ent_fact.pose = pose
        ent_fact.relative_to = self.world
        return ent_fact

    def to_pose(self):
        return self.position + self.rpy  # (x, y, z, r, p, y)
