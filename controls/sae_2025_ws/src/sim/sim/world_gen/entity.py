from ros_gz_interfaces.msg import EntityFactory
from geometry_msgs.msg import Pose
import math
import os


def _quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert roll, pitch, yaw (radians) to quaternion (x, y, z, w). Matches ROS tf convention."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


class Entity:
    """
    Docstring for Entity
    Entity class for dynamically spawning world objects via /world/{competition}/create service
    """

    def __init__(
        self,
        name: str,
        path_to_sdf: str,
        position: tuple[float, float, float],
        rpy: tuple[float, float, float],
        world: str,
    ):
        self.name = name
        self.path_to_sdf = os.path.expanduser(path_to_sdf)
        self.position = position
        self.rpy = rpy
        self.world = world

    def to_entity_factory_msg(self):
        pose = Pose()
        pose.position.x = float(self.position[0])
        pose.position.y = float(self.position[1])
        pose.position.z = float(self.position[2])
        q = _quaternion_from_euler(self.rpy[0], self.rpy[1], self.rpy[2])
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
