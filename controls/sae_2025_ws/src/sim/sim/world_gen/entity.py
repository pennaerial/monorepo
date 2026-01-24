from ros_gz_interfaces.msg import EntityFactory
from geometry_msgs.msg import Pose
import os

from tf_transformations import quaternion_from_euler

class Entity:
    """
    Docstring for Entity
    Entity class for dynamically spawning world objects via /world/{competition}/create service
    """
    def __init__(self,
        name: str, 
        path_to_sdf: str, 
        position: tuple[float, float, float],
        rpy: tuple[float, float, float],
        world: str):

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
        return self.position + self.rpy #(x, y, z, r, p, y)
