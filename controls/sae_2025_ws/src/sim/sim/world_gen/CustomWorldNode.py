from sim.world_gen import WorldNode
from sim.world_gen.Entity import Entity
from typing import Optional
import rclpy
from std_srvs.srv import Trigger
from ros_gz_interfaces.srv import SpawnEntity
import sys
import json


class CustomWorldNode(WorldNode):

    def __init__(self, 
                 world_name: str,
                 output_filename: Optional[str] = None, seed: Optional[int] = None):
        super().__init__(competition_name="custom", output_filename=output_filename, seed=seed)
        self.world_name = world_name
        self.instantiate_static_world(template_world_path=world_name)
    
    def generate_world(self):
        hoop = Entity(name="hoop0", 
                      path_to_sdf="~/.simulation-gazebo/models/hoop/model.sdf",
                      position=(0, 0, 0),
                      rpy = (0, 0, 0))
        req = SpawnEntity.Request()
        req.entity_factory = hoop.to_entity_factory_msg()
        self.spawn_entity_client.call_async(req)
        return super().generate_world()


def main(args=None):
    rclpy.init(args=args)
    node = CustomWorldNode(**json.loads(sys.argv[1]))
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
    rclpy.shutdown()

