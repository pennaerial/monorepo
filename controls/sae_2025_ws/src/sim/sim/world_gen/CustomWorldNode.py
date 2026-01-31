from sim.world_gen import WorldNode
from sim.world_gen.entity import Entity
from typing import Optional
import rclpy
from ros_gz_interfaces.srv import SpawnEntity
import sys
import json


class CustomWorldNode(WorldNode):

    def __init__(self, 
                 template_world: str,
                 output_filename: Optional[str] = None, seed: Optional[int] = None):
        super().__init__(competition_name="custom", output_filename=output_filename, seed=seed)
        self.world_name = template_world
        self.instantiate_static_world(template_world_path=template_world)
    
    def generate_world(self):
        dlz = Entity(
            name="dlz",
            path_to_sdf="~/.simulation-gazebo/models/dlz/model.sdf",
            position=(-5.0, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
            world="custom"
        )
        req = SpawnEntity.Request()
        req.entity_factory = dlz.to_entity_factory_msg()
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

