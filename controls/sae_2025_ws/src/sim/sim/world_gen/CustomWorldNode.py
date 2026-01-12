from sim.world_gen import WorldNode
from typing import Optional
import rclpy
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
        return super().generate_world()


def main(args=None):
    rclpy.init(args=args)
    node = CustomWorldNode(**json.loads(sys.argv[1]))
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
    rclpy.shutdown()

