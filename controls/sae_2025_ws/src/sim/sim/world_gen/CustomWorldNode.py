from sim.world_gen import WorldNode
from sim.world_gen.entity import Entity
from typing import Optional
import rclpy
from ros_gz_interfaces.srv import SpawnEntity
import sys
import json


class CustomWorldNode(WorldNode):
    def __init__(
        self,
        template_world: str,
        physics: Optional[dict] = None,
        output_filename: Optional[str] = None,
        seed: Optional[int] = None,
    ):
        super().__init__(
            competition_name="custom", output_filename=output_filename, seed=seed
        )
        self.world_name = template_world
        # defaults to 0.6 if not provided
        self.physics = physics
        self.instantiate_static_world(
            template_world_path=template_world, physics=physics
        )

    def generate_world(self):
        # Spawn pink DLZ at origin
        dlz = Entity(
            name="dlz_pink",
            path_to_sdf="~/.simulation-gazebo/models/dlz/model.sdf",
            position=(0, 0, 0),
            rpy=(0.0, 0.0, 0.0),
            world=self.world_name,
        )
        req_dlz = SpawnEntity.Request()
        req_dlz.entity_factory = dlz.to_entity_factory_msg()
        self.spawn_entity_client.call_async(req_dlz)

        # Spawn payload in center of DLZ (DLZ covers X: 3.8-6.24, Y: -1.24-1.2)
        payload_0 = Entity(
            name="payload_0",
            path_to_sdf="~/.simulation-gazebo/models/payload/model.sdf",
            position=(5.02, -0.02, 0.5),
            rpy=(0.0, 0.0, 0.0),
            world=self.world_name,
        )
        req = SpawnEntity.Request()
        req.entity_factory = payload_0.to_entity_factory_msg()
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
