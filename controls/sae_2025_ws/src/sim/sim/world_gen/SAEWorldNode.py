from sim.world_gen import WorldNode
from sim.world_gen.entity import Entity
from typing import Optional, List
import rclpy
from ros_gz_interfaces.srv import SpawnEntity
import sys
import json


class SAEWorldNode(WorldNode):
    """
    World generation node for SAE competition.
    Generates the environment for the SAE competition.
    """

    def __init__(
        self,
        template_world: str,
        physics: Optional[dict] = None,
        output_filename: Optional[str] = None,
        seed: Optional[int] = None,
        entities: Optional[List[dict]] = None,
    ):
        super().__init__(
            competition_name="sae", output_filename=output_filename, seed=seed
        )
        self.world_name = template_world
        # defaults to 0.6 if not provided
        self.physics = physics
        self.entity_configs = entities or []
        self.instantiate_static_world(
            template_world_path=template_world, physics=physics
        )

    def generate_world(self):

        for cfg in self.entity_configs:
            entity = Entity(
                name=cfg["name"],
                path_to_sdf=cfg["path_to_sdf"],
                position=tuple(cfg["position"]),
                rpy=tuple(cfg["rpy"]),
                world=self.competition_name,
            )
            req = SpawnEntity.Request()
            req.entity_factory = entity.to_entity_factory_msg()
            self.spawn_entity_client.call_async(req)

        return super().generate_world()


def main(args=None):
    rclpy.init(args=args)
    node = SAEWorldNode(**json.loads(sys.argv[1]))
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    rclpy.shutdown()
