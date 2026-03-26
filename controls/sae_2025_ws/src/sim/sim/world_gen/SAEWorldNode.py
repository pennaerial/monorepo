from sim.world_gen import WorldNode
from typing import Optional
import rclpy
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
        vehicle_pose: Optional[tuple[float, float, float, float, float, float]] = None,
        physics: Optional[dict] = None,
        output_filename: Optional[str] = None,
        seed: Optional[int] = None,
        dlz: Optional[dict] = None,
        payload_0: Optional[dict] = None,
        payload_1: Optional[dict] = None,
        **kwargs,
    ):
        super().__init__(
            competition_name="sae", output_filename=output_filename, seed=seed, **kwargs
        )
        self.world_name = template_world
        self.vehicle_pose = vehicle_pose
        self.physics = physics
        self.dlz = dlz
        self.payload_0 = payload_0
        self.payload_1 = payload_1
        self.instantiate_static_world(
            template_world_path=template_world, physics=physics
        )

    def generate_world(self):
        named_entities = [
            ("dlz", self.dlz),
            ("payload_0", self.payload_0),
            ("payload_1", self.payload_1),
        ]
        for name, cfg in named_entities:
            if cfg is not None:
                self.get_logger().info(f"Spawning entity: {name}")
                self.spawn_entity(name, cfg)
            else:
                self.get_logger().info(f"Skipping entity (not defined): {name}")

        return super().generate_world()


def main(args=None):
    rclpy.init(args=args)
    node = SAEWorldNode(**json.loads(sys.argv[1]))
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    rclpy.shutdown()
