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
        payload_0 = Entity(
            name="payload_0",
            path_to_sdf="~/.simulation-gazebo/models/payload/model.sdf",
            position=(-1.0, 0, 0.5),
            rpy=(0.0, 0.0, 0.0),
            world=self.world_name,
        )
        ef0 = payload_0.to_entity_factory_msg()
        self.get_logger().info(
            "DEBUG | CustomWorldNode spawning payload_0 at (0, 0, 0.5), relative_to='%s'"
            % (getattr(ef0, "relative_to", "?") or "?")
        )
        req = SpawnEntity.Request()
        req.entity_factory = ef0
        self.spawn_entity_client.call_async(req)

        payload_1 = Entity(
            name="payload_1",
            path_to_sdf="~/.simulation-gazebo/models/payload/model.sdf",
            position=(0.8, 0.8, 0.5),
            rpy=(0.0, 0.0, 0.0),
            world=self.world_name,
        )
        ef1 = payload_1.to_entity_factory_msg()
        self.get_logger().info(
            "DEBUG | CustomWorldNode spawning payload_1 at (0.8, 0.8, 0.5), relative_to='%s'"
            % (getattr(ef1, "relative_to", "?") or "?")
        )
        req1 = SpawnEntity.Request()
        req1.entity_factory = ef1
        self.spawn_entity_client.call_async(req1)

        return super().generate_world()


def main(args=None):
    rclpy.init(args=args)
    node = CustomWorldNode(**json.loads(sys.argv[1]))
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    rclpy.shutdown()
