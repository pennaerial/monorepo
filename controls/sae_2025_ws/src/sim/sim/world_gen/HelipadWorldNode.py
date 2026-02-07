from sim.world_gen import WorldNode
from sim.world_gen.entity import Entity
from typing import Optional
import rclpy
from ros_gz_interfaces.srv import SpawnEntity
import sys
import json


class HelipadWorldNode(WorldNode):
    """
    World node that spawns a helipad for testing vision-based landing.
    """

    def __init__(self,
                 template_world: str,
                 helipad_position: list = None,
                 output_filename: Optional[str] = None,
                 seed: Optional[int] = None):
        """
        Args:
            template_world: Path to the base world SDF template
            helipad_position: [x, y, z] position for the helipad (default: [5, 5, 0])
            output_filename: Optional output file for generated world
            seed: Random seed
        """
        super().__init__(competition_name="helipad_test", output_filename=output_filename, seed=seed)
        self.world_name = "helipad_test"
        self.helipad_position = helipad_position or [5, 5, 0]
        self.instantiate_static_world(template_world_path=template_world)

    def generate_world(self):
        """Spawn the helipad in the world."""
        self.get_logger().info(f"Spawning helipad at position {self.helipad_position}")

        helipad = Entity(
            name="helipad",
            path_to_sdf="~/.simulation-gazebo/models/helipad/model.sdf",
            position=tuple(self.helipad_position),
            rpy=(0.0, 0.0, 0.0),
            world=self.world_name
        )

        req = SpawnEntity.Request()
        req.entity_factory = helipad.to_entity_factory_msg()
        self.spawn_entity_client.call_async(req)

        self.get_logger().info("Helipad spawned successfully")
        return super().generate_world()


def main(args=None):
    rclpy.init(args=args)
    node = HelipadWorldNode(**json.loads(sys.argv[1]))
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    rclpy.shutdown()
