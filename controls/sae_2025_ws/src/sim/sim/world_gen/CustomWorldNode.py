from sim.world_gen import WorldNode
from sim.world_gen.entity import Entity
from typing import Optional
import random
import math
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

    def _random_payload_position(self, max_radius=3.0, min_radius=0.5):
        """Random position within max_radius of the VTOL (origin), x < 0."""
        while True:
            r = random.uniform(min_radius, max_radius)
            theta = random.uniform(-math.pi / 2, math.pi / 2)
            x = -abs(r * math.cos(theta))
            y = r * math.sin(theta)
            if abs(x) >= min_radius:
                return (0, -3, 0.5)

    def generate_world(self):
        dlz = Entity(
            name="dlz_pink",
            path_to_sdf="~/.simulation-gazebo/models/dlz/model.sdf",
            position=(-3, 3, -0.005),
            rpy=(0.0, 0.0, 0.0),
            world=self.world_name,
        )
        req_dlz = SpawnEntity.Request()
        req_dlz.entity_factory = dlz.to_entity_factory_msg()
        self.spawn_entity_client.call_async(req_dlz)
        self.get_logger().info("CustomWorldNode spawning dlz_pink at (0, 0, 0.01)")

#        pos0 = self._random_payload_position()
        #yaw0 = random.uniform(-math.pi, math.pi)
        # payload_0 on opposite side from back tag: 2.5 m in front of VTOL (VTOL at origin, back tag on -x)
        pos0 = (-2.5, 0.0, 0.5)
        yaw0 = math.pi  # face VTOL (back tag behind us)
        payload_0 = Entity(
            name="payload_0",
            path_to_sdf="~/.simulation-gazebo/models/payload/model.sdf",
            position=pos0,
            rpy=(0.0, 0.0, yaw0),
            world=self.world_name,
        )
        ef0 = payload_0.to_entity_factory_msg()
        self.get_logger().info(
            "CustomWorldNode spawning payload_0 at (%.2f, %.2f, %.2f) yaw=%.1f deg (2.5 m in front of VTOL)"
            % (pos0[0], pos0[1], pos0[2], math.degrees(yaw0))
        )
        req = SpawnEntity.Request()
        req.entity_factory = ef0
        self.spawn_entity_client.call_async(req)

        payload_1 = Entity(
            name="payload_1",
            path_to_sdf="~/.simulation-gazebo/models/payload/model.sdf",
            position=(2, 2, 0.5),
            rpy=(0.0, 0.0, 0.0),
            world=self.world_name,
        )
        ef1 = payload_1.to_entity_factory_msg()
        self.get_logger().info(
            "CustomWorldNode spawning payload_1 at (2, 2, 0.5)"
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
