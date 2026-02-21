from sim.world_gen import WorldNode
from sim.world_gen.entity import Entity
from typing import Optional
import rclpy
from ros_gz_interfaces.srv import SpawnEntity
import sys
import json
import random
import math


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

        # Spawn payload_0 and aircraft in a subset of DLZ
        # DLZ covers X: 3.8-6.728, Y: -1.728-1.2 (side length ~2.928m)
        # Restrict spawning to center square with side = half DLZ side (~1.464m)
        # DLZ center: (5.264, -0.264), half side = 0.732m
        x_min, x_max = 4.53, 6.0
        y_min, y_max = -1.0, 0.47

        random_x = random.uniform(x_min, x_max)
        random_y = random.uniform(y_min, y_max)
        random_yaw = random.uniform(-math.pi, math.pi)  # Random orientation

        payload_0 = Entity(
            name="payload_0",
            path_to_sdf="~/.simulation-gazebo/models/payload/model.sdf",
            position=(random_x, random_y, 0.5),
            rpy=(0.0, 0.0, random_yaw),
            world=self.world_name,
        )
        req_payload_0 = SpawnEntity.Request()
        req_payload_0.entity_factory = payload_0.to_entity_factory_msg()
        self.spawn_entity_client.call_async(req_payload_0)

        self.get_logger().info(f"Spawned payload_0 at position=({random_x:.2f}, {random_y:.2f}, 0.5), yaw={math.degrees(random_yaw):.1f}°")

        # Spawn VTOL aircraft 0.2m behind payload
        aircraft_distance = 0.2  # meters behind payload
        aircraft_x = random_x - aircraft_distance * math.cos(random_yaw)
        aircraft_y = random_y - aircraft_distance * math.sin(random_yaw)
        aircraft_yaw = random_yaw  # Face same direction as payload

        vtol_aircraft = Entity(
            name="standard_vtol_0",
            path_to_sdf="model://standard_vtol",
            position=(aircraft_x, aircraft_y, 0.5),
            rpy=(0.0, 0.0, aircraft_yaw),
            world=self.world_name,
        )
        req_vtol = SpawnEntity.Request()
        req_vtol.entity_factory = vtol_aircraft.to_entity_factory_msg()
        self.spawn_entity_client.call_async(req_vtol)

        self.get_logger().info(f"Spawned standard_vtol_0 0.2m behind payload at ({aircraft_x:.2f}, {aircraft_y:.2f}, 0.5), yaw={math.degrees(aircraft_yaw):.1f}°")

        # Spawn payload_1 at edge of DLZ, facing center (toward -X)
        # payload_1 = Entity(
        #     name="payload_1",
        #     path_to_sdf="~/.simulation-gazebo/models/payload/model.sdf",
        #     position=(6.0, -0.02, 0.5),
        #     rpy=(0.0, 0.0, 3.14159),  # Face -X direction (toward center)
        #     world=self.world_name,
        # )
        # req_payload_1 = SpawnEntity.Request()
        # req_payload_1.entity_factory = payload_1.to_entity_factory_msg()
        # self.spawn_entity_client.call_async(req_payload_1)

        return super().generate_world()


def main(args=None):
    rclpy.init(args=args)
    node = CustomWorldNode(**json.loads(sys.argv[1]))
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    rclpy.shutdown()
