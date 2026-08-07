from sim.world_gen import WorldNode
from sim.world_gen.entity import Entity
import json
import sys
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException


class CustomWorldNode(WorldNode):
    def __init__(
        self,
        template_world: str,
        physics: Optional[dict] = None,
        output_filename: Optional[str] = None,
        seed: Optional[int] = None,
        entities: Optional[dict] = None,
        controllables: Optional[dict] = None,
        **kwargs,
    ):
        super().__init__(
            competition_name="custom",
            output_filename=output_filename,
            seed=seed,
            entities=entities,
            controllables=controllables,
            **kwargs,
        )
        self.world_name = template_world
        # defaults to 0.6 if not provided
        self.physics = physics
        self.instantiate_static_world(template_world_path=template_world, physics=physics)

    def generate_world(self):
        success = True
        if self.entities or self.controllables:
            success = self.spawn_entities(self.entities, label="static entity") and success
            success = self.spawn_entities(self.controllables, label="controllable") and success
        else:
            payload_0 = Entity(
                name="payload_0",
                path_to_sdf="~/.simulation-gazebo/models/payload/model.sdf",
                position=(0, 0, 0.5),
                rpy=(0.0, 0.0, 0.0),
                world=self.world_name,
            )
            success = self.spawn_entity_object(payload_0) and success

            # payload_1 = Entity(
            #     name="payload_1",
            #     path_to_sdf="~/.simulation-gazebo/models/payload/model.sdf",
            #     position=(0.8, 0.8, 0.5),
            #     rpy=(0.0, 0.0, 0.0),
            #     world=self.world_name,
            # )
            # req1 = SpawnEntity.Request()
            # req1.entity_factory = payload_1.to_entity_factory_msg()
            # self.spawn_entity_client.call_async(req1)

        return super().generate_world() and success


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CustomWorldNode(**json.loads(sys.argv[1]))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        print(e)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
