import rclpy
from rclpy.executors import ExternalShutdownException

from sim.world_gen.world_node import WorldNode
from sim.world_gen.entity import Entity


class CustomWorldNode(WorldNode):
    def __init__(self):
        super().__init__("custom_world_node")
        self.entities = self.sim_params.world.entities
        self.controllables = self.sim_params.world.controllables

    def generate_world(self):
        success = True
        if self.entities or self.controllables:
            success = self.spawn_entities(self.entities) and success
            success = self.spawn_entities(self.controllables) and success
        else:
            payload_0 = Entity(
                name="payload_0",
                model="payload",
                position=(0, 0, 0.5),
                rpy=(0.0, 0.0, 0.0),
                world=self.world,
            )
            success = self.spawn_entity(payload_0) and success
        return success


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CustomWorldNode()
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
