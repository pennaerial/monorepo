from sim.world_gen.world_node import WorldNode

import rclpy
from rclpy.executors import ExternalShutdownException


class SAEWorldNode(WorldNode):
    """
    World generation node for the In House 2026 challenge.
    Generates the environment for the SAE competition.
    """

    def __init__(self):
        super().__init__("sae_world_node")
        self.entities = self.sim_params.world.entities
        self.controllables = self.sim_params.world.controllables

    def generate_world(self):
        success = True
        success = self.spawn_entities(self.entities) and success
        success = self.spawn_entities(self.controllables) and success
        return success


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SAEWorldNode()
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
