from abc import ABC, abstractmethod
import xml.etree.ElementTree as ET
import random

from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from std_srvs.srv import Trigger

from sim.world_gen.entity import Entity
from sim.simulation_params import SimulationParams


class WorldNode(Node, ABC):
    """Abstract base class for world generation nodes.

    Each competition should have a WorldNode implementation that generates
    generates each world at runtime by spawning in the correct entities.

    Should implement any topics/services for world data
    """

    def __init__(self, node_name: str):
        """
        Initialize the WorldNode.

        Args:
            competition_name: Name of the competition (e.g., 'in_house')
            output_filename: Optional custom output filename. If None, uses {competition_name}.sdf
            seed: Optional random seed for reproducible world generation. If None, uses unseeded RNG.
        """
        super().__init__(node_name)
        self.declare_parameter("world", "")
        self.declare_parameter("stage", "base")

        self.world: str = self.get_parameter("world").get_parameter_value().string_value
        self.stage: str = self.get_parameter("stage").get_parameter_value().string_value
        self.sim_params = SimulationParams.load_from_stage(self.world, self.stage)

        # Initialize local RNG instance (isolated from global random state)
        self.rng = random.Random()
        # self.rng = random.Random(seed)
        # self.get_logger().info(
        #     f"Initialized world RNG with seed: {seed if seed is not None else 'unseeded'}"
        # )

        self.trigger_srv = self.create_service(
            Trigger, f"/{self.get_name()}/generate_world", self.generate_world_callback
        )
        self.spawn_entity_client = self.create_client(SpawnEntity, f"/world/{self.world}/create")
        self.get_logger().info(f"Initialized world node: {self.get_name()}")

    def spawn_entity(self, entity: Entity) -> bool:
        if not self.spawn_entity_client.service_is_ready():
            self.get_logger().error(
                f"Cannot spawn entity '{entity.name}': /world/{self.world}/create is not ready"
            )
            return False

        req = SpawnEntity.Request()
        req.entity_factory = entity.to_entity_factory_msg()
        future = self.spawn_entity_client.call_async(req)
        future.add_done_callback(
            lambda completed_future, entity_name=entity.name: self._log_spawn_result(
                entity_name, completed_future
            )
        )
        return True

    def spawn_entities(self, entities: list[Entity]) -> bool:
        success = True
        for entity in entities:
            self.get_logger().info(f"Spawning {entity.name}")
            success = self.spawn_entity(entity) and success
        return success

    def _wait_for_spawn_service(self, timeout_sec: float = 10.0) -> bool:
        if self.spawn_entity_client.service_is_ready():
            return True

        self.get_logger().info(f"Waiting for spawn service /world/{self.world}/create...")
        ready = self.spawn_entity_client.wait_for_service(timeout_sec=timeout_sec)
        if not ready:
            self.get_logger().error(
                f"Spawn service /world/{self.world}/create not ready after {timeout_sec:.1f}s"
            )
        return ready

    def _log_spawn_result(self, name: str, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to spawn entity '{name}': {exc}")
            return

        if getattr(response, "success", False):
            self.get_logger().info(f"Spawned entity successfully: {name}")
            return

        status_message = getattr(response, "status_message", "unknown error")
        self.get_logger().error(f"Failed to spawn entity '{name}': {status_message}")

    def generate_world_callback(self, request, response):
        self.get_logger().info("Starting Dynamic World Generation!")
        if not self._wait_for_spawn_service():
            response.success = False
            response.message = "Spawn service not ready"
            return response
        response.success = self.generate_world()
        response.message = (
            "World generation successful" if response.success else "World generation failed"
        )
        return response

    @abstractmethod
    def generate_world(self) -> bool:
        """
        Generate the world file and write it to self.output_path.

        This method should:
        1. Generate world entities/obstacles based on competition parameters as Entity objects
        2. Dynamically load in the entities through the ros_gz entity create bridge
        3. Store any world metadata needed for services/topics

        return True if dynamic generation is successful, False otherwise
        """

    # NOTE: This function doesn't do anything bc world nodes don't generate .sdf world files
    def set_physics(self, world: ET.Element, physics: dict) -> None:
        """
        Set the physics of the ground plane in the world.
        """
        for key, value in physics.items():
            if key == "friction":
                ns = ""
                # Find <model name="ground_plane"> element
                ground_plane_model = None
                xpaths = [f"{ns}model", "model", ".//model"]
                found = False
                for xpath in xpaths:
                    for model in world.findall(xpath):
                        if model.get("name") == "ground_plane":
                            ground_plane_model = model
                            self.get_logger().info(f"Found <model name='ground_plane'> at {xpath}")
                            found = True
                            break
                    if found:
                        break

                if ground_plane_model is None:
                    self.get_logger().warn(
                        "No <model name='ground_plane'> found to update friction. Defaulting to 0.6."
                    )

                if ground_plane_model is not None:
                    # Traverse the tree to find <ode>
                    try:
                        link = (
                            ground_plane_model.find(f"{ns}link")
                            or ground_plane_model.find("link")
                            or ground_plane_model.find(".//link")
                        )
                        if link is not None:
                            collision = (
                                link.find(f"{ns}collision")
                                or link.find("collision")
                                or link.find(".//collision")
                            )
                            if collision is not None:
                                surface = (
                                    collision.find(f"{ns}surface")
                                    or collision.find("surface")
                                    or collision.find(".//surface")
                                )
                                if surface is not None:
                                    friction_elem = (
                                        surface.find(f"{ns}friction")
                                        or surface.find("friction")
                                        or surface.find(".//friction")
                                    )
                                    if friction_elem is not None:
                                        ode = (
                                            friction_elem.find(f"{ns}ode")
                                            or friction_elem.find("ode")
                                            or friction_elem.find(".//ode")
                                        )
                                        if ode is not None:
                                            mu = (
                                                ode.find(f"{ns}mu")
                                                or ode.find("mu")
                                                or ode.find(".//mu")
                                            )
                                            mu2 = (
                                                ode.find(f"{ns}mu2")
                                                or ode.find("mu2")
                                                or ode.find(".//mu2")
                                            )
                                            # only update if friction is provided. Otherwise, defaults to 0.6 in template.sdf
                                            if mu is not None:
                                                mu.text = str(value)
                                            if mu2 is not None:
                                                mu2.text = str(value)
                                            self.get_logger().info(
                                                f"Successfully updated <mu> and <mu2> in ground_plane friction to {value}"
                                            )
                    except Exception as e:
                        self.get_logger().warn(f"Couldn't update <mu>/<mu2> in ground_plane: {e}")
                else:
                    self.get_logger().warn(
                        "No <model name='ground_plane'> found to update <mu>/<mu2>"
                    )
