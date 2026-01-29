#!/usr/bin/env python3
"""
Abstract base class for world generation nodes.

Each competition should have a corresponding WorldNode subclass that:
1. Generates the world file based on competition-specific parameters
2. Writes the world file to ~/.simulation-gazebo/worlds/
3. Provides services/topics for world information (e.g., obstacle positions)
"""

from pathlib import Path
from typing import Optional
from abc import ABC, abstractmethod
from rclpy.node import Node
from sim.utils import camel_to_snake, find_package_resource, copy_models_to_gazebo
import xml.etree.ElementTree as ET
import logging
import random
from std_srvs.srv import Trigger
from ros_gz_interfaces.srv import SpawnEntity

class WorldNode(Node, ABC):
    """
    Abstract base class for world generation nodes.
    
    Subclasses must implement:
    - generate_world(): Generate and write the world file
    - get_world_path(): Return the path to the generated world file
    """

    def __init__(self, competition_name: str, output_filename: Optional[str] = None, seed: Optional[int] = None):
        """
        Initialize the WorldNode.

        Args:
            competition_name: Name of the competition (e.g., 'in_house')
            output_filename: Optional custom output filename. If None, uses {competition_name}.sdf
            seed: Optional random seed for reproducible world generation. If None, uses unseeded RNG.
        """
        super().__init__(self.__class__.__name__)
        self.competition_name = competition_name

        # Initialize local RNG instance (isolated from global random state)
        self.rng = random.Random(seed)
        self.get_logger().info(f"Initialized world RNG with seed: {seed if seed is not None else 'unseeded'}")
        
        # Determine output path
        if output_filename is None:
            output_filename = f"{competition_name}.sdf"
        
        self.output_dir = Path.home() / '.simulation-gazebo' / 'worlds'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.output_path = self.output_dir / output_filename
        
        self.get_logger().info(f"Initializing world node for competition: {competition_name}")
        self.setup_gazebo_models(self.get_logger())
        self.trigger_srv = self.create_service(Trigger, f"/{self.get_name()}/trigger_world_gen", self.trigger_world_gen_req)
        self.spawn_entity_client = self.create_client(SpawnEntity, f"/world/{competition_name}/create")

    def setup_gazebo_models(self, logger: logging.Logger) -> None:
        """
        Setup Gazebo models by copying from package to user's Gazebo models directory.
        This ensures models are available to Gazebo at runtime.
        """
        try:
            src_models_dir = find_package_resource(
                relative_path='world_gen/models',
                package_name='sim',
                resource_type='directory',
                logger=logger,
                base_file=Path(__file__)
            )
            
            # Ensure output directory exists
            output_world_dir = Path.home() / '.simulation-gazebo' / 'worlds'
            output_world_dir.mkdir(parents=True, exist_ok=True)
            
            dst_models_dir = Path.home() / '.simulation-gazebo' / 'models'
            
            # Copy models
            import time

            max_wait_time = 30  # seconds
            poll_interval = 0.2  # seconds
            waited = 0
            while not dst_models_dir.exists():
                time.sleep(poll_interval)
                waited += poll_interval
                if waited >= max_wait_time:
                    raise TimeoutError(f"Gazebo models directory {dst_models_dir} was not created after {max_wait_time} seconds.")

            copy_models_to_gazebo(src_models_dir, dst_models_dir)
        except Exception as e:
            logger.warning(f"Model setup failed (continuing anyway): {e}")
    
    def instantiate_static_world(self, template_world_path: str) -> None:
        """
        Prepare the template world with the correct competition name prior to dynamic generation.
        Writes the XML to self.output_path.
        Add any preprocessing here that is necessary before dynamic gen.

        """

        # Expand ~, make absolute, and validate paths
        in_path = Path(template_world_path).expanduser().resolve()
        out_path = Path(self.output_path).expanduser().resolve()

        if not in_path.exists():
            raise FileNotFoundError(
                f"Input SDF not found: {in_path}\nCWD: {Path.cwd().resolve()}"
            )
        try:
            out_path.parent.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            raise OSError(f"Failed to create output directory {out_path.parent}: {e}")
        
        # Parse
        try:
            tree = ET.parse(str(in_path))  # str() for safety on older libs
            root = tree.getroot()
        except ET.ParseError as e:
            raise ValueError(f"Failed to parse XML file {in_path}: {e}")
        except Exception as e:
            raise RuntimeError(f"Error reading SDF file {in_path}: {e}")

        # handle namespace if present (root.tag may be like "{...}sdf")
        ns = ""
        if root.tag.startswith("{"):
            ns = root.tag.split("}")[0] + "}"

        # try to find <world> with/without namespace
        world = root.find(f"{ns}world") or root.find("world") or root.find(".//world")
        if world is None:
            raise RuntimeError("No <world> tag found in the SDF!")
        
        old_name = world.get("name")
        new_name = self.competition_name

        world.set("name", new_name)
        if old_name != new_name:
            self.get_logger().info(f"World name changed: {old_name} to {new_name}")
        
        try:
            tree.write(
                str(out_path),
                encoding="utf-8",
                xml_declaration=True
            )
            self.get_logger().info(f"Successfully generated world file: {out_path}")

        except Exception as e:
            raise RuntimeError(f"Failed to write output SDF to {out_path}: {e}")
    
    def trigger_world_gen_req(self, request, response):
        self.get_logger().info("Starting Dynamic World Generation!")
        response.success = self.generate_world()
        response.message = "World generation successful" if response.success else "World generation failed"
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
        return True


    def get_world_path(self) -> Path:
        """
        Get the path to the generated world file.
        
        Returns:
            Path to the world SDF file
        """
        return self.output_path

    @classmethod
    def node_name(cls) -> str:
        """Get the node name in snake_case."""
        return camel_to_snake(cls.__name__)

    @classmethod
    def service_name(cls) -> str:
        """Get the service name prefix."""
        return f'world/{cls.node_name()}'