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
import logging

class WorldNode(Node, ABC):
    """
    Abstract base class for world generation nodes.
    
    Subclasses must implement:
    - generate_world(): Generate and write the world file
    - get_world_path(): Return the path to the generated world file
    """

    def __init__(self, competition_name: str, output_filename: Optional[str] = None):
        """
        Initialize the WorldNode.
        
        Args:
            competition_name: Name of the competition (e.g., 'in_house')
            output_filename: Optional custom output filename. If None, uses {competition_name}.sdf
        """
        super().__init__(self.__class__.__name__)
        self.competition_name = competition_name
        
        # Determine output path
        if output_filename is None:
            output_filename = f"{competition_name}.sdf"
        
        self.output_dir = Path.home() / '.simulation-gazebo' / 'worlds'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.output_path = self.output_dir / output_filename
        
        self.get_logger().info(f"Initializing world node for competition: {competition_name}")
        
        self.setup_gazebo_models(self.get_logger())
        self.get_logger().info(f"Wrote world file to: {self.output_path}")

    def setup_gazebo_models(self, logger: logging.Logger) -> None:
        """
        Setup Gazebo models by copying from package to user's Gazebo models directory.
        This ensures models are available to Gazebo at runtime.
        Only copies models required for this competition.
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

            # Get list of required models for this competition
            required_models = self.get_required_models()
            logger.info(f"Copying {len(required_models)} required models: {required_models}")

            copy_models_to_gazebo(src_models_dir, dst_models_dir, models=required_models)
        except Exception as e:
            logger.warning(f"Model setup failed (continuing anyway): {e}")

    @abstractmethod
    def generate_world(self) -> None:
        """
        Generate the world file and write it to self.output_path.

        This method should:
        1. Generate world geometry/obstacles based on competition parameters
        2. Write the SDF world file to self.output_path
        3. Store any world metadata needed for services/topics
        """
        pass

    @abstractmethod
    def get_required_models(self) -> list[str]:
        """
        Return a list of model names required for this competition.

        This method should return the names of all models that need to be
        copied to the Gazebo models directory for this world to function.

        Returns:
            List of model directory names (e.g., ['hoop', 'dlz_red', 'payload'])
        """
        pass

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