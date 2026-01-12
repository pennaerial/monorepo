#!/usr/bin/env python3
"""
Abstract base class for world generation nodes.

Each competition should have a corresponding WorldNode subclass that:
1. Generates the world file based on competition-specific parameters
2. Writes the world file to ~/.simulation-gazebo/worlds/
3. Provides services/topics for world information (e.g., obstacle positions)
"""

from pathlib import Path
from abc import ABC, abstractmethod
from rclpy.node import Node
from sim.utils import camel_to_snake, find_package_resource, copy_models_to_gazebo
import logging

class WorldNode(Node, ABC):
    """
    Abstract base class for world generation nodes.

    Subclasses implement generate_world() to create their world SDF file.
    Parent handles file generation, model detection, and model copying automatically.
    """

    def __init__(self, competition_name: str):
        """
        Initialize the WorldNode.

        Args:
            competition_name: Name of the competition (e.g., 'in_house')
        """
        super().__init__(self.__class__.__name__)
        self.competition_name = competition_name

        # Output filename is always {competition_name}.sdf
        output_filename = f"{competition_name}.sdf"

        self.output_dir = Path.home() / '.simulation-gazebo' / 'worlds'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.output_path = self.output_dir / output_filename

        self.get_logger().info(f"Initializing world node for competition: {competition_name}")

        # Generate the world file (subclass implements this)
        self.generate_world()

        # Auto-detect and copy models from the generated world file
        self.setup_gazebo_models(self.get_logger())
        self.get_logger().info(f"World file ready at: {self.output_path}")

    @abstractmethod
    def generate_world(self) -> None:
        """Generate the world SDF file and write it to self.output_path."""
        pass

    def setup_gazebo_models(self, logger: logging.Logger) -> None:
        """
        Setup Gazebo sim models by automatically detecting and copying them.
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

            # Automatically extract models from the generated world file
            if self.output_path.exists():
                from sim.utils import extract_models_from_sdf
                required_models = extract_models_from_sdf(self.output_path)
                logger.info(f"Auto-detected {len(required_models)} models from world file: {required_models}")

                if required_models:
                    copy_models_to_gazebo(src_models_dir, dst_models_dir, models=required_models)
                else:
                    logger.warning(f"No models detected in {self.output_path}")
            else:
                logger.warning(f"World file not found at {self.output_path}, skipping model setup")
        except FileNotFoundError as e:
            logger.warning(f"World file not found, skipping model setup: {e}")
        except Exception as e:
            logger.error(f"Model setup failed: {e}")
            raise RuntimeError(f"Failed to setup Gazebo models: {e}") from e

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