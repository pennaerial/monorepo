#!/usr/bin/env python3
"""
Abstract base class for world generators.

WorldGenerator is the primary abstraction for world generation. Each generator:
1. Handles full world generation (SDF file creation)
2. Creates and returns the associated ROS node with generation data
"""

from abc import ABC, abstractmethod
from pathlib import Path

from sim.world_gen.WorldNode import WorldNode


class WorldGenerator(ABC):
    """
    Abstract base class for world generators.

    Subclasses implement world generation and node creation.
    """

    @abstractmethod
    def generate_world(self, output_dir: Path) -> None:
        """
        Generate the world SDF file.

        Args:
            output_dir: Directory where world file will be written
        """
        pass

    @abstractmethod
    def create_node(self) -> WorldNode:
        """
        Create and return the ROS node.

        Returns:
            Configured WorldNode instance with data from generation
        """
        pass

    @abstractmethod
    def _validate_parameters(self) -> None:
        """
        Validate constructor parameters.

        Called during __init__ to ensure parameters are valid before generation.
        Raises ValueError if parameters are invalid.
        """
        pass
