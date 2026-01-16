#!/usr/bin/env python3
"""
Abstract base class for world generation nodes.

Each competition should have a corresponding WorldNode subclass that:
1. Generates the world file based on competition-specific parameters
2. Writes the world file to ~/.simulation-gazebo/worlds/
3. Provides services/topics for world information (e.g., obstacle positions)
"""

from abc import ABC, abstractmethod
from pathlib import Path

from rclpy.node import Node

from sim.utils import camel_to_snake

class WorldNode(Node, ABC):
    """
    Abstract base class for world generation nodes.

    Subclasses implement generate_world() to create their world SDF file.
    Parent only handles path setup and calls generate_world().
    Directory creation and all model copying is handled by sim.launch.py.
    """

    def __init__(self):
        """
        Initialize the WorldNode.
        """
        super().__init__(self.__class__.__name__)

    @abstractmethod
    def generate_world(self, output_dir: Path) -> None:
        """
        Generate the world SDF file and write it to output_dir.

        Args:
            output_dir: Directory where world file will be written (created by sim.launch.py)
        """
        pass

    @classmethod
    def node_name(cls) -> str:
        """Get the node name in snake_case."""
        return camel_to_snake(cls.__name__)

    @classmethod
    def service_name(cls) -> str:
        """Get the service name prefix."""
        return f'world/{cls.node_name()}'