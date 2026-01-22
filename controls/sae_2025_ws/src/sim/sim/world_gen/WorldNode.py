#!/usr/bin/env python3
"""
Abstract base class for world nodes.

WorldNode is the ROS node abstraction for world-related services.
Nodes are created BY WorldGenerator subclasses via create_node().
"""

from abc import ABC

from rclpy.node import Node

from sim.utils import camel_to_snake


class WorldNode(Node, ABC):
    """
    Abstract base class for world nodes.

    WorldNode subclasses provide ROS services/topics for world information
    (e.g., obstacle positions, hoop lists). They are created BY WorldGenerator
    subclasses via create_node(), which passes generation data to the node.

    Directory creation and model copying is handled by sim.launch.py.
    """

    def __init__(self):
        """Initialize the WorldNode."""
        super().__init__(self.__class__.__name__)

    @classmethod
    def node_name(cls) -> str:
        """Get the node name in snake_case."""
        return camel_to_snake(cls.__name__)

    @classmethod
    def service_name(cls) -> str:
        """Get the service name prefix."""
        return f'world/{cls.node_name()}'