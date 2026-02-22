#!/usr/bin/env python3
"""
Abstract base class for scoring nodes.

Each competition should have a corresponding ScoringNode subclass that:
1. Tracks competition-specific scoring criteria
2. Publishes score updates
3. Monitors UAV state and competition progress
"""

from typing import Optional
from abc import ABC, abstractmethod
from rclpy.node import Node
from uav.utils import camel_to_snake
from std_msgs.msg import Float32, String


class ScoringNode(Node, ABC):
    """
    Abstract base class for competition scoring nodes.

    Subclasses must implement:
    - update_scoring(): Update score based on current competition state
    - get_current_score(): Return the current score
    - reset_scoring(): Reset scoring state (optional, for restart scenarios)
    """

    def __init__(self, competition_name: str):
        """
        Initialize the ScoringNode.

        Args:
            competition_name: Name of the competition (e.g., 'in_house')
        """
        super().__init__(self.__class__.__name__)
        self.competition_name = competition_name
        self.current_score = 0.0

        # Publishers for score updates
        self.score_publisher = self.create_publisher(
            Float32, f"/scoring/{competition_name}/score", 10
        )
        self.status_publisher = self.create_publisher(
            String, f"/scoring/{competition_name}/status", 10
        )

        self.get_logger().info(
            f"Initialized scoring node for competition: {competition_name}"
        )

    @abstractmethod
    def update_scoring(self) -> None:
        """
        Update scoring based on current competition state.

        This method should be called periodically (via timer or callback)
        to check competition progress and update scores.
        """
        pass

    def get_current_score(self) -> float:
        """
        Get the current score.

        Returns:
            Current score value
        """
        return self.current_score

    def publish_score(self, score: Optional[float] = None) -> None:
        """
        Publish the current score.

        Args:
            score: Score to publish. If None, uses self.current_score
        """
        if score is None:
            score = self.current_score

        msg = Float32()
        msg.data = float(score)
        self.score_publisher.publish(msg)

    def publish_status(self, status: str) -> None:
        """
        Publish a status message.

        Args:
            status: Status message string
        """
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Status: {status}")

    def reset_scoring(self) -> None:
        """
        Reset scoring state.

        Subclasses can override this to reset competition-specific state.
        """
        self.current_score = 0.0
        self.get_logger().info("Scoring reset")

    @classmethod
    def node_name(cls) -> str:
        """Get the node name in snake_case."""
        return camel_to_snake(cls.__name__)

    @classmethod
    def service_name(cls) -> str:
        """Get the service name prefix."""
        return f"scoring/{cls.node_name()}"
