from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class TrajectoryPoint:
    """Position, velocity, acceleration (local NED) and yaw (rad) at one instant."""

    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    yaw: float


class Trajectory(ABC):
    """A path as a function of time, starting at t = 0."""

    @abstractmethod
    def sample(self, t: float) -> TrajectoryPoint: ...

    @abstractmethod
    def done(self, t: float) -> bool: ...
