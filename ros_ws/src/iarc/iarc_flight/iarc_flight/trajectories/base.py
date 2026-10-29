from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrajectoryPoint:
    """A reference or setpoint at one instant, in the PX4 local NED frame."""

    position: np.ndarray  # (3,) meters
    velocity: np.ndarray  # (3,) m/s
    acceleration: np.ndarray  # (3,) m/s^2
    yaw: float  # radians


class Trajectory(ABC):
    """A path as a function of time. t = 0 is when the mode starts flying it."""

    @abstractmethod
    def sample(self, t: float) -> TrajectoryPoint:
        """Reference state at time t. After the trajectory is done it holds the final point."""

    @abstractmethod
    def done(self, t: float) -> bool:
        """True once the trajectory has finished at time t."""
