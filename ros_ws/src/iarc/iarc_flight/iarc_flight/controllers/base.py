from abc import ABC, abstractmethod

import numpy as np

from iarc_flight.trajectories.base import Trajectory, TrajectoryPoint


class Controller(ABC):
    """Turns the drone state and a trajectory into the setpoint sent to PX4."""

    @abstractmethod
    def step(self, state: np.ndarray, trajectory: Trajectory, t: float) -> TrajectoryPoint:
        """state is [x, y, z, vx, vy, vz] in local NED, t is seconds into the trajectory."""
