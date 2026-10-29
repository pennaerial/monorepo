from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from iarc_flight.trajectories import Trajectory, TrajectoryPoint


class Controller(ABC):
    """Turns the vehicle state and a trajectory into the setpoint sent to PX4 each tick."""

    @abstractmethod
    def step(self, state: np.ndarray, trajectory: Trajectory, t: float) -> TrajectoryPoint:
        """
        Args:
            state: [x, y, z, vx, vy, vz] in the PX4 local NED frame.
            trajectory: The trajectory being tracked.
            t: Seconds since the trajectory started.
        """
