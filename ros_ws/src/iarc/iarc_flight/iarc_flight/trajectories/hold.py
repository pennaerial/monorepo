import numpy as np

from iarc_flight.trajectories.base import Trajectory, TrajectoryPoint


class Hold(Trajectory):
    """Stay at one point at zero velocity"""

    def __init__(self, position: np.ndarray, yaw: float) -> None:
        self.point = TrajectoryPoint(np.asarray(position, float), np.zeros(3), np.zeros(3), yaw)

    def sample(self, t: float) -> TrajectoryPoint:
        return self.point

    def done(self, t: float) -> bool:
        return False
