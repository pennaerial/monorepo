from __future__ import annotations

from typing import Literal

import numpy as np
from pydantic import BaseModel

from iarc_flight.controllers.base import Controller
from iarc_flight.trajectories.base import Trajectory, TrajectoryPoint


class FeedforwardConfig(BaseModel):
    type: Literal["feedforward"] = "feedforward"

    def build(self) -> FeedforwardController:
        return FeedforwardController()


class FeedforwardController(Controller):
    """Sends the trajectory as-is and lets PX4 track it."""

    def step(self, state: np.ndarray, trajectory: Trajectory, t: float) -> TrajectoryPoint:
        return trajectory.sample(t)
