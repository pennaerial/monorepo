from __future__ import annotations

import math
from typing import Literal

import numpy as np
from pydantic import BaseModel

from iarc_flight.trajectories.base import Trajectory, TrajectoryPoint


class CircleConfig(BaseModel):
    type: Literal["circle"] = "circle"
    radius: float = 5.0  # m
    initial_speed: float = 1.0  # m/s
    tangential_accel: float = 0.2  # m/s^2, negative slows down
    max_speed: float | None = 5.0  # m/s, None for no cap
    altitude: float | None = None  # m above the origin, None keeps the current height
    center: tuple[float, float] | None = None  # local NED (x, y), None starts at the drone
    laps: float = 1.0
    face_velocity: bool = True  # yaw along the path, otherwise hold the starting yaw

    def build(self, start: np.ndarray, yaw: float) -> Circle:
        return Circle(self, start, yaw)


class Circle(Trajectory):
    """Circle flown at a speed that changes at a constant rate until it hits max_speed or 0."""

    def __init__(self, c: CircleConfig, start: np.ndarray, yaw: float) -> None:
        self.c = c
        self.start_yaw = yaw
        self.altitude = c.altitude if c.altitude is not None else -start[2]
        # Without a center, put the drone on the circle at angle 0
        self.center = c.center if c.center is not None else (start[0] - c.radius, start[1])
        self.start_angle = math.atan2(start[1] - self.center[1], start[0] - self.center[0])
        self.length = c.laps * 2 * math.pi * c.radius

    def _progress(self, t: float) -> tuple[float, float, float]:
        """Speed, tangential acceleration and distance flown at time t."""
        v0, a = self.c.initial_speed, self.c.tangential_accel
        # Time when the speed stops changing: speeding up hits max_speed, slowing down hits 0
        if a > 0 and self.c.max_speed is not None:
            t_limit = max(0.0, (self.c.max_speed - v0) / a)
        elif a < 0:
            t_limit = v0 / -a
        else:
            t_limit = math.inf

        # Still accelerating: v = v0 + a*t, d = v0*t + a*t^2/2
        if t <= t_limit:
            return v0 + a * t, a, v0 * t + 0.5 * a * t**2
        # Past the limit: distance covered while accelerating, then constant speed v after that
        v = v0 + a * t_limit
        return v, 0.0, v0 * t_limit + 0.5 * a * t_limit**2 + v * (t - t_limit)

    def done(self, t: float) -> bool:
        return self._progress(t)[2] >= self.length

    def sample(self, t: float) -> TrajectoryPoint:
        speed, accel, dist = self._progress(t)
        if dist >= self.length:  # hold where the last lap ends
            speed, accel, dist = 0.0, 0.0, self.length

        r = self.c.radius
        theta = self.start_angle + dist / r
        cos, sin = math.cos(theta), math.sin(theta)
        a_c = speed**2 / r  # centripetal, toward the center

        position = np.array([self.center[0] + r * cos, self.center[1] + r * sin, -self.altitude])
        velocity = speed * np.array([-sin, cos, 0.0])
        acceleration = np.array([-a_c * cos - accel * sin, -a_c * sin + accel * cos, 0.0])
        yaw = math.atan2(cos, -sin) if self.c.face_velocity else self.start_yaw
        return TrajectoryPoint(position, velocity, acceleration, yaw)
