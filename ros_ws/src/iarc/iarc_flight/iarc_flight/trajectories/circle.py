from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

import numpy as np
from pydantic import BaseModel

from .base import Trajectory, TrajectoryPoint

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class CircleState:
    """Reference state on the circle at one instant, in the PX4 local NED frame."""

    position: Vec3
    velocity: Vec3
    acceleration: Vec3
    yaw: float  # heading along the velocity, radians
    distance: float  # arc length flown since t = 0, meters


def circle_state(
    t: float,
    radius: float = 5.0,
    initial_speed: float = 2.0,
    tangential_accel: float = 0.5,
    altitude: float = 10.0,
    cx: float = 0.0,
    cy: float = 0.0,
    start_angle: float = 0.0,
    max_speed: float | None = None,
) -> CircleState:
    """
    Position, velocity and acceleration feedforward for a circle flown with changing speed.

    Speed starts at initial_speed and changes by tangential_accel each second. It stops at
    max_speed when accelerating, and at 0 when decelerating (negative tangential_accel).

    Args:
        t: Seconds since the start of the circle.
        radius: Circle radius in meters.
        initial_speed: Speed at t = 0, m/s.
        tangential_accel: Rate the speed changes along the circle, m/s^2.
        altitude: Height above the origin in meters (positive up; converted to NED z).
        cx, cy: Circle center in local NED (x north, y east), meters.
        start_angle: Angle on the circle at t = 0, radians from +x toward +y.
        max_speed: Speed cap in m/s, or None for no cap.

    Returns:
        The reference state at time t.
    """
    v, a_t, s = _speed_profile(t, initial_speed, tangential_accel, max_speed)

    theta = start_angle + s / radius
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    # Centripetal acceleration points at the center, tangential along the direction of travel
    a_c = v**2 / radius

    return CircleState(
        position=(cx + radius * cos_t, cy + radius * sin_t, -altitude),
        velocity=(-v * sin_t, v * cos_t, 0.0),
        acceleration=(-a_c * cos_t - a_t * sin_t, -a_c * sin_t + a_t * cos_t, 0.0),
        yaw=math.atan2(cos_t, -sin_t),
        distance=s,
    )


def _speed_profile(
    t: float, v0: float, accel: float, max_speed: float | None
) -> tuple[float, float, float]:
    """
    Speed, tangential acceleration and distance at time t for constant acceleration until a limit.

    Returns:
        (speed, tangential acceleration, arc length since t = 0)
    """
    # Time when the speed reaches its limit; after that it stays constant
    if accel > 0 and max_speed is not None:
        t_limit = max(0.0, (max_speed - v0) / accel)
    elif accel < 0:
        t_limit = v0 / -accel
    else:
        t_limit = math.inf

    if t <= t_limit:
        return v0 + accel * t, accel, v0 * t + 0.5 * accel * t**2

    v_limit = v0 + accel * t_limit
    s_limit = v0 * t_limit + 0.5 * accel * t_limit**2
    return v_limit, 0.0, s_limit + v_limit * (t - t_limit)


class CircleConfig(BaseModel):
    """
    radius: Circle radius in meters.
    initial_speed: Speed when the circle starts, m/s.
    tangential_accel: Rate the speed changes along the circle, m/s^2.
    max_speed: Speed cap in m/s, or null for no cap.
    altitude: Height above the origin in meters (positive up), or null to keep the current height.
    center: (x, y) center in local NED, or null to place the circle so it starts at the drone.
    laps: Number of times around before the trajectory is done.
    face_velocity: If true, yaw along the direction of travel; otherwise hold the starting yaw.
    """

    type: Literal["circle"] = "circle"
    radius: float = 5.0
    initial_speed: float = 1.0
    tangential_accel: float = 0.2
    max_speed: float | None = 5.0
    altitude: float | None = None
    center: tuple[float, float] | None = None
    laps: float = 1.0
    face_velocity: bool = True

    def build(self, start_position: np.ndarray, start_yaw: float) -> Circle:
        """
        Fixes the circle's center, altitude and start angle from the drone's position.

        Without a configured center, the circle is placed so the drone is already on it at angle 0.
        Otherwise the circle starts at the point nearest the drone.
        """
        x, y, z = start_position
        altitude = self.altitude if self.altitude is not None else -z
        if self.center is None:
            return Circle(self, x - self.radius, y, altitude, 0.0, start_yaw)
        cx, cy = self.center
        return Circle(self, cx, cy, altitude, math.atan2(y - cy, x - cx), start_yaw)


class Circle(Trajectory):
    def __init__(
        self,
        config: CircleConfig,
        cx: float,
        cy: float,
        altitude: float,
        start_angle: float,
        start_yaw: float,
    ) -> None:
        self.c = config
        self.cx, self.cy, self.altitude = cx, cy, altitude
        self.start_angle = start_angle
        self.start_yaw = start_yaw
        self.length = config.laps * 2 * math.pi * config.radius

    def _state(self, t: float) -> CircleState:
        return circle_state(
            t,
            radius=self.c.radius,
            initial_speed=self.c.initial_speed,
            tangential_accel=self.c.tangential_accel,
            altitude=self.altitude,
            cx=self.cx,
            cy=self.cy,
            start_angle=self.start_angle,
            max_speed=self.c.max_speed,
        )

    def done(self, t: float) -> bool:
        return self._state(t).distance >= self.length

    def sample(self, t: float) -> TrajectoryPoint:
        state = self._state(t)
        yaw = state.yaw if self.c.face_velocity else self.start_yaw
        if state.distance >= self.length:
            # Hold the point where the last lap ends
            theta = self.start_angle + self.length / self.c.radius
            end = (
                self.cx + self.c.radius * math.cos(theta),
                self.cy + self.c.radius * math.sin(theta),
                -self.altitude,
            )
            return TrajectoryPoint(np.array(end), np.zeros(3), np.zeros(3), yaw)
        return TrajectoryPoint(
            np.array(state.position), np.array(state.velocity), np.array(state.acceleration), yaw
        )
