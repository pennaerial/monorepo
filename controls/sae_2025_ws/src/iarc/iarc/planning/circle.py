from __future__ import annotations

import math
from dataclasses import dataclass

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
