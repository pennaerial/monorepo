import math

import numpy as np
import pytest
from iarc_flight.trajectories import CircleConfig, circle_state


def test_starts_at_start_angle():
    s = circle_state(0.0, radius=5.0, altitude=10.0, cx=1.0, cy=2.0)
    assert s.position == pytest.approx((6.0, 2.0, -10.0))
    assert s.distance == 0.0


def test_stays_on_circle():
    for t in [0.0, 1.3, 7.0, 42.0]:
        s = circle_state(t, radius=4.0, cx=1.0, cy=-2.0, max_speed=3.0)
        assert math.hypot(s.position[0] - 1.0, s.position[1] + 2.0) == pytest.approx(4.0)


def test_velocity_is_tangent_and_matches_speed():
    s = circle_state(2.0, radius=5.0, initial_speed=2.0, tangential_accel=0.5)
    radial = (s.position[0], s.position[1])
    assert radial[0] * s.velocity[0] + radial[1] * s.velocity[1] == pytest.approx(0.0, abs=1e-9)
    assert math.hypot(s.velocity[0], s.velocity[1]) == pytest.approx(3.0)


def test_speed_is_capped():
    s = circle_state(100.0, initial_speed=1.0, tangential_accel=1.0, max_speed=4.0)
    assert math.hypot(s.velocity[0], s.velocity[1]) == pytest.approx(4.0)
    # Only centripetal acceleration remains once the speed is constant
    assert math.hypot(s.acceleration[0], s.acceleration[1]) == pytest.approx(4.0**2 / 5.0)


def test_distance_after_cap():
    # Accelerate 1 -> 4 m/s over 3 s (7.5 m), then 2 s at 4 m/s (8 m)
    s = circle_state(5.0, initial_speed=1.0, tangential_accel=1.0, max_speed=4.0)
    assert s.distance == pytest.approx(15.5)


def test_deceleration_stops_at_zero():
    s = circle_state(10.0, initial_speed=2.0, tangential_accel=-1.0)
    assert math.hypot(s.velocity[0], s.velocity[1]) == pytest.approx(0.0)
    assert s.distance == pytest.approx(2.0)


def test_finite_difference_matches_velocity_and_acceleration():
    kw = dict(radius=6.0, initial_speed=1.5, tangential_accel=0.3)
    t, h = 3.0, 1e-4
    before, now, after = (circle_state(t + d, **kw) for d in (-h, 0.0, h))
    for i in range(2):
        assert (after.position[i] - before.position[i]) / (2 * h) == pytest.approx(
            now.velocity[i], rel=1e-5
        )
        assert (after.velocity[i] - before.velocity[i]) / (2 * h) == pytest.approx(
            now.acceleration[i], rel=1e-5
        )


def test_yaw_points_along_velocity():
    s = circle_state(1.7, radius=5.0)
    assert s.yaw == pytest.approx(math.atan2(s.velocity[1], s.velocity[0]))


def test_circle_starts_at_drone():
    circle = CircleConfig(radius=5.0).build(np.array([1.0, 2.0, -3.0]), start_yaw=0.0)
    assert circle.sample(0.0).position == pytest.approx([1.0, 2.0, -3.0])


def test_circle_holds_end_point_when_done():
    circle = CircleConfig(radius=2.0, initial_speed=1.0, tangential_accel=0.0, laps=1.0).build(
        np.zeros(3), start_yaw=0.0
    )
    lap_time = 2 * math.pi * 2.0
    assert not circle.done(lap_time - 0.1)
    assert circle.done(lap_time + 0.1)
    end = circle.sample(lap_time + 5.0)
    assert end.position == pytest.approx(
        [0.0, 0.0, 0.0], abs=1e-9
    )  # one full lap returns to the start
    assert end.velocity == pytest.approx([0.0, 0.0, 0.0])
