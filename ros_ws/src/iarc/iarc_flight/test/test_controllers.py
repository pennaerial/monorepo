import numpy as np
import pytest
from iarc_flight.controllers import FeedforwardConfig, MPCConfig
from iarc_flight.trajectories import CircleConfig

CIRCLE = CircleConfig(radius=5.0, initial_speed=2.0, tangential_accel=0.0, altitude=3.0)


def on_circle(t: float):
    circle = CIRCLE.build(np.array([0.0, 0.0, -3.0]), start_yaw=0.0)
    ref = circle.sample(t)
    return circle, np.concatenate([ref.position, ref.velocity])


def test_feedforward_sends_reference():
    circle, state = on_circle(1.0)
    setpoint = FeedforwardConfig().build().step(state, circle, 1.0)
    assert setpoint.position == pytest.approx(circle.sample(1.0).position)


@pytest.fixture(scope="module")
def mpc():
    pytest.importorskip("cvxpy")
    return MPCConfig().build()


def test_mpc_tracks_reference_when_on_it(mpc):
    circle, state = on_circle(1.0)
    setpoint = mpc.step(state, circle, 1.0)
    # Next planned point is about one step (0.1 s at 2 m/s) along the circle
    assert setpoint.position == pytest.approx(circle.sample(1.1).position, abs=0.05)
    assert mpc.failed_solves == 0


def test_mpc_moves_toward_reference_and_respects_limits(mpc):
    circle, state = on_circle(0.0)
    state[:3] += [2.0, 0.0, 0.0]  # 2 m off the circle
    setpoint = mpc.step(state, circle, 0.0)
    ref = circle.sample(0.1).position
    assert np.linalg.norm(setpoint.position - ref) < np.linalg.norm(state[:3] - ref)
    # Limits hold to within OSQP's solver tolerance
    a_xy_max = 9.81 * np.tan(np.deg2rad(MPCConfig().max_tilt_deg))
    assert np.all(np.abs(setpoint.acceleration[:2]) <= a_xy_max + 1e-2)
    assert np.all(np.abs(setpoint.velocity) <= MPCConfig().max_speed + 1e-2)
