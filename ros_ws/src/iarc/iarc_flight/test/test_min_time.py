import numpy as np
import pytest

pytest.importorskip("pydrake")

from iarc_flight.trajectories.min_time import (  # noqa: E402
    CHECK_SAMPLES,
    SegmentLimits,
    limit_residuals,
    scaled_guess,
    solve_min_time,
)

WAYPOINTS = np.array([[0, 0, 0], [2, 1, 1], [5, 2, 1], [8, 0, 2]], dtype=float)
LIMITS = [SegmentLimits(3.0, 2.0), SegmentLimits(1.5, 1.0), SegmentLimits(4.0, 3.0)]


def test_scaled_guess_is_feasible_and_tight():
    r = limit_residuals(WAYPOINTS, scaled_guess(WAYPOINTS, LIMITS), LIMITS, CHECK_SAMPLES)
    assert r.max() <= 1e-9  # nothing over a limit
    assert r.max() > -1e-6  # and the worst point sits right at its limit


def test_optimized_is_feasible_and_faster_than_guess():
    res = solve_min_time(WAYPOINTS, LIMITS)
    assert res.solver in ("SNOPT", "IPOPT")
    r = limit_residuals(WAYPOINTS, res.durations, LIMITS, CHECK_SAMPLES)
    assert r.max() <= 1e-9  # nothing over a limit, even between the solver's sample points
    assert r.max() > -1e-6  # and something is tight, so no time was left on the table
    assert res.durations.sum() < res.initial_guess.sum()


def test_single_segment_matches_scaling():
    # One leg has nothing to trade off, so uniform scaling is already optimal
    wps = np.array([[0, 0, 0], [5, 0, 0]], dtype=float)
    limits = [SegmentLimits(3.0, 2.0)]
    res = solve_min_time(wps, limits)
    assert res.solver in ("SNOPT", "IPOPT")
    assert res.durations[0] == pytest.approx(res.initial_guess[0], rel=1e-3)


def test_mismatched_limits_rejected():
    with pytest.raises(ValueError):
        solve_min_time(WAYPOINTS, LIMITS[:2])


def test_config_per_waypoint_limits_and_summary():
    from iarc_flight.trajectories.min_time import MinTimeConfig

    cfg = MinTimeConfig(
        v_max=6.0,
        a_max=3.0,
        waypoints=[
            {"x": 5, "y": 0, "z": -3, "T": 4.0},
            {"x": 5, "y": 5, "z": -3, "v_max": 1.0, "a_max": 0.5},  # slow leg
            {"x": 0, "y": 5, "z": -3},
        ],
    )
    limits = cfg.limits()
    assert [(lim.v_max, lim.a_max) for lim in limits] == [(6.0, 3.0), (1.0, 0.5), (6.0, 3.0)]

    traj = cfg.build(np.array([0.0, 0.0, -3.0]), 0.0)
    wps = np.array([[0, 0, -3], [5, 0, -3], [5, 5, -3], [0, 5, -3]], dtype=float)
    T = np.diff(traj.position.times)
    assert limit_residuals(wps, T, limits, CHECK_SAMPLES).max() <= 1e-9
    assert traj.summary[0].startswith("min_time: 3 legs")
    assert traj.total_time == pytest.approx(T.sum())
