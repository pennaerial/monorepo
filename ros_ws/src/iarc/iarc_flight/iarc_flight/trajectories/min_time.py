"""Minimum-time segment durations for the min-snap trajectory.

Two layers:
  inner: solve_min_derivative (min_snap.py) turns fixed durations T into polynomials
  outer: SNOPT (through Drake) picks T to minimize sum(T) while every segment
         stays under its own speed and acceleration limits

The outer problem is nonconvex, so it's started from a feasible guess built by
temporal scaling, and that guess is the fallback if the solver fails.

MinTimeConfig is the mission-facing side (trajectory `type: min_time`): waypoints with
optional per-leg limits and time guesses, solved when TrackMode builds it, then flown
as a MinSnap trajectory. pydrake is imported only when a solve runs, so missions
that don't use min_time don't need it installed.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from functools import lru_cache
from typing import Literal

import numpy as np
from pydantic import BaseModel, Field

from iarc_flight.trajectories.min_snap import Keyframe, MinSnap, MinSnapConfig, solve_min_derivative


@dataclass
class SegmentLimits:
    v_max: float  # m/s, peak speed allowed in this segment
    a_max: float  # m/s^2, peak acceleration allowed in this segment
    T_min: float = 0.2  # s, keeps the solver away from T -> 0
    T_max: float = 60.0  # s


@dataclass
class MinTimeResult:
    durations: np.ndarray  # (M,) seconds per segment
    initial_guess: np.ndarray  # (M,) the temporally scaled guess
    solver: str  # "SNOPT", "IPOPT", or "fallback" if the solver result was rejected
    message: str
    solve_time: float = 0.0  # s, wall time for the whole solve_min_time call
    inner_solves: int = 0  # min-snap solves run, the main cost
    # One row per solver iterate (the points it linearizes at), starting with the guess:
    # (total time s, worst limit residual, seconds since the solve started).
    history: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))


CHECK_SAMPLES = 500  # points per segment for the final check (and the scaling that passes it)


@lru_cache(maxsize=8)
def _basis(samples: int) -> tuple[np.ndarray, np.ndarray]:
    """Rows that turn 8 coefficients into d/ds and d^2/ds^2 at each sample s in [0, 1]."""
    s = np.linspace(0.0, 1.0, samples)
    i = np.arange(8)
    d1 = i * s[:, None] ** np.maximum(i - 1, 0)  # d/ds s^i = i s^(i-1)
    d2 = i * (i - 1) * s[:, None] ** np.maximum(i - 2, 0)
    return d1, d2


def limit_residuals(
    waypoints: np.ndarray, T: np.ndarray, limits: list[SegmentLimits], samples: int = 30
) -> np.ndarray:
    """One value per sample point: |v|^2/v_max^2 - 1 and |a|^2/a_max^2 - 1.

    <= 0 means that sample is within its limit. Squared norms keep this smooth,
    which matters because the solver differentiates it. Ordered segment by
    segment, speed samples then acceleration samples, so reshape(M, 2, samples) works.
    """
    c = solve_min_derivative(waypoints, T, 4)  # (M, 8, 3), s in [0, 1] per segment
    d1, d2 = _basis(samples)
    # d/dt = (1/T) d/ds, so velocity divides by T and acceleration by T^2
    v = np.einsum("si,kid->ksd", d1, c) / T[:, None, None]  # (M, samples, 3)
    a = np.einsum("si,kid->ksd", d2, c) / T[:, None, None] ** 2
    v_max = np.array([lim.v_max for lim in limits])[:, None]
    a_max = np.array([lim.a_max for lim in limits])[:, None]
    rv = (v**2).sum(axis=2) / v_max**2 - 1.0  # (M, samples)
    ra = (a**2).sum(axis=2) / a_max**2 - 1.0
    return np.stack([rv, ra], axis=1).ravel()


def _scale_to_limits(
    waypoints: np.ndarray, T: np.ndarray, limits: list[SegmentLimits], samples: int = CHECK_SAMPLES
) -> np.ndarray:
    """Multiply every duration by one factor so the tightest limit is exactly met.

    Scaling all of T by alpha keeps the path shape and divides speed by alpha and
    acceleration by alpha^2, so the right alpha comes straight from the peaks.
    """
    r = limit_residuals(waypoints, T, limits, samples).reshape(len(limits), 2, -1) + 1.0
    speed_ratio = np.sqrt(r[:, 0].max())  # worst |v| / v_max
    accel_ratio = np.sqrt(r[:, 1].max())  # worst |a| / a_max
    alpha = max(speed_ratio, np.sqrt(accel_ratio))
    T_min = np.array([lim.T_min for lim in limits])
    T_max = np.array([lim.T_max for lim in limits])
    return np.clip(T * alpha, T_min, T_max)


def scaled_guess(
    waypoints: np.ndarray, limits: list[SegmentLimits], T: np.ndarray | None = None
) -> np.ndarray:
    """Feasible starting durations, scaled so the tightest limit is exactly met.

    T sets the relative time split between segments. NaN entries (or T=None) fall
    back to distance / v_max for that segment.
    """
    v_max = np.array([lim.v_max for lim in limits])
    T_min = np.array([lim.T_min for lim in limits])
    dist = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    rough = dist / v_max
    if T is not None:
        T = np.asarray(T, dtype=float)
        rough = np.where(np.isnan(T), rough, T)
    return _scale_to_limits(waypoints, np.maximum(rough, T_min), limits)


def _jacobian(f, T: np.ndarray, f0: np.ndarray, rel_step: float = 1e-6) -> np.ndarray:
    """Forward finite differences of f at T, given f0 = f(T). One inner solve per column."""
    cols = []
    for j in range(len(T)):
        h = rel_step * max(1.0, T[j])
        Tp = T.copy()
        Tp[j] += h
        cols.append((f(Tp) - f0) / h)
    return np.column_stack(cols)


def solve_min_time(
    waypoints: np.ndarray,
    limits: list[SegmentLimits],
    samples: int = 30,
    initial_guess: np.ndarray | None = None,
) -> MinTimeResult:
    """Shortest durations through waypoints (M+1, 3) that respect each segment's limits.

    initial_guess (M,) sets the starting time split; NaN entries use distance / v_max.
    """
    from pydrake.autodiffutils import ExtractGradient, ExtractValue, InitializeAutoDiff
    from pydrake.solvers import IpoptSolver, MathematicalProgram, SnoptSolver

    start_time = time.perf_counter()
    waypoints = np.asarray(waypoints, dtype=float)
    M = len(waypoints) - 1
    if len(limits) != M:
        raise ValueError(f"{M} segments but {len(limits)} SegmentLimits")

    inner = 0
    history: list[tuple[float, float, float]] = []

    def g(T: np.ndarray) -> np.ndarray:
        nonlocal inner
        inner += 1
        return limit_residuals(waypoints, T, limits, samples)

    def constraint(z):
        # Drake calls this with plain floats or with AutoDiffXd (values carrying
        # derivatives). For AutoDiff, compute the values and Jacobian ourselves and
        # hand both back, since solve_min_derivative only understands floats.
        if z.dtype == float:
            return g(z)
        T = ExtractValue(z).ravel()
        g0 = g(T)
        history.append((T.sum(), g0.max(), time.perf_counter() - start_time))
        J = _jacobian(g, T, g0) @ ExtractGradient(z)  # chain rule: dg/dz = dg/dT dT/dz
        return InitializeAutoDiff(g0, J).ravel()

    def feasible(T: np.ndarray) -> bool:
        return limit_residuals(waypoints, T, limits, CHECK_SAMPLES).max() <= 1e-9

    T0 = scaled_guess(waypoints, limits, initial_guess)
    if not feasible(T0):
        # Only happens when T_max clipping stops the scaling from slowing down enough
        raise ValueError("limits can't be met within the segments' T_max")
    history.append((T0.sum(), g(T0).max(), time.perf_counter() - start_time))

    prog = MathematicalProgram()
    T = prog.NewContinuousVariables(M, "T")
    prog.AddLinearCost(np.ones(M), 0.0, T)  # total flight time
    prog.AddBoundingBoxConstraint([lim.T_min for lim in limits], [lim.T_max for lim in limits], T)
    n = 2 * M * samples
    prog.AddConstraint(constraint, np.full(n, -np.inf), np.zeros(n), T, description="limits")
    prog.SetInitialGuess(T, T0)

    snopt = SnoptSolver()
    solver = snopt if snopt.available() and snopt.enabled() else IpoptSolver()
    result = solver.Solve(prog)
    name = "SNOPT" if solver is snopt else "IPOPT"
    status = str(result.get_solution_result())

    if result.is_success():
        # The solver only saw `samples` points per segment, so peaks between them can
        # sit a fraction of a percent over. Rescale against a dense grid to remove that.
        T_opt = _scale_to_limits(waypoints, result.GetSolution(T), limits)
        if feasible(T_opt) and T_opt.sum() <= T0.sum() * (1 + 1e-9):  # never worse than the guess
            elapsed = time.perf_counter() - start_time
            return MinTimeResult(T_opt, T0, name, status, elapsed, inner, np.array(history))
    elapsed = time.perf_counter() - start_time
    return MinTimeResult(T0, T0, "fallback", f"{name}: {status}", elapsed, inner, np.array(history))


class Waypoint(BaseModel):
    """A point to pass through. Limits and T apply to the leg arriving here."""

    x: float  # m, local NED
    y: float
    z: float
    yaw: float = 0.0  # rad
    v_max: float | None = Field(None, gt=0)  # m/s, None uses the trajectory's v_max
    a_max: float | None = Field(None, gt=0)  # m/s^2, None uses the trajectory's a_max
    T: float | None = Field(
        None, gt=0
    )  # s, initial guess for this leg's time (only the split matters)
    T_min: float | None = Field(None, gt=0)  # s, None uses the trajectory's T_min
    T_max: float | None = Field(None, gt=0)


class MinTimeConfig(BaseModel):
    """Min-snap trajectory whose leg times are solved for minimum total time."""

    type: Literal["min_time"] = "min_time"
    waypoints: list[Waypoint] = Field(min_length=1)
    v_max: float = Field(6.0, gt=0)  # defaults for legs that don't set their own
    a_max: float = Field(3.0, gt=0)
    T_min: float = Field(0.2, gt=0)
    T_max: float = Field(60.0, gt=0)
    samples: int = Field(30, ge=5)  # points per leg the solver checks

    def limits(self) -> list[SegmentLimits]:
        return [
            SegmentLimits(
                v_max=w.v_max or self.v_max,
                a_max=w.a_max or self.a_max,
                T_min=w.T_min or self.T_min,
                T_max=w.T_max or self.T_max,
            )
            for w in self.waypoints
        ]

    def build(self, start: np.ndarray, yaw: float) -> MinSnap:
        """Solve leg times from the drone's position through the waypoints, then build MinSnap."""
        wps = np.vstack([start[:3], [[w.x, w.y, w.z] for w in self.waypoints]])
        limits = self.limits()
        guess = np.array([np.nan if w.T is None else w.T for w in self.waypoints])
        res = solve_min_time(wps, limits, self.samples, guess)

        keyframes = [
            Keyframe(x=w.x, y=w.y, z=w.z, yaw=w.yaw, T=float(T))
            for w, T in zip(self.waypoints, res.durations)
        ]
        traj = MinSnap(MinSnapConfig(keyframes=keyframes), start, yaw)
        traj.summary = summarize(wps, limits, res)
        return traj


def summarize(wps: np.ndarray, limits: list[SegmentLimits], res: MinTimeResult) -> list[str]:
    """Human-readable solve report, one log line per string."""
    r = limit_residuals(wps, res.durations, limits, CHECK_SAMPLES).reshape(len(limits), 2, -1) + 1
    lines = [
        f"min_time: {len(limits)} legs, {res.solver} ({res.message}), solve {res.solve_time * 1e3:.0f} ms, "
        f"{res.inner_solves} min-snap solves, {len(res.history) - 1} iterations",
        f"min_time: total {res.initial_guess.sum():.2f} s (scaled guess) -> {res.durations.sum():.2f} s",
    ]
    for k, lim in enumerate(limits):
        v, a = np.sqrt(r[k, 0].max()) * lim.v_max, np.sqrt(r[k, 1].max()) * lim.a_max
        tight = [
            n
            for n, hit in [
                ("v", v >= 0.999 * lim.v_max),
                ("a", a >= 0.999 * lim.a_max),
                ("T_min", res.durations[k] <= 1.001 * lim.T_min),
            ]
            if hit
        ]
        x, y, z = wps[k + 1]
        lines.append(
            f"min_time: leg {k + 1:>2} -> ({x:g}, {y:g}, {z:g}): T {res.durations[k]:5.2f} s, "
            f"peak v {v:4.2f}/{lim.v_max:g} m/s, peak a {a:4.2f}/{lim.a_max:g} m/s^2"
            + (f"  [at limit: {', '.join(tight)}]" if tight else "")
        )
    return lines


def _main() -> None:
    """Solve a mission's min_time trajectory offline and print the report TrackMode would log.

    python3 -m iarc_flight.trajectories.min_time missions/iarc_min_time.yaml
    The drone is assumed to start at the first waypoint (in flight it's wherever the drone is).
    """
    import argparse

    import yaml

    ap = argparse.ArgumentParser(description=_main.__doc__)
    ap.add_argument("mission", help="mission yaml with a TrackMode using trajectory type min_time")
    args = ap.parse_args()

    modes = yaml.safe_load(open(args.mission))["modes"]
    name, mode = next(
        (n, m)
        for n, m in modes.items()
        if m.get("params", {}).get("trajectory", {}).get("type") == "min_time"
    )
    cfg = MinTimeConfig(**mode["params"]["trajectory"])
    w0 = cfg.waypoints[0]
    traj = cfg.build(np.array([w0.x, w0.y, w0.z]), w0.yaw)
    print(f"mode '{name}'")
    print("\n".join(traj.summary))


if __name__ == "__main__":
    _main()
