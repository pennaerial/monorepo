"""Minimum-snap trajectory through keyframes.

Makes a smooth path through keyframes (x, y, z, yaw), starting and ending at rest
Position is one 7-deg polynomial per leg minimizing the integral of snap^2
yaw is cubic per leg minimizing (d^2/dt^2 yaw)^2
Leg times T come from the mission or min_snap_time.optimize_times

This QP has no inequality constraints and has an exact cost Hessian, so it can be solved
in one np.linalg.solve.

Adapted from:
D. Mellinger, V. Kumar, "Minimum Snap Trajectory Generation and Control for
Quadrotors", ICRA 2011.
"""

from __future__ import annotations

import math
from typing import Literal

import numpy as np
from numpy.polynomial import polynomial as P
from pydantic import BaseModel, Field, PositiveFloat, model_validator
from scipy.linalg import block_diag

from iarc_flight.trajectories.base import Trajectory, TrajectoryPoint


class Keyframe(BaseModel):
    """Local NED target, T and mode describe the leg flown to reach it."""

    x: float  # m
    y: float  # m
    z: float  # m
    yaw: float  # rad
    T: float | None = Field(default=None, gt=0)  # s, required unless time_optimal
    mode: str | None = None  # key into v_max / a_max; required if time_optimal


class MinSnapConfig(BaseModel):
    type: Literal["min_snap"] = "min_snap"
    keyframes: list[Keyframe] = Field(min_length=1)
    time_optimal: bool = False  # choose T as short as v_max / a_max allow
    v_max: dict[str, PositiveFloat] = {}  # m/s per mode, e.g. {move: 4, scan: 1.5}
    a_max: dict[str, PositiveFloat] = {}  # m/s^2 per mode
    yaw_rate_max: float = Field(default=math.pi / 2, gt=0)  # rad/s, lower-bounds each T

    @model_validator(mode="after")
    def _check(self) -> MinSnapConfig:
        if self.time_optimal:
            self.limits()
        elif any(kf.T is None for kf in self.keyframes):
            raise ValueError("every keyframe needs T unless time_optimal: true")
        return self

    def limits(self) -> tuple[np.ndarray, np.ndarray]:
        """Per-leg limits"""
        for i, kf in enumerate(self.keyframes):
            mode = kf.mode
            if mode is None or mode not in self.v_max or mode not in self.a_max:
                raise ValueError(f"keyframe {i} mode {mode!r} needs an entry in v_max and a_max")
        v_max = np.array([self.v_max[kf.mode] for kf in self.keyframes if kf.mode is not None])
        a_max = np.array([self.a_max[kf.mode] for kf in self.keyframes if kf.mode is not None])
        return v_max, a_max

    def build(self, start: np.ndarray, yaw: float) -> MinSnap:
        return MinSnap(self, start, yaw)


def min_derivative_kkt(w: np.ndarray, T: np.ndarray, r: int):
    """KKT matrix for min-derivative problem"""
    M, D, n = len(T), w.shape[1], 2 * r
    i = np.arange(r, n)  # exact Hessian on s in [0, 1], scaled by T^(1-2r) per leg
    f = np.array([math.perm(k, r) for k in i])
    Q = np.zeros((n, n))
    Q[r:, r:] = np.outer(f, f) / (i[:, None] + i - 2 * r + 1)
    H = block_diag(*(Q * Tk ** (1 - 2 * r) for Tk in T))

    def d(k: int, m: int, s: float) -> np.ndarray:  # row giving the m-th time derivative of leg k
        row = np.zeros(M * n)
        row[k * n + m : k * n + n] = [
            math.perm(j, m) * s ** (j - m) / T[k] ** m for j in range(m, n)
        ]
        return row

    z, ends = np.zeros(D), ((0, 0), (M - 1, 1))
    rows = [(d(k, 0, e), w[k + e], 0) for k in range(M) for e in (0, 1)]  # through keyframes
    rows += [(d(k, m, e), z, m) for m in range(1, r) for k, e in ends]  # at rest at both ends
    rows += [(d(k, m, 1) - d(k + 1, m, 0), z, m) for k in range(M - 1) for m in range(1, r + 1)]
    A, b, order = (np.array(x) for x in zip(*rows))
    K = np.block([[2 * H, A.T], [A, np.zeros((len(A), len(A)))]])
    return K, np.vstack([np.zeros((M * n, D)), b]), order


def solve_min_derivative(w: np.ndarray, T: np.ndarray, r: int) -> np.ndarray:
    """Coeffs (M, 2r, D); leg k is sum_i c[k, i] s^i with s = (t - t_k) / T_k. r=4: snap."""
    K, rhs, _ = min_derivative_kkt(w, T, r)
    # Exact: equality constraints only. K is symmetric indefinite (saddle point), so plain solve
    return np.linalg.solve(K, rhs)[: len(T) * 2 * r].reshape(len(T), 2 * r, -1)


def unwrap_yaws(yaws: list[float]) -> np.ndarray:
    """Shift each yaw by 2 pi to within pi of the previous one (never turn the long way)."""
    out = [yaws[0]]
    for y in yaws[1:]:
        out.append(out[-1] + (y - out[-1] + math.pi) % (2 * math.pi) - math.pi)
    return np.array(out)


class PiecewisePolynomial:
    def __init__(self, coeffs: np.ndarray, T: np.ndarray) -> None:
        self.c, self.T, self.knots = coeffs, T, np.r_[0.0, np.cumsum(T)]

    def __call__(self, t: float, m: int = 0) -> np.ndarray:
        """m-th time derivative at t (clamped to the trajectory)."""
        t = min(max(t, 0.0), self.knots[-1])
        k = min(np.searchsorted(self.knots, t, side="right") - 1, len(self.T) - 1)
        s = (t - self.knots[k]) / self.T[k]
        return P.polyval(s, P.polyder(self.c[k], m, axis=0)) / self.T[k] ** m


class MinSnap(Trajectory):
    def __init__(self, c: MinSnapConfig, start: np.ndarray, yaw: float) -> None:
        kfs = c.keyframes
        self.keyframes = np.vstack([start[:3], [[kf.x, kf.y, kf.z] for kf in kfs]])  # (M+1, 3)
        yaws = unwrap_yaws([yaw] + [kf.yaw for kf in kfs])
        self.time_allocation = None
        if c.time_optimal:
            from iarc_flight.trajectories.min_snap_time import optimize_times  # needs pydrake

            pts = np.column_stack([self.keyframes, yaws])
            self.time_allocation = optimize_times(pts, *c.limits(), c.yaw_rate_max)
            T = self.durations = self.time_allocation.durations
        else:
            T = self.durations = np.array([kf.T for kf in kfs])
        self.position = PiecewisePolynomial(solve_min_derivative(self.keyframes, T, 4), T)
        self.yaw = PiecewisePolynomial(solve_min_derivative(yaws[:, None], T, 2), T)
        self.total_time = float(self.position.knots[-1])
        ts = np.linspace(0.0, self.total_time, 50 * len(kfs) + 1)  # peaks, for TrackMode's log
        peak = lambda m: max(float(np.linalg.norm(self.position(t, m))) for t in ts)  # noqa: E731
        self.max_speed, self.max_accel = peak(1), peak(2)

    def done(self, t: float) -> bool:
        return t >= self.total_time

    def sample(self, t: float) -> TrajectoryPoint:  # past the end: last keyframe, at rest
        p = self.position
        return TrajectoryPoint(p(t), p(t, 1), p(t, 2), float(self.yaw(t)[0]))
