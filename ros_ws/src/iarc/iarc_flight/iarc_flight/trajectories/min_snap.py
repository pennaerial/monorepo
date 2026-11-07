"""min snap based on the paper

Minimum Snap Trajectory Generation and Control for Quadrotors
Daniel Mellinger and Vijay Kumar (ICRA 2011)

not including corridor or temporal scaling

M keyframes -> O(M) linalg system, solved in O(M^3) time, O(M log M) sampling
"""

from __future__ import annotations

import math
from typing import Literal

import numpy as np
from numpy.polynomial import polynomial as P
from pydantic import BaseModel, Field

from iarc_flight.trajectories.base import Trajectory, TrajectoryPoint


class Keyframe(BaseModel):
    x: float  # m
    y: float
    z: float
    yaw: float  # rad
    T: float = Field(gt=0)  # s, time from prev


class MinSnapConfig(BaseModel):
    type: Literal["min_snap"] = "min_snap"
    keyframes: list[Keyframe] = Field(min_length=1)

    def build(self, start: np.ndarray, yaw: float) -> MinSnap:
        return MinSnap(self, start, yaw)


def solve_min_derivative(waypoints: np.ndarray, durations: np.ndarray, r: int) -> np.ndarray:
    """Piecewise polynomial through the waypoints minimizing the integral of ||d^r p/dt^r||^2.

    waypoints is (M+1, D), durations is (M,), r is 4 for snap and 2 for acceleration.
    Starts and ends with derivatives 1..r-1 at zero.

    Returns coeffs (M, 2r, D): segment k is sum_i coeffs[k, i] * s^i with s = (t - t_k) / T_k
    in [0, 1]. Normalizing time per segment keeps the numbers well scaled.
    """
    M, D = len(durations), waypoints.shape[1]
    n = 2 * r  # coefficients per segment (degree 2r - 1)

    # Cost: integral over t of (d^r p/dt^r)^2. On [0, 1] in s the exact Hessian is
    # Q_ij = i!/(i-r)! * j!/(j-r)! / (i + j - 2r + 1). Since d^r/dt^r = T^-r d^r/ds^r and
    # dt = T ds, each segment's block is Q scaled by T^(1 - 2r).
    Q = np.zeros((n, n))
    for i in range(r, n):
        for j in range(r, n):
            Q[i, j] = math.perm(i, r) * math.perm(j, r) / (i + j - 2 * r + 1)
    H = np.zeros((M * n, M * n))
    for k, T in enumerate(durations):
        H[k * n : (k + 1) * n, k * n : (k + 1) * n] = Q * T ** (1 - 2 * r)

    def deriv(k: int, m: int, s: float) -> np.ndarray:
        """Row giving the m-th time derivative of segment k at local s."""
        row = np.zeros(M * n)
        for i in range(m, n):
            row[k * n + i] = math.perm(i, m) * s ** (i - m) / durations[k] ** m
        return row

    rows, rhs = [], []
    # Each segment starts and ends at its keyframes
    for k in range(M):
        rows += [deriv(k, 0, 0.0), deriv(k, 0, 1.0)]
        rhs += [waypoints[k], waypoints[k + 1]]
    # Start and end at rest
    for m in range(1, r):
        rows += [deriv(0, m, 0.0), deriv(M - 1, m, 1.0)]
        rhs += [np.zeros(D), np.zeros(D)]
    # Derivatives 1..r continuous at interior keyframes, values left free
    for k in range(M - 1):
        for m in range(1, r + 1):
            rows.append(deriv(k, m, 1.0) - deriv(k + 1, m, 0.0))
            rhs.append(np.zeros(D))
    A, b = np.array(rows), np.array(rhs)

    # KKT system for min c^T H c subject to A c = b, one right-hand side per axis
    KKT = np.block([[2 * H, A.T], [A, np.zeros((len(A), len(A)))]])
    c = np.linalg.solve(KKT, np.vstack([np.zeros((M * n, D)), b]))[: M * n]
    return c.reshape(M, n, D)


class PiecewisePolynomial:
    """Evaluates the output of solve_min_derivative in real time."""

    def __init__(self, coeffs: np.ndarray, durations: np.ndarray) -> None:
        self.coeffs = coeffs  # (M, n, D)
        self.durations = durations
        self.times = np.concatenate([[0.0], np.cumsum(durations)])
        self.total_time = float(self.times[-1])

    def __call__(self, t: float, m: int = 0) -> np.ndarray:
        """m-th time derivative at t. Times outside [0, total_time] are clamped."""
        t = min(max(t, 0.0), self.total_time)
        k = min(int(np.searchsorted(self.times, t, side="right")) - 1, len(self.durations) - 1)
        T = self.durations[k]
        c = P.polyder(self.coeffs[k], m, axis=0) if m else self.coeffs[k]
        return P.polyval((t - self.times[k]) / T, c) / T**m


class MinSnap(Trajectory):
    """Minimum-snap trajectory starting at the drone and passing through each keyframe on schedule."""

    def __init__(self, c: MinSnapConfig, start: np.ndarray, yaw: float) -> None:
        kfs = c.keyframes
        durations = np.array([kf.T for kf in kfs])

        self.keyframes = np.vstack([start[:3], [[kf.x, kf.y, kf.z] for kf in kfs]])  # (M+1, 3)
        self.position = PiecewisePolynomial(
            solve_min_derivative(self.keyframes, durations, 4), durations
        )

        # Unwrap so each keyframe's yaw is within pi of the previous one (no spinning the long way)
        yaws = [yaw]
        for kf in kfs:
            yaws.append(yaws[-1] + (kf.yaw - yaws[-1] + math.pi) % (2 * math.pi) - math.pi)
        self.yaw = PiecewisePolynomial(
            solve_min_derivative(np.array(yaws)[:, None], durations, 2), durations
        )

        self.total_time = self.position.total_time

        # Peak speed and acceleration, logged by TrackMode to check T against vehicle limits
        ts = np.linspace(0.0, self.total_time, 50 * len(kfs) + 1)
        self.max_speed = max(float(np.linalg.norm(self.position(t, 1))) for t in ts)
        self.max_accel = max(float(np.linalg.norm(self.position(t, 2))) for t in ts)

    def done(self, t: float) -> bool:
        return t >= self.total_time

    def sample(self, t: float) -> TrajectoryPoint:
        # Past the end this holds the last keyframe at rest, because time is clamped
        return TrajectoryPoint(
            position=self.position(t, 0),
            velocity=self.position(t, 1),
            acceleration=self.position(t, 2),
            yaw=float(self.yaw(t, 0)[0]),
        )
