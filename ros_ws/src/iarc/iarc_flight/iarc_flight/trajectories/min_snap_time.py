"""Time allocation for min snap.

Solves the shortest leg times T for the min-snap path through given points such that each
leg i stays under its v_max[i] and a_max[i]

SLSQP (scipy) over T with limits.
K(T) z = rhs, so dz/dT_j = -K^-1 (dK/dT_j) z, where each entry of K scales
as a power of one T_j. If SLSQP fails, the initial guess is used.

This problem has
 - few variables
 - many (mostly inactive) ineq constraints, a dense Jacobian
 - exact gradients
 - a good initial guess
SLSQP's dense QP subproblem handles that well. SNOPT via Drake gave the same T but was slow
"""

from __future__ import annotations

import contextlib
import math
import time
from dataclasses import dataclass

import numpy as np
from scipy.linalg import lu_factor, lu_solve
from scipy.optimize import minimize

from iarc_flight.trajectories.min_snap import min_derivative_kkt, solve_min_derivative, unwrap_yaws

try:
    from threadpoolctl import threadpool_limits
except ImportError:  # optional; planning is just slower without it
    threadpool_limits = None

R, N = 4, 8  # snap derivative order, polynomial degree
SAMPLES = 12  # limit checks per leg (24 is ~2x slower for a ~0.5% shorter path)
T_MIN = 0.1  # s


def _basis(s: np.ndarray, m: int) -> np.ndarray:
    """(len(s), N): basis @ coeffs is the m-th s-derivative at s."""
    return np.array(
        [[math.perm(i, m) * x ** (i - m) if i >= m else 0 for i in range(N)] for x in s]
    )


def _derivs(c: np.ndarray, T: np.ndarray, s: np.ndarray, m: int) -> np.ndarray:
    """m-th time derivative at samples s of each leg, (M, S, D)."""
    return np.einsum("sn,knd->ksd", _basis(s, m), c) / T[:, None, None] ** m


def _limit_ratios(w, T, v_max, a_max, s):
    """Limit ratios at samples s of every leg, and their gradients with respect to T.

    g (2, M, S): g[0] = |v|^2 / v_max^2, g[1] = |a|^2 / a_max^2; the limits hold where g <= 1.
    dg (2, M, S, M): dg[..., j] = dg / dT_j.

    Gradient by implicit differentiation of K(T) z = rhs (rhs does not depend on T):
        dz/dT_j = -K^-1 (dK/dT_j) z
    Only leg j's entries of K depend on T_j, each as a power of T_j, so on those entries
    dK/dT_j = (power / T_j) K:
        cost block 2H_j            ~ T_j^(1-2R)
        order-m constraint entries ~ T_j^(-m)
    """
    M, D = len(T), w.shape[1]
    n = M * N  # polynomial coefficients; the rest of z is the Lagrange multipliers
    K, rhs, order = min_derivative_kkt(w, T, R)
    lu = lu_factor(K)  # factor once, reused for z and all M gradient solves
    z = lu_solve(lu, rhs)
    c, lam = z[:n], z[n:]
    A = K[n:, :n]
    power = -order[:, None]  # each constraint row scales as T^-order

    # dKz[..., j] = (dK/dT_j) z, from leg j's blocks only
    dKz = np.zeros((len(z), D, M))
    for j in range(M):
        leg = slice(j * N, (j + 1) * N)
        H2_j, A_j = K[leg, leg], A[:, leg]
        dKz[leg, :, j] = ((1 - 2 * R) * H2_j @ c[leg] + A_j.T @ (power * lam)) / T[j]
        dKz[n:, :, j] = power * (A_j @ c[leg]) / T[j]

    dz = -lu_solve(lu, dKz.reshape(len(z), -1)).reshape(len(z), D, M)
    dc = dz[:n].reshape(M, N, D, M)  # dc[..., j] = d coeffs / dT_j
    c = c.reshape(M, N, D)

    g, dg = [], []
    for m, lim in ((1, v_max), (2, a_max)):  # m = 1: velocity, m = 2: acceleration
        X = _derivs(c, T, s, m)  # (M, S, D)
        # X_k = B c_k / T_k^m: the coefficients move with every T_j, and T_k^-m adds
        # -m X_k / T_k when j == k
        dX = np.einsum("sn,kndj->ksdj", _basis(s, m), dc) / T[:, None, None, None] ** m
        dX[np.arange(M), :, :, np.arange(M)] -= m * X / T[:, None, None]
        g.append((X**2).sum(axis=2) / lim[:, None] ** 2)
        dg.append(2 * np.einsum("ksd,ksdj->ksj", X, dX) / lim[:, None, None] ** 2)
    return np.stack(g), np.stack(dg)


def segment_peaks(w: np.ndarray, T: np.ndarray, per_segment: int = 400) -> np.ndarray:
    """Peak |v| and |a| per leg on a dense grid, (2, M)."""
    c, s = solve_min_derivative(w, T, R), np.linspace(0.0, 1.0, per_segment)
    return np.array([np.linalg.norm(_derivs(c, T, s, m), axis=2).max(1) for m in (1, 2)])


def _stretch(w: np.ndarray, T: np.ndarray, v_max: np.ndarray, a_max: np.ndarray) -> float:
    """Uniform factor on T that puts the worst leg exactly on its limit."""
    v, a = segment_peaks(w, T)
    return float(max(np.max(v / v_max), np.sqrt(np.max(a / a_max))))


@dataclass
class TimeAllocation:
    durations: np.ndarray  # (M,) s
    success: bool  # SLSQP converged
    status: str
    stretch: float  # factor applied after SLSQP (1 = already within limits)
    peak_speed: np.ndarray  # (M,) per leg, dense grid
    peak_accel: np.ndarray
    solve_time: float  # s

    @property
    def total_time(self) -> float:
        return float(self.durations.sum())


def optimize_times(points, v_max, a_max, yaw_rate_max: float | None = None) -> TimeAllocation:
    """points (M+1, 4) of x, y, z, yaw; v_max, a_max scalars or (M,), leg i is points[i] -> [i+1]."""
    t0 = time.perf_counter()
    # Limit BLAS to one thread (~3x faster for small solves on my laptop)
    blas = threadpool_limits(1, user_api="blas") if threadpool_limits else contextlib.nullcontext()
    with blas:
        points = np.asarray(points, float)
        w, M = points[:, :3], len(points) - 1
        v_max, a_max = (np.broadcast_to(np.asarray(x, float), (M,)) for x in (v_max, a_max))
        lo = np.full(M, T_MIN)
        if yaw_rate_max:
            lo = np.maximum(lo, np.abs(np.diff(unwrap_yaws(list(points[:, 3])))) / yaw_rate_max)

        # T_0 is safe initial guess off of distance
        dist = np.linalg.norm(np.diff(w, axis=0), axis=1)
        T0 = np.maximum(lo, np.maximum(1.5 * dist / v_max, np.sqrt(6.0 * dist / a_max)))
        T0 = np.maximum(lo, T0 * _stretch(w, T0, v_max, a_max))

        s = np.linspace(0.0, 1.0, SAMPLES)

        # SLSQP asks for the constraint and its Jacobian separately at the same T, so cache the
        # last evaluation (one KKT factorization gives both)
        last: dict = {}

        def ratios(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            if last.get("T") is None or not np.array_equal(last["T"], T):
                g, dg = _limit_ratios(w, T, v_max, a_max, s)
                last.update(T=T.copy(), g=g.ravel(), dg=dg.reshape(-1, M))
            return last["g"], last["dg"]

        res = minimize(
            lambda T: T.sum(),  # same minimizer as the total time squared, better scaled
            T0,
            jac=lambda T: np.ones(M),
            method="SLSQP",
            bounds=[(lb, None) for lb in lo],
            constraints=[  # SLSQP wants fun >= 0: 1 - g >= 0 where the limits hold
                {
                    "type": "ineq",
                    "fun": lambda T: 1.0 - ratios(T)[0],
                    "jac": lambda T: -ratios(T)[1],
                }
            ],
            options={"ftol": 1e-6, "maxiter": 200},
        )

        ok, status = bool(res.success), str(res.message)
        Tf = np.maximum(lo, res.x) if ok else T0
        Tf *= (k := max(1.0, _stretch(w, Tf, v_max, a_max)))
        v, a = segment_peaks(w, Tf)
    return TimeAllocation(Tf, ok, status, k, v, a, time.perf_counter() - t0)
