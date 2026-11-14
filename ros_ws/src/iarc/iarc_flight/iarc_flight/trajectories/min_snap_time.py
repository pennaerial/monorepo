"""Time allocation for min snap.

Solves the shortest leg times T for the min-snap path through given points such that each
leg i stays under its v_max[i] and a_max[i]

SNOPT (via Drake) over T with limits.
K(T) z = rhs, so dz/dT_j = -K^-1 (dK/dT_j) z, where each entry of K scales
as a power of one T_j. If SNOPT fails, the initial guess is used.

This problem has
 - few variables
 - many (mostly inactive) ineq constraints
 - exact gradients
 - a good initial guess
which snopt (sparse sqp) is good for
"""

from __future__ import annotations

import contextlib
import math
import time
from dataclasses import dataclass

import numpy as np
from scipy.linalg import lu_factor, lu_solve

from iarc_flight.trajectories.min_snap import min_derivative_kkt, solve_min_derivative, unwrap_yaws

try:
    from threadpoolctl import threadpool_limits
except ImportError:  # optional; planning is just slower without it
    threadpool_limits = None

R, N = 4, 8  # snap derivative order, polynomial degree
SAMPLES = 12  # SNOPT limit checks per leg (24 is ~2x slower for a ~0.5% shorter path)
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
    """|v|^2/v_max^2 and |a|^2/a_max^2 at samples s, (2, M, S), and d/dT, (2, M, S, M)."""
    M, D = len(T), w.shape[1]
    K, rhs, order = min_derivative_kkt(w, T, R)
    lu = lu_factor(K)
    z = lu_solve(lu, rhs)
    c = z[: M * N]
    lam = z[M * N :]
    A = K[M * N :, : M * N]
    p = -order[:, None]

    # U[..., j] = (dK/dT_j) z, without forming dK/dT_j.
    dKz = np.zeros((len(z), D, M))
    for j in range(M):
        coeff_slice = slice(j * N, (j + 1) * N)
        dKz[coeff_slice, :, j] = (
            (1 - 2 * R) * K[coeff_slice, coeff_slice] @ c[coeff_slice]
            + A[:, coeff_slice].T @ (p * lam)
        ) / T[j]
        dKz[M * N :, :, j] = p * (A[:, coeff_slice] @ c[coeff_slice]) / T[j]

    dc = -lu_solve(lu, dKz.reshape(len(z), -1))
    dc = dc.reshape(len(z), D, M)[: M * N].reshape(M, N, D, M)
    c = c.reshape(M, N, D)
    g, dg = [], []

    # compute |v|^2/v_max^2 and |a|^2/a_max^2 and their gradients
    for m, lim in ((1, v_max), (2, a_max)):
        X = _derivs(c, T, s, m)
        dX = np.einsum("sn,kndj->ksdj", _basis(s, m), dc) / T[:, None, None, None] ** m
        dX[np.arange(M), :, :, np.arange(M)] -= m * X / T[:, None, None]  # from the T^-m factor
        g.append((X**2).sum(2) / lim[:, None] ** 2)
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
    success: bool  # SNOPT converged
    status: str
    stretch: float  # factor applied after SNOPT (1 = already within limits)
    peak_speed: np.ndarray  # (M,) per leg, dense grid
    peak_accel: np.ndarray
    solve_time: float  # s

    @property
    def total_time(self) -> float:
        return float(self.durations.sum())


def optimize_times(points, v_max, a_max, yaw_rate_max: float | None = None) -> TimeAllocation:
    """points (M+1, 4) of x, y, z, yaw; v_max, a_max scalars or (M,), leg i is points[i] -> [i+1]."""
    from pydrake.autodiffutils import ExtractGradient, ExtractValue, InitializeAutoDiff
    from pydrake.solvers import MathematicalProgram, SnoptSolver

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

        def limits(Tv: np.ndarray) -> np.ndarray:  # Drake passes floats or AutoDiffXd
            if Tv.dtype != object:
                return _limit_ratios(w, Tv, v_max, a_max, s)[0].ravel()
            g, dg = _limit_ratios(w, ExtractValue(Tv).ravel(), v_max, a_max, s)
            return InitializeAutoDiff(g.ravel(), dg.reshape(-1, M) @ ExtractGradient(Tv)).ravel()

        # Drake Solve provides access to https://ccom.ucsd.edu/~optimizers/solvers/snopt/
        prog = MathematicalProgram()
        T = prog.NewContinuousVariables(M, "T")
        prog.AddBoundingBoxConstraint(lo, np.full(M, np.inf), T)
        prog.AddCost(sum(T) ** 2)
        prog.AddConstraint(limits, np.full(2 * M * SAMPLES, -np.inf), np.ones(2 * M * SAMPLES), T)
        res = SnoptSolver().Solve(prog, T0)  # default tolerances

        ok, status = res.is_success(), str(res.get_solution_result())
        Tf = np.maximum(lo, res.GetSolution(T)) if ok else T0
        Tf *= (k := max(1.0, _stretch(w, Tf, v_max, a_max)))
        v, a = segment_peaks(w, Tf)
    return TimeAllocation(Tf, ok, status, k, v, a, time.perf_counter() - t0)
