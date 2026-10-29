from __future__ import annotations

from typing import Literal

import numpy as np
from pydantic import BaseModel

from iarc_flight.trajectories import Trajectory, TrajectoryPoint

from .base import Controller

G = 9.81


class MPCConfig(BaseModel):
    """
    dt: Prediction step in seconds. Matches the 10 Hz mode update rate.
    horizon: Number of prediction steps (horizon * dt seconds ahead).
    max_tilt_deg: Tilt limit that sets the horizontal acceleration limit, degrees.
    max_vertical_accel: Vertical acceleration limit, m/s^2.
    max_speed: Speed limit per axis, m/s.
    position_weight, velocity_weight, accel_weight: Cost weights on position error, velocity error and acceleration.
    """

    type: Literal["mpc"] = "mpc"
    dt: float = 0.1
    horizon: int = 20
    max_tilt_deg: float = 30.0
    max_vertical_accel: float = 4.0
    max_speed: float = 3.0
    position_weight: float = 10.0
    velocity_weight: float = 1.0
    accel_weight: float = 0.1

    def build(self) -> MPCController:
        return MPCController(self)


class MPCController(Controller):
    """
    Linear MPC on a double integrator: state [position, velocity], input acceleration.

    Each step it plans `horizon` steps ahead against the trajectory, within the acceleration and
    speed limits, and sends PX4 the first step of the plan. If a solve fails it falls back to the
    trajectory's reference.
    """

    def __init__(self, config: MPCConfig) -> None:
        import cvxpy as cp  # imported here so missions without MPC don't need cvxpy

        self.c = config
        self.failed_solves = 0
        n, dt = config.horizon, config.dt

        I3, Z3 = np.eye(3), np.zeros((3, 3))
        A = np.block([[I3, dt * I3], [Z3, I3]])
        B = np.vstack([0.5 * dt**2 * I3, dt * I3])

        self.x0 = cp.Parameter(6)
        self.xref = cp.Parameter((6, n + 1))
        self.x = cp.Variable((6, n + 1))
        self.u = cp.Variable((3, n))

        w = np.sqrt([config.position_weight] * 3 + [config.velocity_weight] * 3).reshape(6, 1)
        cost = cp.sum_squares(
            cp.multiply(w, self.x - self.xref)
        ) + config.accel_weight * cp.sum_squares(self.u)

        a_xy_max = G * np.tan(np.deg2rad(config.max_tilt_deg))
        constraints = [self.x[:, 0] == self.x0]
        for k in range(n):
            constraints += [
                self.x[:, k + 1] == A @ self.x[:, k] + B @ self.u[:, k],
                cp.abs(self.u[0:2, k]) <= a_xy_max,
                cp.abs(self.u[2, k]) <= config.max_vertical_accel,
                cp.abs(self.x[3:6, k + 1]) <= config.max_speed,
            ]
        self.problem = cp.Problem(cp.Minimize(cost), constraints)
        self.solver = cp.OSQP

        # The first solve compiles the problem and is slow, so do it now instead of in flight
        self._solve(np.zeros(6), np.zeros((6, n + 1)))

    def _solve(self, state: np.ndarray, ref: np.ndarray) -> bool:
        self.x0.value = state
        self.xref.value = ref
        self.problem.solve(solver=self.solver, warm_start=True)
        return self.problem.status in ("optimal", "optimal_inaccurate") and self.u.value is not None

    def step(self, state: np.ndarray, trajectory: Trajectory, t: float) -> TrajectoryPoint:
        points = [trajectory.sample(t + k * self.c.dt) for k in range(self.c.horizon + 1)]
        ref = np.array([np.concatenate([p.position, p.velocity]) for p in points]).T

        if not np.all(np.isfinite(state)) or not self._solve(state, ref):
            self.failed_solves += 1
            return points[0]
        return TrajectoryPoint(
            self.x.value[0:3, 1], self.x.value[3:6, 1], self.u.value[:, 0], points[0].yaw
        )
