from __future__ import annotations

from typing import Literal

import numpy as np
from pydantic import BaseModel

from iarc_flight.controllers.base import Controller
from iarc_flight.trajectories.base import Trajectory, TrajectoryPoint


class MPCConfig(BaseModel):
    type: Literal["mpc"] = "mpc"
    dt: float = 0.1  # s, matches the 10 Hz mode update
    horizon: int = 20  # steps, so 2 s ahead
    max_tilt_deg: float = 30.0  # sets the horizontal accel limit
    max_vertical_accel: float = 4.0  # m/s^2
    max_speed: float = 3.0  # m/s per axis
    position_weight: float = 10.0
    velocity_weight: float = 1.0
    accel_weight: float = 0.1

    def build(self) -> MPCController:
        return MPCController(self)


class MPCController(Controller):
    """
    Linear MPC on a double integrator: state is position + velocity, input is acceleration.
    Plans `horizon` steps along the trajectory within the limits and sends PX4 the first step.
    """

    def __init__(self, c: MPCConfig) -> None:
        import cvxpy as cp  # only needed when a mission uses MPC

        self.c = c
        n, dt = c.horizon, c.dt
        eye, zero = np.eye(3), np.zeros((3, 3))
        A = np.block([[eye, dt * eye], [zero, eye]])
        B = np.vstack([0.5 * dt**2 * eye, dt * eye])

        self.x0 = cp.Parameter(6)
        self.ref = cp.Parameter((6, n + 1))
        self.x = cp.Variable((6, n + 1))
        self.u = cp.Variable((3, n))

        w = np.sqrt([c.position_weight] * 3 + [c.velocity_weight] * 3).reshape(6, 1)
        tracking = cp.sum_squares(cp.multiply(w, self.x - self.ref))
        cost = tracking + c.accel_weight * cp.sum_squares(self.u)
        a_xy_max = 9.81 * np.tan(np.deg2rad(c.max_tilt_deg))
        constraints = [self.x[:, 0] == self.x0]
        for k in range(n):
            constraints += [
                self.x[:, k + 1] == A @ self.x[:, k] + B @ self.u[:, k],
                cp.abs(self.u[:2, k]) <= a_xy_max,
                cp.abs(self.u[2, k]) <= c.max_vertical_accel,
                cp.abs(self.x[3:, k + 1]) <= c.max_speed,
            ]
        self.problem = cp.Problem(cp.Minimize(cost), constraints)
        self.solver = cp.OSQP

        # The first solve compiles the problem and is slow, so do it before flight
        self._solve(np.zeros(6), np.zeros((6, n + 1)))

    def _solve(self, state: np.ndarray, ref: np.ndarray) -> bool:
        self.x0.value, self.ref.value = state, ref
        self.problem.solve(solver=self.solver, warm_start=True)
        return self.problem.status in ("optimal", "optimal_inaccurate")

    def step(self, state: np.ndarray, trajectory: Trajectory, t: float) -> TrajectoryPoint:
        points = [trajectory.sample(t + k * self.c.dt) for k in range(self.c.horizon + 1)]
        ref = np.array([np.concatenate([p.position, p.velocity]) for p in points]).T
        if not np.all(np.isfinite(state)) or not self._solve(state, ref):
            return points[0]  # fall back to the raw reference
        x1 = self.x.value[:, 1]  # planned state one step ahead
        return TrajectoryPoint(x1[:3], x1[3:], self.u.value[:, 0], points[0].yaw)
