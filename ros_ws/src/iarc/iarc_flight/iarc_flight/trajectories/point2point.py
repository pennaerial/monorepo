
import numpy as np
import cvxpy as cp
from scipy.interpolate import BSpline
from numpy.polynomial.legendre import leggauss


class PointToPointTrajectory:
    def __init__(self, start, goal, T, control_points, knots, degree):
        self.T = T
        self.spline = BSpline(knots, control_points, degree)

    def sample(self, t):
        # Convert real time to normalized spline time
        s = np.clip(t / self.T, 0.0, 1.0)

        p = self.spline(s)
        v = self.spline.derivative(1)(s) / self.T
        a = self.spline.derivative(2)(s) / self.T**2

        return p, v, a


class PointToPointPlanner:
    def __init__(self, vmax=3.0, amax=2.0, jmax=5.0):
        self.vmax = vmax
        self.amax = amax
        self.jmax = jmax
        self.degree = 5
        self.n = 8

        # Clamped quintic spline with 8 control points
        d, n = self.degree, self.n
        interior = np.linspace(0, 1, n - d + 1)[1:-1]

        self.knots = np.r_[
            np.zeros(d + 1),
            interior,
            np.ones(d + 1)
        ]

        # Basis functions only; control points remain unknown
        self.basis = BSpline(
            self.knots, np.eye(n), d
        )

    def B(self, s, order, T):
        # Linear map from control points to derivative at time T*s
        return self.basis.derivative(order)(s) / T**order

    def solve_for_duration(self, start, goal, v0, T):
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        v0 = np.asarray(v0, dtype=float)

        # All n XYZ control points are solved together
        P = cp.Variable((self.n, 3))

        # Initial and final position, velocity, acceleration
        constraints = [
            self.B(0, 0, T) @ P == start,
            self.B(1, 0, T) @ P == goal,
            self.B(0, 1, T) @ P == v0,
            self.B(1, 1, T) @ P == 0,
            self.B(0, 2, T) @ P == 0,
            self.B(1, 2, T) @ P == 0,
        ]

        # Bound derivative spline control points.
        # This implies continuous-time component-wise bounds.
        for order, limit in [
            (1, self.vmax),
            (2, self.amax),
            (3, self.jmax),
        ]:
            derived = self.basis.derivative(order)
            n_derived = len(derived.t) - derived.k - 1

            D = derived.c[:n_derived] / T**order
            derivative_control_points = D @ P

            constraints += [
                derivative_control_points <= limit,
                derivative_control_points >= -limit,
            ]

        # Integrated squared jerk.
        # 3-point Gaussian quadrature is exact here because
        # jerk is piecewise quadratic for a quintic spline.
        nodes, weights = leggauss(3)
        unique_knots = np.unique(self.knots)
        cost = 0

        for left, right in zip(
            unique_knots[:-1], unique_knots[1:]
        ):
            half = (right - left) / 2
            center = (right + left) / 2

            for node, weight in zip(nodes, weights):
                s = center + half * node
                jerk = self.B(s, 3, T) @ P

                # Convert integral over s to real time
                cost += (
                    T * half * weight
                    * cp.sum_squares(jerk)
                )

        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP)

        if problem.status not in (
            cp.OPTIMAL, cp.OPTIMAL_INACCURATE
        ):
            return None

        return PointToPointTrajectory(
            start, goal, T, P.value,
            self.knots, self.degree
        )

    def plan(self, start, goal, v0, tolerance=0.05):
        # Find a feasible upper duration bound
        low, high = 0.25, 2.0

        solution = self.solve_for_duration(
            start, goal, v0, high
        )

        while solution is None and high < 128:
            high *= 2
            solution = self.solve_for_duration(
                start, goal, v0, high
            )

        if solution is None:
            raise RuntimeError("No feasible trajectory found")

        # Refine shortest feasible duration
        for _ in range(20):
            if high - low <= tolerance:
                break

            mid = (low + high) / 2
            candidate = self.solve_for_duration(
                start, goal, v0, mid
            )

            if candidate is None:
                low = mid
            else:
                high = mid
                solution = candidate

        return solution


if __name__ == "__main__":
    planner = PointToPointPlanner()

    trajectory = planner.plan(
        start=[0, 0, 0],
        goal=[10, 0, 0],
        v0=[0, 0, 0],
    )

    print("Flight duration:", trajectory.T)

    for t in np.linspace(0, trajectory.T, 6):
        p, v, a = trajectory.sample(t)
        print(f"t={t:.2f}: p={p}, v={v}, a={a}")
