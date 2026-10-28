import numpy as np
import cvxpy as cp
from scipy.interpolate import make_interp_spline

# TODO: get these from params/yaml
# TODO: change dt and px4 rate to higher Hz maybe
dt = 0.1 # 10hz to match px4 trajectory setpoint rate
N = 20 # 2s horizon
g = 9.81
a_xy_max = g * np.tan(np.deg2rad(30)) # max horizontal accel from tilting 30 deg (~5.7 m/s^2)
a_z_max = 4.0 # max vertical accel [m/s^2]
v_max = 3.0 # max speed per axis [m/s]

I3 = np.eye(3)
Z3 = np.zeros((3, 3))
A = np.block([[I3, dt * I3], [Z3, I3]])
B = np.vstack([0.5 * dt**2 * I3, dt * I3])

# state: x = [px, py, pz, vx, vy, vz]
# control: u = [ax, ay, az]
x0 = cp.Parameter(6)
xref = cp.Parameter((6, N + 1))
x = cp.Variable((6, N + 1))
u = cp.Variable((3, N))

q_weights = np.array([10, 10, 10, 1, 1, 1]) # weights for pos & vel error in cost
r_weight = 0.1 # weight for control effort in cost
wq = np.sqrt(q_weights).reshape(6, 1)
wr = np.sqrt(r_weight)
cost = cp.sum_squares(cp.multiply(wq, x - xref)) + cp.sum_squares(wr * u)

constraints: list[cp.Constraint] = [x[:, 0] == x0]
for k in range(N):
    constraints += [
        x[:, k + 1] == A @ x[:, k] + B @ u[:, k], # dynamics
        cp.abs(u[0:2, k]) <= a_xy_max, # lateral accel limit
        cp.abs(u[2, k]) <= a_z_max, # vertical accel limit
        cp.abs(x[3:6, k + 1]) <= v_max, # velocity limit
    ]

#uses last good solution as warm start
problem = cp.Problem(cp.Minimize(cost), constraints)


def mpc_step(x_now, ref):
    """Returns (p1, v1, a0), or None"""
    x0.value = x_now
    xref.value = ref
    problem.solve(solver=cp.OSQP, warm_start=True)
    if (
        problem.status not in ("optimal", "optimal_inaccurate")
        or x.value is None 
        or u.value is None
    ):
        return None
    return x.value[0:3, 1], x.value[3:6, 1], u.value[:, 0]


def make_reference(spline, T, t_now):
    """Returns 6xN+1 ref trajectory for MPC"""
    t = t_now + dt * np.arange(N + 1)
    s = np.clip(t / T, 0.0, 1.0) # [0,1] paramed spline
    ref = np.zeros((6, N + 1))
    ref[0:3, :] = spline(s).T
    ref[3:6, :] = spline.derivative(1)(s).T / T # ds/dt = ds/dt * dx/ds = 1/T * dx/ds
    ref[3:6, t >= T] = 0.0 # stop at end of spline, hold pos
    return ref


def build_demo_spline(p0):
    """4-point demo path, offset to start at p0 (NED)"""
    pts = np.array([[0, 0, 0], [0, 0, -2], [4, 0, -2], [4, 3, -2.5]], dtype=float) + p0
    zero = np.zeros(3)
    return make_interp_spline(
        np.linspace(0, 1, len(pts)), pts, k=3, bc_type=([(1, zero)], [(1, zero)])
    )
