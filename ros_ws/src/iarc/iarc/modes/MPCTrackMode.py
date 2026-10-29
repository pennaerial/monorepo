from __future__ import annotations

import time
from typing import override

import numpy as np
from px4_msgs.msg import TrajectorySetpoint
from rclpy.node import Node
from scipy.interpolate import make_interp_spline
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from iarc import mpc as mpc # linear mpc tracking bspline impl
from uav.vehicles.UAV import UAV


def demo_spline(p0):
    """Large horizontal figure-eight path, offset to start at p0 (NED)."""
    pts = np.array(
        [
            [0, 0, 0],
            [6, 4, 0],
            [12, 0, 0],
            [6, -4, 0],
            [0, 0, 0],
            [-6, 4, 0],
            [-12, 0, 0],
            [-6, -4, 0],
            [0, 0, 0],
        ],
        dtype=float,
    ) + p0
    zero = np.zeros(3)
    return make_interp_spline(
        np.linspace(0, 1, len(pts)), pts, k=3, bc_type=([(1, zero)], [(1, zero)])
    )

class MPCTrackParams(ParamsBase):
    """ duration (s) of bspline tracking, hold_time (s) to settle"""
    duration: float = 8.0
    hold_time: float = 1.0


@register_mode(
    id="iarc.MPCTrackMode",
    params_cls=MPCTrackParams,
    targets=[UAV],
    transition_labels=["complete"],
)
class MPCTrackMode(Mode[UAV, MPCTrackParams]):
    """Track a bspline with linear mpc\\
    publishing pos/vel/acc as px4_msgs/TrajectorySetpoint\\
    Using a time-paramed demo_spline
    """

    @override
    def initialize(self, node: Node, vehicle: UAV, params: MPCTrackParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params
        self.spline = None
        self.t_start = None
        self.hold_yaw = 0.0
        self.last_sp = None
        self.done = False
        self.tick = 0

        # first solve is expensive so do it now (a bit hacky)
        # TODO: find a better way to warmp up solver
        mpc.mpc_step(np.zeros(6), np.zeros((6, mpc.N + 1)))

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _publish(self, p, v, a) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float(c) for c in p]
        msg.velocity = [float(c) for c in v]
        #it seems like px4 ignores acceleration, but supposedly it can be used for feedforward control.
        # TODO: convince px4 to use acc for feedfwd (ros_ws/src/common/uav/uav/vehicles/UAV.py:291)
        msg.acceleration = [float(c) for c in a] 
        msg.jerk = [float("nan"), float("nan"), float("nan")] # not used by px4
        msg.yaw = self.hold_yaw
        msg.yawspeed = float("nan") # not used by px4
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000) # time in us
        self.vehicle.trajectory_publisher.publish(msg)

    @override
    def on_update(self, time_delta: float) -> None:
        lp = self.vehicle.local_position
        if lp is None:
            return

        # x_0 6dof state
        x_now = np.array([lp.x, lp.y, lp.z, lp.vx, lp.vy, lp.vz], dtype=float)

        if self.t_start is None:  # first tick in this mode, set spline to drone pos
            self.hold_yaw = float(self.vehicle.yaw if self.vehicle.yaw is not None else lp.heading)
            self.spline = demo_spline(x_now[:3])
            self.t_start = self._now()
            self.log(f"MPC track start at {x_now[:3]}, T={self.p.duration}s")

        t = self._now() - self.t_start
        ref = mpc.make_reference(self.spline, self.p.duration, t) # x_ref 6x(N+1) reference trajectory for mpc
        # TODO: add a check to ensure the reference is valid
        out, solve_ms = None, float("nan")
        if np.all(np.isfinite(x_now)):
            t0 = time.perf_counter()
            out = mpc.mpc_step(x_now, ref) # p1, v1, a0 = MPC (x_now, x_ref)
            solve_ms = (time.perf_counter() - t0) * 1e3

        if out is not None:
            self.last_sp = out
        if self.last_sp is None:  # nothing solved yet
            self.last_sp = (x_now[:3], np.zeros(3), np.zeros(3))
        self._publish(self.last_sp[0], self.last_sp[1], self.last_sp[2])  # on a failed solve this uses last good one
        # self._publish(ref[0:3, 1], ref[3:6, 1], np.zeros(3)) # skip the mpc

        self.tick += 1
        #log error
        if self.tick % 10 == 0:
            err = np.linalg.norm(x_now[:3] - ref[:3, 0])
            self.log(f"t={t:.2f}s pos_err={err:.3f}m solve={solve_ms:.1f}ms ok={out is not None}")

        if t > self.p.duration + self.p.hold_time:
            self.done = True

    def check_status(self) -> str:
        return "complete" if self.done else "continue"
