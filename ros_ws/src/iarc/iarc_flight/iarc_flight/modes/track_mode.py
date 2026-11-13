import time
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Annotated, override

import numpy as np
from pydantic import Field
from rclpy.node import Node
from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from iarc_flight.controllers.feedforward import FeedforwardConfig
from iarc_flight.controllers.mpc import MPCConfig
from iarc_flight.trajectories.base import Trajectory
from iarc_flight.trajectories.circle import CircleConfig
from iarc_flight.trajectories.min_snap import MinSnapConfig

ControllerConfig = Annotated[FeedforwardConfig | MPCConfig, Field(discriminator="type")]
TrajectoryConfig = Annotated[CircleConfig | MinSnapConfig, Field(discriminator="type")]


class TrackParams(ParamsBase):
    trajectory: TrajectoryConfig
    controller: ControllerConfig = FeedforwardConfig()


@register_mode(
    id="iarc_flight.TrackMode",
    params_cls=TrackParams,
    targets=[UAV],
    transition_labels=["complete"],
)
class TrackMode(Mode[UAV, TrackParams]):
    """Flies a trajectory with a controller, both picked by `type` in the mission."""

    @override
    def initialize(self, node: Node, vehicle: UAV, params: TrackParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params
        # Built at startup so the MPC's slow first solve isn't mid-flight
        self.controller = params.controller.build()
        self.trajectory = None  # placed at the drone on the first update
        # Planning can take a while (time_optimal min snap runs SNOPT), so it runs off the control
        # loop while the drone holds where tracking started
        self.planning: Future | None = None
        self.hold: tuple[np.ndarray, float] | None = None
        self.plan_start = 0.0  # perf_counter when planning began
        self.plan_time = 0.0  # s spent building the trajectory
        self.t = 0.0
        self.errors: list[float] = []
        self.step_ms: list[float] = []
        self.speeds: list[float] = []

    @override
    def on_update(self, time_delta: float) -> None:
        pos = self.vehicle.local_position
        if pos is None:
            self.node.get_logger().warning(
                "TrackMode not updating: no valid position", throttle_duration_sec=1.0
            )
            return
        # Local NED position and velocity
        state = np.array([pos.x, pos.y, pos.z, pos.vx, pos.vy, pos.vz])

        # First update plans the trajectory from the drone, holding until it is ready; later
        # updates advance time along it
        if self.trajectory is None:
            if self.planning is None:
                yaw = self.vehicle.yaw if self.vehicle.yaw is not None else pos.heading
                self.hold = (state[:3].copy(), yaw)
                pool = ThreadPoolExecutor(max_workers=1)  # plan on seperate thread
                self.plan_start = time.perf_counter()
                self.planning = pool.submit(self.timed_build, self.hold[0], yaw)
                pool.shutdown(wait=False)  # the worker exits once the build returns
            if not self.planning.done():
                assert self.hold is not None
                self.vehicle.publish_trajectory_setpoint(
                    self.hold[0], np.zeros(3), np.zeros(3), self.hold[1]
                )
                return
            self.trajectory, self.plan_time = self.planning.result()  # re-raises a planning error
            self.log_plan()
        else:
            self.t += time_delta

        # Ask the controller for a setpoint, timing it and recording distance from the ideal path
        start = time.perf_counter()
        setpoint = self.controller.step(state, self.trajectory, self.t)
        self.step_ms.append((time.perf_counter() - start) * 1000.0)
        planned = self.trajectory.sample(self.t).position
        self.errors.append(np.linalg.norm(state[:3] - planned))
        self.speeds.append(float(np.linalg.norm(state[3:])))

        self.vehicle.publish_trajectory_setpoint(
            setpoint.position, setpoint.velocity, setpoint.acceleration, setpoint.yaw
        )

    def timed_build(self, start: np.ndarray, yaw: float) -> tuple[Trajectory, float]:
        """Builds the trajectory and returns it with the build's wall time in seconds."""
        t0 = time.perf_counter()
        traj = self.p.trajectory.build(start, yaw)
        return traj, time.perf_counter() - t0

    def log_plan(self) -> None:
        held = time.perf_counter() - self.plan_start
        self.log(
            f"Tracking {self.p.trajectory.type} with {self.p.controller.type}, "
            f"path generated in {self.plan_time * 1000:.0f} ms (held {held * 1000:.0f} ms)"
        )
        alloc = getattr(self.trajectory, "time_allocation", None)
        if alloc is not None:
            self.log(
                f"Time optimal: {alloc.status} in {alloc.solve_time * 1000:.0f} ms, "
                f"stretch {alloc.stretch:.3f}, "
                f"T = [{', '.join(f'{T:.2f}' for T in alloc.durations)}]"
            )
        if hasattr(self.trajectory, "max_speed"):
            self.log(
                f"Planned {self.trajectory.total_time:.1f} s, max speed {self.trajectory.max_speed:.2f} m/s, "
                f"max accel {self.trajectory.max_accel:.2f} m/s^2"
            )

    @override
    def on_exit(self) -> None:
        if self.errors:
            e = np.array(self.errors)
            self.log(
                f"Tracking error rms {np.sqrt(np.mean(e**2)):.2f} m, max {e.max():.2f} m, "
                f"controller step max {max(self.step_ms):.1f} ms, flown max speed {max(self.speeds):.2f} m/s"
            )

    @override
    def check_status(self) -> str:
        done = self.trajectory is not None and self.trajectory.done(self.t)
        return "complete" if done else "continue"
