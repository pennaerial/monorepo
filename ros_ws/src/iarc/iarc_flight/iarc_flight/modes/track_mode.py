import multiprocessing as mp
import os
import signal
import threading
import time
from concurrent.futures import Future, ProcessPoolExecutor
from typing import Annotated, Literal, override

import numpy as np
from pydantic import Field
from rclpy.node import Node
from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from iarc_flight.controllers.feedforward import FeedforwardConfig
from iarc_flight.controllers.mpc import MPCConfig
from iarc_flight.runtime.gps import RtkMonitor
from iarc_flight.runtime.stop import StopPlanner, StopTrigger
from iarc_flight.trajectories.base import Trajectory
from iarc_flight.trajectories.circle import CircleConfig
from iarc_flight.trajectories.min_snap import MinSnapConfig

ControllerConfig = Annotated[FeedforwardConfig | MPCConfig, Field(discriminator="type")]
TrajectoryConfig = Annotated[CircleConfig | MinSnapConfig, Field(discriminator="type")]


_planner: ProcessPoolExecutor | None = None  # shared by every trackmode in the mission


def planner() -> ProcessPoolExecutor:
    """One worker process that builds trajectories, started (and its imports loaded) on first use."""
    global _planner
    if _planner is None:
        _planner = ProcessPoolExecutor(
            max_workers=1, mp_context=mp.get_context("spawn"), initializer=_planner_init
        )
        _planner.submit(int)  # start the worker now, so its ~1 s of imports happens at mission load
    return _planner


def _planner_init() -> None:
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Ignore Ctrl-C in the worker,
    # The node's pid as the node recorded it: os.getppid() here could already be init's if the
    # node died while this worker was still starting up
    parent = mp.parent_process()
    parent = parent.pid if parent is not None else os.getppid()

    def exit_with_parent() -> None:
        while os.getppid() == parent:
            time.sleep(1.0)
        os._exit(0)

    threading.Thread(target=exit_with_parent, daemon=True).start()
    import iarc_flight.trajectories.min_snap_time  # noqa: F401  preload scipy.optimize


def timed_build(
    config: "TrajectoryConfig", start: np.ndarray, yaw: float
) -> tuple[Trajectory, float]:
    """Builds the trajectory (in the planner process) and returns it with the build's wall time."""
    t0 = time.perf_counter()
    traj = config.build(start, yaw)
    return traj, time.perf_counter() - t0


class TrackParams(ParamsBase):
    trajectory: TrajectoryConfig
    controller: ControllerConfig = FeedforwardConfig()
    # What brakes it at once and returns `stop` (map it to an iarc_flight.StopMode)
    stop_on: list[Literal["service", "rtk_loss"]] = []
    gps_topic: str | None = None  # None = PX4's vehicle_gps_position
    gps_timeout: float = Field(default=1.0, gt=0)  # s


@register_mode(
    id="iarc_flight.TrackMode",
    params_cls=TrackParams,
    targets=[UAV],
    transition_labels=["complete", "stop"],
)
class TrackMode(Mode[UAV, TrackParams]):
    """Flies a trajectory with a controller, both picked by `type` in the mission."""

    @override
    def initialize(self, node: Node, vehicle: UAV, params: TrackParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.params = params
        # Built at startup so the MPC's slow first solve isn't mid-flight
        self.controller = params.controller.build()
        self.trajectory = None  # placed at the drone on the first update
        # Planning can take a while (time_optimal min snap runs SLSQP), so it runs in another
        # process while the drone holds where tracking started
        planner()
        self.planning: tuple[Future, np.ndarray, float] | None = None  # (build, hold pos, yaw)
        self.plan_start = 0.0  # perf_counter when planning began
        self.plan_time = 0.0  # s spent building the trajectory
        self.t = 0.0
        self.errors: list[float] = []
        self.step_ms: list[float] = []
        self.speeds: list[float] = []
        self.trigger = StopTrigger.get(node) if params.stop_on else None
        if "rtk_loss" in params.stop_on:
            self.rtk = RtkMonitor.get(node, vehicle, params.gps_topic, params.gps_timeout)

    @override
    def on_enter(self) -> None:
        self.stop_planner = None
        if self.trigger is not None:
            self.stop_planner = StopPlanner.existing(self.node)  # configured by the StopMode
            self.trigger.arm(self.params.stop_on)
            if "rtk_loss" in self.params.stop_on:
                self.rtk.watch(True)  # not RTK fixed on entry stops right away

    @property
    def stopping(self) -> bool:
        return self.trigger is not None and self.trigger.fired is not None

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

        # Stopping: brake until the `stop` transition, never track again
        if self.stopping:
            self.stop_planner.publish(self.stop_planner.step(state))
            return

        # First update plans the trajectory from the drone, holding until it is ready; later
        # updates advance time along it
        if self.trajectory is None:
            if self.planning is None:
                yaw = self.vehicle.yaw if self.vehicle.yaw is not None else pos.heading
                hold = state[:3].copy()
                self.plan_start = time.perf_counter()
                build = planner().submit(timed_build, self.params.trajectory, hold, yaw)
                self.planning = (build, hold, yaw)
            build, hold, hold_yaw = self.planning
            if not build.done():
                self.vehicle.publish_trajectory_setpoint(hold, np.zeros(3), np.zeros(3), hold_yaw)
                self.step_stop(state)
                return
            self.trajectory, self.plan_time = build.result()  # re-raises a planning error
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
        self.step_stop(state)

    def step_stop(self, state: np.ndarray) -> None:
        """Keep a stop setpoint from this state ready for a trigger to send immediately."""
        if self.stop_planner is not None:
            self.stop_planner.step(state)

    def log_plan(self) -> None:
        held = time.perf_counter() - self.plan_start
        self.log(
            f"Tracking {self.params.trajectory.type} with {self.params.controller.type}, "
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
        if self.trigger is not None:
            self.trigger.disarm()
        if "rtk_loss" in self.params.stop_on:
            self.rtk.watch(False)
        if self.errors:
            e = np.array(self.errors)
            self.log(
                f"Tracking error rms {np.sqrt(np.mean(e**2)):.2f} m, max {e.max():.2f} m, "
                f"controller step max {max(self.step_ms):.1f} ms, flown max speed {max(self.speeds):.2f} m/s"
            )

    @override
    def check_status(self) -> str:
        if self.stopping:
            return "stop"
        done = self.trajectory is not None and self.trajectory.done(self.t)
        return "complete" if done else "continue"
