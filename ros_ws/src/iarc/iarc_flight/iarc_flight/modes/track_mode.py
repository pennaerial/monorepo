import time
from concurrent.futures import Future, ThreadPoolExecutor, wait
from typing import Annotated, override

import numpy as np
from pydantic import Field
from rclpy.node import Node
from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from iarc_flight.controllers.feedforward import FeedforwardConfig
from iarc_flight.controllers.mpc import MPCConfig
from iarc_flight.trajectories.circle import CircleConfig
from iarc_flight.trajectories.min_snap import MinSnapConfig
from iarc_flight.trajectories.min_time import MinTimeConfig

ControllerConfig = Annotated[FeedforwardConfig | MPCConfig, Field(discriminator="type")]
TrajectoryConfig = Annotated[
    CircleConfig | MinSnapConfig | MinTimeConfig, Field(discriminator="type")
]


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
        # Building can take a while (min_time solves an optimization), so it runs off the
        # update thread while the drone holds position. PX4 leaves offboard after ~1 s
        # without setpoints, which a blocking solve could trigger.
        self._builder = ThreadPoolExecutor(max_workers=1)
        self._build: Future | None = None
        self._hold: tuple[np.ndarray, float] | None = None
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

        # First update starts building the trajectory at the drone. Until it's ready, hold
        # position; after that, advance time along it.
        if self.trajectory is None:
            if self._build is None:
                yaw = self.vehicle.yaw if self.vehicle.yaw is not None else pos.heading
                self._hold = (state[:3].copy(), yaw)
                self._build_start = time.perf_counter()
                self._build = self._builder.submit(self.p.trajectory.build, state[:3].copy(), yaw)
                wait([self._build], timeout=0.05)  # quick builds (circle, min_snap) start this tick
            if not self._build.done():
                hold_pos, hold_yaw = self._hold
                self.vehicle.publish_trajectory_setpoint(
                    hold_pos, np.zeros(3), np.zeros(3), hold_yaw
                )
                return
            self.trajectory = self._build.result()  # re-raises if the build failed
            build_ms = (time.perf_counter() - self._build_start) * 1000.0
            self.log(
                f"Tracking {self.p.trajectory.type} with {self.p.controller.type} "
                f"(built in {build_ms:.0f} ms while holding)"
            )
            for line in getattr(self.trajectory, "summary", []):
                self.log(line)
            if hasattr(self.trajectory, "max_speed"):
                self.log(
                    f"Planned {self.trajectory.total_time:.1f} s, max speed {self.trajectory.max_speed:.2f} m/s, "
                    f"max accel {self.trajectory.max_accel:.2f} m/s^2"
                )
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

    @override
    def on_exit(self) -> None:
        self._builder.shutdown(wait=False, cancel_futures=True)
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
