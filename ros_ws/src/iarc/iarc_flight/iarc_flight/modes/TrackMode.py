from __future__ import annotations

import time
from typing import Annotated, override

import numpy as np
from px4_msgs.msg import TrajectorySetpoint
from pydantic import Field
from rclpy.node import Node
from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from iarc_flight.controllers import FeedforwardConfig, MPCConfig
from iarc_flight.trajectories import CircleConfig, Trajectory


class TrackParams(ParamsBase):
    """
    trajectory: The path to fly, chosen by its `type`.
    controller: How to track it, chosen by its `type`.
    """

    trajectory: CircleConfig
    controller: Annotated[FeedforwardConfig | MPCConfig, Field(discriminator="type")] = (
        FeedforwardConfig()
    )


@register_mode(
    id="iarc_flight.TrackMode",
    params_cls=TrackParams,
    targets=[UAV],
    transition_labels=["complete"],
)
class TrackMode(Mode[UAV, TrackParams]):
    """Flies a trajectory with a controller, publishing a TrajectorySetpoint every update."""

    @override
    def initialize(self, node: Node, vehicle: UAV, params: TrackParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params
        self.controller = params.controller.build()
        # Placed on the first update, from the drone's position
        self.trajectory: Trajectory | None = None
        self.t = 0.0
        self.finished = False
        self.errors: list[float] = []
        self.solve_ms: list[float] = []

    @override
    def on_update(self, time_delta: float) -> None:
        pos = self.vehicle.local_position
        if pos is None:
            return
        state = np.array([pos.x, pos.y, pos.z, pos.vx, pos.vy, pos.vz], dtype=float)

        if self.trajectory is None:
            yaw = self.vehicle.yaw if self.vehicle.yaw is not None else pos.heading
            self.trajectory = self.p.trajectory.build(state[:3], yaw)
            self.log(
                f"Tracking {self.p.trajectory.type} with {self.p.controller.type} from {state[:3].round(2)}"
            )
        elif not self.finished:
            self.t += time_delta

        start = time.perf_counter()
        setpoint = self.controller.step(state, self.trajectory, self.t)
        self.solve_ms.append((time.perf_counter() - start) * 1e3)
        self.errors.append(
            float(np.linalg.norm(state[:3] - self.trajectory.sample(self.t).position))
        )

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x) for x in setpoint.position]
        msg.velocity = [float(x) for x in setpoint.velocity]
        msg.acceleration = [float(x) for x in setpoint.acceleration]
        msg.yaw = float(setpoint.yaw)
        self.vehicle.trajectory_publisher.publish(msg)

        if not self.finished and self.trajectory.done(self.t):
            self.finished = True
            errors = np.array(self.errors)
            self.log(
                f"Trajectory done in {self.t:.1f}s: position error rms {np.sqrt(np.mean(errors**2)):.2f}m, "
                f"max {errors.max():.2f}m, controller step mean {np.mean(self.solve_ms):.1f}ms "
                f"max {np.max(self.solve_ms):.1f}ms, failed solves {getattr(self.controller, 'failed_solves', 0)}"
            )

    @override
    def check_status(self) -> str:
        return "complete" if self.finished else "continue"
