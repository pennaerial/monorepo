import time
from typing import Annotated, override

import numpy as np
from px4_msgs.msg import TrajectorySetpoint
from pydantic import Field
from rclpy.node import Node
from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from iarc_flight.controllers.feedforward import FeedforwardConfig
from iarc_flight.controllers.mpc import MPCConfig
from iarc_flight.trajectories.circle import CircleConfig

ControllerConfig = Annotated[FeedforwardConfig | MPCConfig, Field(discriminator="type")]


class TrackParams(ParamsBase):
    trajectory: CircleConfig
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
        self.t = 0.0
        self.errors: list[float] = []
        self.step_ms: list[float] = []

    @override
    def on_update(self, time_delta: float) -> None:
        pos = self.vehicle.local_position
        if pos is None:
            return
        state = np.array([pos.x, pos.y, pos.z, pos.vx, pos.vy, pos.vz])

        if self.trajectory is None:
            yaw = self.vehicle.yaw if self.vehicle.yaw is not None else pos.heading
            self.trajectory = self.p.trajectory.build(state[:3], yaw)
            self.log(f"Tracking {self.p.trajectory.type} with {self.p.controller.type}")
        else:
            self.t += time_delta

        start = time.perf_counter()
        setpoint = self.controller.step(state, self.trajectory, self.t)
        self.step_ms.append((time.perf_counter() - start) * 1e3)
        self.errors.append(np.linalg.norm(state[:3] - self.trajectory.sample(self.t).position))

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        msg.position = setpoint.position.tolist()
        msg.velocity = setpoint.velocity.tolist()
        msg.acceleration = setpoint.acceleration.tolist()
        msg.yaw = float(setpoint.yaw)
        self.vehicle.trajectory_publisher.publish(msg)

    @override
    def on_exit(self) -> None:
        if self.errors:
            e = np.array(self.errors)
            self.log(
                f"Tracking error rms {np.sqrt(np.mean(e**2)):.2f} m, max {e.max():.2f} m, "
                f"controller step max {max(self.step_ms):.1f} ms"
            )

    @override
    def check_status(self) -> str:
        done = self.trajectory is not None and self.trajectory.done(self.t)
        return "complete" if done else "continue"
