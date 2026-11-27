from __future__ import annotations

from collections.abc import Iterable
from typing import Any

import numpy as np
from std_srvs.srv import Trigger

from iarc_flight.controllers.mpc import MPCConfig
from iarc_flight.runtime import per_node
from iarc_flight.trajectories.base import TrajectoryPoint
from iarc_flight.trajectories.hold import Hold


class StopMPCConfig(MPCConfig):
    """MPCConfig with defaults for stopping: track zero velocity and hold only the altitude, so each
    0.5 s plan brakes as hard as the limits allow."""

    horizon: int = 5  # steps of 0.1 s; longer stops carry through over several plans
    max_speed: float = 20.0  # m/s per axis, above any entry speed or the first step is infeasible
    position_weights: tuple[float, float, float] | None = (0.0, 0.0, 5.0)
    accel_weight: float = 0.05


class StopPlanner:
    """The stop MPC, configured by StopMode at mission load. A stoppable TrackMode
    steps it every update alongside tracking, StopMode then keeps stepping it."""

    @classmethod
    def configure(cls, node: Any, vehicle: Any, config: StopMPCConfig) -> StopPlanner:
        planner = per_node(node, "stop_planner", lambda: cls(vehicle, config))
        if planner.config != config:
            raise ValueError("every StopMode in a mission must use the same stop `controller`")
        return planner

    @classmethod
    def existing(cls, node: Any) -> StopPlanner | None:
        return per_node(node, "stop_planner")

    def __init__(self, vehicle: Any, config: StopMPCConfig) -> None:
        self.vehicle = vehicle
        self.config = config
        self.controller = config.build()  # at mission load, so the slow first solve isn't in flight
        self.latest: TrajectoryPoint | None = None
        self.hold: tuple[float, float] | None = None  # (z, yaw) once engaged

    @property
    def engaged(self) -> bool:
        return self.hold is not None

    def yaw(self) -> float:
        yaw = self.vehicle.yaw
        return yaw if yaw is not None else self.vehicle.local_position.heading

    def step(self, state: np.ndarray) -> TrajectoryPoint:
        z, yaw = self.hold if self.hold is not None else (state[2], self.yaw())
        self.latest = self.controller.step(state, Hold(np.array([state[0], state[1], z]), yaw), 0.0)
        return self.latest

    def publish(self, sp: TrajectoryPoint) -> None:
        self.vehicle.publish_trajectory_setpoint(sp.position, sp.velocity, sp.acceleration, sp.yaw)

    def engage(self) -> None:
        """Brake now: freeze the height and yaw and send the latest stop setpoint (or a fresh one)."""
        if self.engaged:
            return
        pos = self.vehicle.local_position
        state = np.array([pos.x, pos.y, pos.z, pos.vx, pos.vy, pos.vz])
        if self.latest is None:
            self.step(state)
        self.hold = (float(pos.z), self.yaw())
        self.publish(self.latest)

    def reset(self) -> None:
        self.latest, self.hold = None, None


class StopTrigger:
    """Ends tracking. Sources: the `mpc_stop` service and the RTK monitor."""

    @classmethod
    def get(cls, node: Any) -> StopTrigger:
        return per_node(node, "stop_trigger", lambda: cls(node))

    @classmethod
    def existing(cls, node: Any) -> StopTrigger | None:
        return per_node(node, "stop_trigger")

    def __init__(self, node: Any) -> None:
        self.node = node
        self.sources: set[str] = set()
        self.fired: str | None = None  # the reason, while stopping
        self.service = node.create_service(Trigger, "mpc_stop", self.on_service)

    def arm(self, sources: Iterable[str]) -> None:
        self.sources, self.fired = set(sources), None

    def disarm(self) -> None:
        self.sources = set()

    def reset(self) -> None:
        self.fired = None

    def fire(self, source: str, reason: str) -> tuple[bool, str]:
        if self.fired is not None:
            return True, "Already stopping"
        if source not in self.sources:
            return False, f"No active mode stops on {source}"
        self.fired = reason
        self.node.get_logger().warning(f"MPC stop: {reason}")
        planner = StopPlanner.existing(self.node)
        if planner is not None:
            planner.engage()
        return True, f"Stopping: {reason}"

    def on_service(self, request, response):
        response.success, response.message = self.fire("service", "mpc_stop service called")
        return response
