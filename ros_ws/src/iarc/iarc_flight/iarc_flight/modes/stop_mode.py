from typing import override

import numpy as np
from rclpy.node import Node
from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from iarc_flight.runtime.stop import StopMPCConfig, StopPlanner, StopTrigger


class StopParams(ParamsBase):
    controller: StopMPCConfig = StopMPCConfig()
    stopped_speed: float = 0.1  # m/s, counts as stopped below this
    stopped_time: float = 0.5  # s, must stay stopped this long to complete


@register_mode(
    id="iarc_flight.StopMode",
    params_cls=StopParams,
    targets=[UAV],
    transition_labels=["complete"],
)
class StopMode(Mode[UAV, StopParams]):
    """Brings the drone to v = 0 as fast as the limits allow at the height the stop began,
    re-solving the stop MPC every update. Usually entered from a TrackMode's `stop`, already
    braking; entered any other way it starts braking itself."""

    @override
    def initialize(self, node: Node, vehicle: UAV, params: StopParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.params = params
        self.planner = StopPlanner.configure(node, vehicle, params.controller)

    @override
    def on_enter(self) -> None:
        self.planner.engage()
        self.t = 0.0
        self.stopped_for = 0.0
        self.logged = False

    @override
    def on_update(self, time_delta: float) -> None:
        pos = self.vehicle.local_position
        if pos is None:
            return
        state = np.array([pos.x, pos.y, pos.z, pos.vx, pos.vy, pos.vz])
        self.planner.publish(self.planner.step(state))
        self.t += time_delta
        if np.linalg.norm(state[3:]) < self.params.stopped_speed:
            if not self.logged:
                self.log(f"Stopped in {self.t:.2f} s")
                self.logged = True
            self.stopped_for += time_delta
        else:
            self.stopped_for = 0.0

    @override
    def on_exit(self) -> None:
        self.planner.reset()
        trigger = StopTrigger.existing(self.node)
        if trigger is not None:
            trigger.reset()

    @override
    def check_status(self) -> str:
        return "complete" if self.stopped_for >= self.params.stopped_time else "continue"
