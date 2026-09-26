from typing import override

from rclpy.node import Node

from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from enum import Enum


class State(Enum):
    MOVING_TO_LOCATION = 1
    SCANNING_PATCH = 2
    CHECK_SHAPE = 3
    WAIT_FOR_WORLD_SETUP = 4

class WinParams(ParamsBase):
    pass


@register_mode(
    id="uav.WinMode",
    params_cls=WinParams,
    targets=[UAV],
    transition_labels=["complete"],
)
class WinMode(Mode[UAV, WinParams]):
    @override
    def initialize(
        self,
        node: Node,
        vehicle: UAV,
        params: WinParams,
    ) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params

        self.target_patches = []
        self.state = State.WAIT_FOR_WORLD_SETUP
        self.move_target = (0, 0, 0)

        self.finished = false

    @override
    def on_update(self, time_delta: float) -> None:
        self.node.get_logger().info(f"Time Delta: {time_delta}")

        match self.state:
            case State.WAIT_FOR_WORLD_SETUP:
                # if self.vehicle.local_position is None: return
                # Wait for the world to initialize
                # Check the service
                pass
            case State.SCANNING_PATCH:
                pass
            case State.CHECK_SHAPE:
                pass
            case State.MOVING_TO_LOCATION:
                self.vehicle.publish_position_setpoint(self.p.target)

    @override
    def check_status(self) -> str:
        # if self.vehicle.local_position is None:
        #     return "continue"

        # distance = self.vehicle.distance_to_waypoint("LOCAL", self.move_target)
        # if distance <= self.p.margin:
        #     return "complete"
        if self.finished:
            return "complete"
        return "continue"