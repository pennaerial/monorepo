from enum import Enum
from typing import override

from rclpy.node import Node
from sim_interfaces.srv import GetSearchLocations
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from uav.vehicles.UAV import UAV


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
        self.target_tag_id = -1
        self.state = State.WAIT_FOR_WORLD_SETUP
        self.move_target = (0, 0, 0)
        self.finished = False

        # served by sim's in_house_2026_node once all shapes have spawned
        self.patch_client = node.create_client(GetSearchLocations, "/get_search_patches")
        self.patch_future = None  # None = no request in flight

    @override
    def on_update(self, time_delta: float) -> None:
        match self.state:
            case State.WAIT_FOR_WORLD_SETUP:
                self.wait_for_world_setup()
            case State.SCANNING_PATCH:
                pass
            case State.CHECK_SHAPE:
                pass
            case State.MOVING_TO_LOCATION:
                self.vehicle.publish_position_setpoint(self.move_target)

    def wait_for_world_setup(self) -> None:
        """Poll get_search_patches without blocking until the world reports ready."""
        if self.patch_future is None:
            if self.patch_client.service_is_ready():
                self.patch_future = self.patch_client.call_async(GetSearchLocations.Request())
            return

        if not self.patch_future.done():
            return

        res = self.patch_future.result()
        self.patch_future = None  # clear so the next tick can re-request
        if res is None or not res.ready:
            return

        self.target_patches = list(res.search_locations)
        self.target_tag_id = res.target_tag_id
        self.node.get_logger().info(
            f"World ready: {len(self.target_patches)} search patches, "
            f"target tag {self.target_tag_id}"
        )
        self.state = State.MOVING_TO_LOCATION

    @override
    def check_status(self) -> str:
        if self.finished:
            return "complete"
        return "continue"
