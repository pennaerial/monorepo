from enum import Enum
from typing import override

from rclpy.node import Node
from sim_interfaces.srv import GetSearchLocations
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from uav.vehicles.UAV import UAV


class State(Enum):
    MOVING_BETWEEN_PATCHES = 1
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
        self.scan_waypoints = []

        self.finished = False

    @override
    def on_enter(self):
        # All initialization code
        pass

    @override
    def on_update(self, time_delta: float) -> None:
        match self.state:
            case State.WAIT_FOR_WORLD_SETUP:
                self.wait_for_world_setup()
            case State.SCANNING_PATCH:
                if not self.scan_waypoints:
                    self.generate_scanned_waypoints()
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
            case State.MOVING_BETWEEN_PATCHES:
                pass

    # def move_to_target(self):
    #     self.timer = self.create_timer(1.0, self.timer_callback)

    #     self.vehicle.publish_position_setpoint(self.p.target)
    #     self.vehicle.hover()
    def generate_scanned_waypoints(self, patch_center):
        PATCH_RADIUS_M = 4.0
        SNAKE_PATH_SEPARATION_M = 1.0
        FLIGHT_HEIGHT_M = 5.0
        PATCH_BORDER_PADDING_M = 0.5

        self.scan_waypoints = []
        center_x, center_y = patch_center[:2]
        min_x = center_x - PATCH_RADIUS_M + PATCH_BORDER_PADDING_M
        max_x = center_x + PATCH_RADIUS_M - PATCH_BORDER_PADDING_M
        min_y = center_y - PATCH_RADIUS_M + PATCH_BORDER_PADDING_M
        max_y = center_y + PATCH_RADIUS_M - PATCH_BORDER_PADDING_M
        z = -abs(FLIGHT_HEIGHT_M)

        corners = ((min_x, min_y), (min_x, max_y), (max_x, min_y), (max_x, max_y))
        if self.vehicle.local_position is None:
            start_x, start_y = min_x, min_y
        else:
            # Basically just find the closest corner of the patch near us
            current_x = self.vehicle.local_position.x
            current_y = self.vehicle.local_position.y
            start_x, start_y = min(
                corners,
                key=lambda corner: (corner[0] - current_x) ** 2 + (corner[1] - current_y) ** 2,
            )

        end_x = max_x if start_x == min_x else min_x
        y_step = SNAKE_PATH_SEPARATION_M if start_y == min_y else -SNAKE_PATH_SEPARATION_M

        # Generate the actual path coordinates
        y_values = []
        y = start_y
        if y_step > 0:
            while y <= max_y:
                y_values.append(y)
                y += y_step
            if y_values[-1] != max_y:
                y_values.append(max_y)
        else:
            while y >= min_y:
                y_values.append(y)
                y += y_step
            if y_values[-1] != min_y:
                y_values.append(min_y)

        for i, y in enumerate(y_values):
            line_start_x = start_x if i % 2 == 0 else end_x
            line_end_x = end_x if i % 2 == 0 else start_x
            self.scan_waypoints.append((line_start_x, y, z))
            self.scan_waypoints.append((line_end_x, y, z))


    def scan(self):
        # This timer will check 
        self.timer = self.create_timer(1.0, self.timer_callback)
        # self.timer.cancel()
        for waypoint in generated_waypoints:
            pass
        
        self.vehicle.publish_position_setpoint(self.p.target)
        self.vehicle.hover()

    @override
    def check_status(self) -> str:
        if self.finished:
            return "complete"
        return "continue"
