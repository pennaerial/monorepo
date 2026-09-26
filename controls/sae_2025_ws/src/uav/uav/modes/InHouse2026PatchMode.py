from __future__ import annotations

from typing import override

from pydantic import Field
from rclpy.node import Node
from sim_interfaces.srv import GetSearchLocations
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from uav.vehicles.UAV import UAV


class FlyToPatchParams(ParamsBase):
    patch_number: int = Field(default=0, ge=0)
    altitude: float = 7.0 # hover altitude for the patch
    margin: float = 1.0
    hold: bool = False # testing


@register_mode(
    id="uav.FlyToPatch",
    params_cls=FlyToPatchParams,
    targets=[UAV],
    transition_labels=["complete"],
)
class FlyToPatch(Mode[UAV, FlyToPatchParams]):
    """Request a generated patch location and fly over it at a fixed altitude."""

    @override
    def initialize(
        self, node: Node, vehicle: UAV, params: FlyToPatchParams
    ) -> None:
        self.node = node
        self.vehicle = vehicle
        self.params = params
        self.client = self.node.create_client(GetSearchLocations, "/get_search_patches")
        self.request_future = None
        self.target = None
        self.failed = False
        self.wait_remaining = 0.0

    @override
    def on_enter(self) -> None:
        if not self.client.service_is_ready():
            self.log("Waiting for /get_search_patches")
        self.request_future = self.client.call_async(GetSearchLocations.Request())

    @override
    def on_update(self, time_delta: float) -> None:
        if self.target is None and self.request_future is not None and self.request_future.done():
            try:
                response = self.request_future.result() # ask for patches
            except Exception as e:
                self.log(f"Patch request failed: {e}")
                self.failed = True
                return

            if not response.ready:
                self.log("Patch gen not ready")
                self.failed = True
                return

            if self.params.patch_number >= len(response.search_locations):
                self.log(f"Patch {self.params.patch_number} was not returned")
                self.failed = True
                return

            location = response.search_locations[self.params.patch_number]
            if self.vehicle.gps_origin is None:
                return

            target_altitude = self.vehicle.gps_origin[2] + self.params.altitude
            self.target = self.vehicle.gps_to_local(
                (location.latitude_deg, location.longitude_deg, target_altitude)
            )
            self.wait_remaining = 1.0
            self.log(
                f"Flying over patch {self.params.patch_number} at "
                f"{self.params.altitude:.1f} m; target tag {response.target_tag_id}"
            )

        if self.target is not None:
            distance = self.vehicle.distance_to_waypoint("LOCAL", self.target)
            self.vehicle.publish_position_setpoint(
                self.target, lock_yaw=distance < self.params.margin
            )
            if distance < self.params.margin and self.wait_remaining > 0:
                self.wait_remaining -= time_delta
        elif self.vehicle.local_position is not None:
            current = self.vehicle.local_position
            self.vehicle.publish_position_setpoint(
                (current.x, current.y, current.z), lock_yaw=True
            )

    @override
    def check_status(self) -> str:
        if self.failed:
            return "error"
        if self.params.hold:
            return "continue"
        if self.target is not None and self.wait_remaining <= 0:
            return "complete"
        return "continue"