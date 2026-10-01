from __future__ import annotations

from typing import override

from rclpy.node import Node
from sim_interfaces.srv import (
    GetAprilTagId,
    GetPatchPoints,
    GetSearchLocations,
    SolveTSP,
)
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from uav.vehicles.UAV import UAV


class FlyToPatchParams(ParamsBase):
    altitude: float = 7.0
    low_altitude: float = 0.5
    hover_seconds: float = 3.0
    """Used as the stabilization wait after arriving over a patch
    (before requesting points) and as the per-point time used to
    check for the target AprilTag."""
    descent_rate: float = 10.0
    patch_margin: float = 1.0
    point_margin: float = 0.3
    settle_seconds: float = 0.5
    """time to stabilize before scanning apriltag"""
    hold: bool = True


@register_mode(
    id="uav.FlyToPatch",
    params_cls=FlyToPatchParams,
    targets=[UAV],
    transition_labels=["complete"],
)
class FlyToPatch(Mode[UAV, FlyToPatchParams]):
    """Request a generated patch location, then sweep the target pts at low alt,
    searching for target apriltag. infinitely loops until target found.
    Patches and the points within each patch are visited in a TSP-optimized
    order, computed by the /solve_tsp service (open path from the current position)."""

    @override
    def initialize(self, node: Node, vehicle: UAV, params: FlyToPatchParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.params = params
        self.patch_client = self.node.create_client(GetSearchLocations, "/get_search_patches")
        self.point_client = self.node.create_client(GetPatchPoints, "/get_patch_points")
        self.tag_client = self.node.create_client(GetAprilTagId, "/get_apriltag_id")
        self.tsp_client = self.node.create_client(SolveTSP, "/solve_tsp")
        self.patch_request_future = None
        self.point_request_future = None
        self.tag_request_future = None
        self.tsp_future = None
        self.target_tag_id = None
        self.target_found = False
        self.raw_patches = []
        self.raw_points = []
        self.patch_locations = []
        self.patch_index = 0
        self.points = []
        self.point_index = 0
        self.stage = "request_patches"
        self.target = None
        self.failed = False
        self.wait_remaining = 0.0
        self.descent_z = None
        self.point_target = None
        self.patch_target = None

    @override
    def on_enter(self) -> None:
        if not self.patch_client.service_is_ready():
            self.log("Waiting for /get_search_patches")
        self.patch_request_future = self.patch_client.call_async(GetSearchLocations.Request())

    @override
    def on_update(self, time_delta: float) -> None:
        if self.target_found:
            # Hold position; check_status() will hand off to the next mode.
            if self.target is not None:
                self.vehicle.publish_position_setpoint(self.target, lock_yaw=True)
            return

        # surely there is a better way to do this
        if self.stage == "request_patches":
            self._handle_request_patches()
            return

        if self.stage == "solve_patch_tsp":
            self._handle_solve_patch_tsp()
            return

        if self.stage == "fly_patch":
            self._handle_fly_patch()
            return

        if self.stage == "patch_stabilize":
            self._handle_patch_stabilize(time_delta)
            return

        if self.stage == "request_points":
            self._handle_request_points()
            return

        if self.stage == "solve_point_tsp":
            self._handle_solve_point_tsp()
            return

        if self.stage == "fly_point":
            self._handle_fly_point(time_delta)
            return

        if self.stage == "point_settle":
            self._handle_point_settle(time_delta)
            return

        if self.stage == "point_query":
            self._handle_point_query()
            return

        if self.stage == "ascend_patch":
            self._handle_ascend_patch()
            return

        # fallback to hold position if there is no gps origin detected
        if self.vehicle.local_position is not None:
            current = self.vehicle.local_position
            self.vehicle.publish_position_setpoint((current.x, current.y, current.z), lock_yaw=True)

    # stage handlers

    def _handle_request_patches(self) -> None:
        """Request patches from the patch generator service, then ask the TSP service to order them."""
        if self.patch_request_future is None or not self.patch_request_future.done():
            return
        try:
            response = self.patch_request_future.result()
        except Exception as e:
            self.log(f"Patch request failed: {e}")
            self.failed = True
            return

        if not response.ready:
            self.log("Patch gen not ready")
            self.failed = True
            return

        if not response.search_locations:
            self.log("No patches were returned")
            self.failed = True
            return

        # Need a GPS origin and a position fix to build the request; retry next tick.
        if self.vehicle.gps_origin is None or self.vehicle.local_position is None:
            return

        self.raw_patches = list(response.search_locations)
        self.target_tag_id = int(response.target_tag_id)
        self.tsp_future = self._call_tsp(
            [(p.latitude_deg, p.longitude_deg) for p in self.raw_patches]
        )
        self.stage = "solve_patch_tsp"

    def _handle_solve_patch_tsp(self) -> None:
        """Wait for the TSP service, reorder the patches, and fly to the first one."""
        order = self._tsp_result(len(self.raw_patches))
        if order is None:
            return
        self.patch_locations = [self.raw_patches[i] for i in order]
        self.patch_index = 0
        self.log(f"Received {len(self.patch_locations)} patches (TSP-ordered)")
        self._set_patch_target()

    def _handle_fly_patch(self) -> None:
        """Fly to the patch target, then request the patch points once arrived."""
        if self.target is None:
            return
        distance = self.vehicle.distance_to_waypoint("LOCAL", self.target)
        self.vehicle.publish_position_setpoint(
            self.target, lock_yaw=distance < self.params.patch_margin
        )
        if distance >= self.params.patch_margin:
            return
        self.wait_remaining = self.params.hover_seconds
        self.stage = "patch_stabilize"

    def _handle_patch_stabilize(self, time_delta: float) -> None:
        """Wait for the vehicle to stabilize over the patch, then request the patch points."""
        if self.target is None:
            return
        self.vehicle.publish_position_setpoint(self.target, lock_yaw=True)
        self.wait_remaining -= time_delta
        if self.wait_remaining <= 0:
            self.point_request_future = self.point_client.call_async(GetPatchPoints.Request())
            self.stage = "request_points"

    def _handle_request_points(self) -> None:
        """Request the patch points from the patch point service, then ask the TSP service to order them."""
        if self.point_request_future is None or not self.point_request_future.done():
            return
        try:
            response = self.point_request_future.result()
        except Exception as e:
            self.log(f"Patch point request failed: {e}")
            self.failed = True
            return
        if len(response.latitude_deg) != len(response.longitude_deg):
            self.log("the lat long arrays are bad")
            self.failed = True
            return
        if not response.latitude_deg:
            self.log("nothing returned")
            self.failed = True
            return

        if self.vehicle.gps_origin is None or self.vehicle.local_position is None:
            return

        self.raw_points = list(zip(response.latitude_deg, response.longitude_deg))
        self.tsp_future = self._call_tsp(self.raw_points)
        self.stage = "solve_point_tsp"

    def _handle_solve_point_tsp(self) -> None:
        """Wait for the TSP service, reorder the points, and fly to the first one."""
        order = self._tsp_result(len(self.raw_points))
        if order is None:
            return
        self.points = [self.raw_points[i] for i in order]
        self.point_index = 0
        self.log(f"Received {len(self.points)} points for patch {self.patch_index} (TSP-ordered)")
        self._set_point_target()

    def _handle_fly_point(self, time_delta: float) -> None:
        """fly to point target, descend, settle for apriltag query"""
        if self.point_target is None or self.descent_z is None:
            return
        target_z = self.point_target[2]
        self.descent_z = self._step_toward(
            self.descent_z, target_z, self.params.descent_rate * time_delta
        )
        commanded = (self.point_target[0], self.point_target[1], self.descent_z)
        self.vehicle.publish_position_setpoint(commanded, lock_yaw=True)

        distance = self.vehicle.distance_to_waypoint("LOCAL", self.point_target)
        if distance < self.params.point_margin:
            self.wait_remaining = self.params.settle_seconds
            self.stage = "point_settle"

    def _handle_point_settle(self, time_delta: float) -> None:
        """Wait for the vehicle to settle at the point target, then query the apriltag service."""
        if self.point_target is None:
            return
        self.vehicle.publish_position_setpoint(self.point_target, lock_yaw=True)

        distance = self.vehicle.distance_to_waypoint("LOCAL", self.point_target)
        if distance >= self.params.point_margin:
            # Drifted back out before settling; restart the timer.
            self.wait_remaining = self.params.settle_seconds
            return

        self.wait_remaining -= time_delta
        if self.wait_remaining > 0:
            return

        self.tag_request_future = self.tag_client.call_async(GetAprilTagId.Request())
        self.stage = "point_query"

    def _handle_point_query(self) -> None:
        """query apriltag"""
        if self.point_target is None:
            return
        self.vehicle.publish_position_setpoint(self.point_target, lock_yaw=True)

        if self.tag_request_future is None or not self.tag_request_future.done():
            return

        try:
            response = self.tag_request_future.result()
        except Exception as e:
            self.log(f"AprilTag id request failed: {e}")
        else:
            seen = list(response.tag_ids) if response.success else []
            self.log(f"target={self.target_tag_id}, seen={seen}")
            if self.target_tag_id is not None and self.target_tag_id in seen:
                latitude, longitude = self.points[self.point_index]
                self.log(
                    f"\n\nTarget found at ({latitude:.6f}, {longitude:.6f}) in patch #{self.patch_index + 1}\n"
                )
                self.target_found = True
        self.tag_request_future = None

        if self.target_found:
            return

        self.point_index += 1
        if self.point_index < len(self.points):
            self._set_point_target()
        else:
            self.target = self.patch_target
            self.stage = "ascend_patch"

    def _handle_ascend_patch(self) -> None:
        """leave"""
        if self.target is None:
            return
        distance = self.vehicle.distance_to_waypoint("LOCAL", self.target)
        self.vehicle.publish_position_setpoint(
            self.target, lock_yaw=distance < self.params.patch_margin
        )
        if distance >= self.params.patch_margin:
            return
        self.patch_index = (self.patch_index + 1) % len(self.patch_locations)
        self._set_patch_target()

    # helpers

    @staticmethod
    def _step_toward(current: float, target: float, max_delta: float) -> float:
        if current < target:
            return min(target, current + max_delta)
        return max(target, current - max_delta)

    def _call_tsp(self, latlons: list[tuple[float, float]]):
        """Convert (lat, lon) pairs to local x/y and ask the TSP service for a visiting order,
        starting from the vehicle's current position."""
        origin_z = self.vehicle.gps_origin[2]
        current = self.vehicle.local_position
        xy = [self.vehicle.gps_to_local((lat, lon, origin_z))[:2] for lat, lon in latlons]
        request = SolveTSP.Request()
        request.start_x = float(current.x)
        request.start_y = float(current.y)
        request.x = [float(p[0]) for p in xy]
        request.y = [float(p[1]) for p in xy]
        return self.tsp_client.call_async(request)

    def _tsp_result(self, expected: int) -> list[int] | None:
        """Return the TSP order once the service replies, else None (or fail the mode)."""
        if self.tsp_future is None or not self.tsp_future.done():
            return None
        try:
            response = self.tsp_future.result()
        except Exception as e:
            self.log(f"TSP request failed: {e}")
            self.failed = True
            return None
        self.tsp_future = None
        if not response.success or len(response.order) != expected:
            self.log("TSP service returned a bad ordering")
            self.failed = True
            return None
        return list(response.order)

    def _set_patch_target(self) -> None:
        if self.vehicle.gps_origin is None:
            return
        location = self.patch_locations[self.patch_index]
        self.patch_target = self.vehicle.gps_to_local(
            (
                location.latitude_deg,
                location.longitude_deg,
                self.vehicle.gps_origin[2] + self.params.altitude,
            )
        )
        self.target = self.patch_target
        self.stage = "fly_patch"
        self.log(f"Flying over patch {self.patch_index + 1} at {self.params.altitude:.1f} m")

    def _set_point_target(self) -> None:
        if self.vehicle.gps_origin is None:
            return
        latitude, longitude = self.points[self.point_index]
        self.point_target = self.vehicle.gps_to_local(
            (latitude, longitude, self.vehicle.gps_origin[2] + self.params.low_altitude)
        )
        # Start the z-ramp from wherever we currently are (patch altitude
        # for the first point, already-low altitude for subsequent ones),
        # while x/y jump straight to the new point so the vehicle glides
        # diagonally instead of flying over-then-dropping.
        current = self.vehicle.local_position
        self.descent_z = current.z if current is not None else self.point_target[2]
        self.stage = "fly_point"
        self.log(f"Flying to point {self.point_index + 1}/{len(self.points)}")

    @override
    def check_status(self) -> str:
        if self.failed:
            return "error"
        if self.target_found:
            return "complete"  # move onto next mode and land
        return "continue"
