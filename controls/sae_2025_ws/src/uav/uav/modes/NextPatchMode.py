from __future__ import annotations

from typing import override

from rclpy.node import Node
from rclpy.task import Future
from sim_interfaces.msg import SearchLocation
from sim_interfaces.srv import GetSearchLocations
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from uav.vehicles.UAV import UAV


class NextPatchParams(ParamsBase):
    """Configuration parameters for NextPatchMode.

    altitude: Cruising height above takeoff ground in meters (NED z = -altitude).
    margin: Distance threshold in meters to consider arrival at patch location.
    service_name: ROS 2 service name providing search patches.
    """

    altitude: float = 5.0
    margin: float = 1.0
    service_name: str = "/get_search_patches"


@register_mode(
    id="uav.NextPatchMode",
    params_cls=NextPatchParams,
    targets=[UAV],
    transition_labels=["complete", "no_patches"],
)
class NextPatchMode(Mode[UAV, NextPatchParams]):
    """Mission mode for sequential patch visits in the in_house_2026 challenge world.

    When first in focus, it queries the /get_search_patches service to initialize
    all patch locations. On each activation, it pops a patch location and commands
    the UAV toward it, reporting "complete" upon arrival or "no_patches" when
    all patches have been visited.
    """

    transition_labels = ("complete", "no_patches")

    @override
    def initialize(self, node: Node, vehicle: UAV, params: NextPatchParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params

        self.patches: list[SearchLocation] | None = None
        self.search_locations: list[SearchLocation] = []
        self.target_tag_id: int | None = None
        self.current_patch: SearchLocation | None = None
        self.target: tuple[float, float, float] | None = None

        service_name = self.p.service_name
        if not service_name.startswith("/"):
            service_name = f"/{service_name}"
        self.resolved_service_name = service_name
        self.client = self.node.create_client(GetSearchLocations, self.resolved_service_name)
        self._future: Future | None = None
        self._fetching = False
        self._exhausted = False
        self._arrived = False

    def pop_patch(self) -> SearchLocation | None:
        """Pop and return the next patch location from the queue."""
        if self.patches and len(self.patches) > 0:
            self.current_patch = self.patches.pop(0)
            return self.current_patch
        self.current_patch = None
        return None

    def pop_and_set_target(self) -> bool:
        """Pop the next patch and compute its local NED target coordinate."""
        patch = self.pop_patch()
        if patch is None:
            self._exhausted = True
            self.target = None
            self.log("No more patches in queue.")
            return False

        self._exhausted = False
        self._arrived = False
        self._compute_target_for_patch(patch)
        self.log(
            f"Popped patch at ({patch.latitude_deg:.6f}, {patch.longitude_deg:.6f}), "
            f"target LOCAL: {self.target}, remaining: {len(self.patches) if self.patches is not None else 0}"
        )
        return True

    def _compute_target_for_patch(self, patch: SearchLocation) -> None:
        """Convert GPS coordinates of the given patch to local NED frame."""
        if self.vehicle.gps_origin is not None:
            ref_alt = self.vehicle.gps_origin[2]
            target_gps = (patch.latitude_deg, patch.longitude_deg, ref_alt + self.p.altitude)
            local_pos = self.vehicle.gps_to_local(target_gps)
            if local_pos is not None:
                self.target = (local_pos[0], local_pos[1], -abs(self.p.altitude))
                return
        self.target = None

    def _fetch_patches(self) -> None:
        """Initiate asynchronous service request to fetch search patches."""
        if self._fetching:
            return

        if not self.client.service_is_ready():
            self.log(f"Waiting for {self.resolved_service_name} service to become available...")
            return

        self._fetching = True
        request = GetSearchLocations.Request()
        self._future = self.client.call_async(request)
        self.log(f"Requested patch locations from {self.resolved_service_name}.")

    @override
    def on_enter(self) -> None:
        """Called whenever NextPatchMode becomes active ("in focus")."""
        self._arrived = False

        if self.patches is None:
            # First time in focus: initialize patches from service
            self.log("First time in focus: initializing patch locations...")
            self._fetch_patches()
        else:
            # Re-entering focus: pop next patch
            self.pop_and_set_target()

    @override
    def on_update(self, time_delta: float) -> None:
        """Periodic control loop: handle async response and command UAV setpoint."""
        # Handle asynchronous initialization if still pending
        if self.patches is None:
            if self._future is not None and self._future.done():
                self._fetching = False
                try:
                    response = self._future.result()
                    if response is not None and response.ready:
                        self.search_locations = list(response.search_locations)
                        self.target_tag_id = response.target_tag_id
                        self.patches = list(response.search_locations)
                        self.log(
                            f"Loaded {len(self.patches)} patch locations "
                            f"(target tag ID: {self.target_tag_id})."
                        )
                        self.pop_and_set_target()
                    else:
                        self.log("Patches not ready in world generator yet; will retry.")
                except Exception as exc:
                    self.log(f"Error querying {self.resolved_service_name}: {exc}")
                self._future = None
            elif self._future is None and not self._fetching:
                self._fetch_patches()

        # If a patch was popped but target couldn't be computed yet (e.g. waiting for gps_origin)
        if self.current_patch is not None and self.target is None:
            self._compute_target_for_patch(self.current_patch)

        # UAV setpoint streaming
        if self.vehicle.local_position is None:
            return

        if self.target is not None:
            self.vehicle.publish_position_setpoint(self.target)
        else:
            # Hover in place at configured altitude until target is ready
            self.vehicle.publish_position_setpoint(
                (
                    self.vehicle.local_position.x,
                    self.vehicle.local_position.y,
                    -abs(self.p.altitude),
                ),
                lock_yaw=True,
            )

    @override
    def check_status(self) -> str:
        """Report outcome of the mode."""
        if self._exhausted:
            return "no_patches"

        if self.patches is None or self.target is None or self.vehicle.local_position is None:
            return "continue"

        dist = self.vehicle.distance_to_waypoint("LOCAL", self.target)
        if dist <= self.p.margin:
            self._arrived = True
            self.log(f"Arrived at patch! Dist: {dist:.2f}m <= margin {self.p.margin}m")
            return "complete"

        return "continue"

    @override
    def on_exit(self) -> None:
        """Called when NextPatchMode is deactivated."""
        self._arrived = False
