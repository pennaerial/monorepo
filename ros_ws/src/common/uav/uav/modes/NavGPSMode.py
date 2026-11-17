from __future__ import annotations

from typing import Literal, override

from px4_msgs.msg import VtolVehicleStatus
from rclpy.node import Node
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from uav.vehicles.UAV import UAV
from uav.vehicles.VTOL import VTOL


class NavGPSParams(ParamsBase):
    """
    coordinates: The coordinates to navigate to (x/y/z or lon/lat/alt, wait time, GPS/LOCAL).
        Local are NED coordinates, relative to the starting position (https://docs.px4.io/main/en/ros2/user_guide.html#ros-2-px4-frame-conventions).
    max_horizontal_speed: Maximum horizontal speed in m/s.
    max_vertical_speed: Maximum vertical speed in m/s.
    max_heading_rate: Maximum yaw rate in rad/s.
    margin: The margin of error for the GPS coordinate.

    """

    coordinates: list[tuple[tuple[float, float, float], float, Literal["GPS", "LOCAL"]]]
    max_horizontal_speed: float = 5
    max_vertical_speed: float = 3
    max_heading_rate: float = 0.5
    margin: float = 1


@register_mode(
    id="uav.NavGPSMode",
    params_cls=NavGPSParams,
    targets=[UAV],
    transition_labels=["complete"],
)
class NavGPSMode(Mode[UAV, NavGPSParams]):
    """
    A mode for navigating to a GPS coordinate
    """

    # TODO: Create "RELATIVE" navigation system, which initializes direction vectors once in the beginning
    # and converts all coordinates to that frame. +X should point to forward, +Y should point to the right, -Z should point up

    @override
    def initialize(self, node: Node, vehicle: UAV, params: NavGPSParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params
        self.goal = None
        self.index = -1
        self.coordinate_system = None
        self.target = None
        self.wait_time = 0.0
        self.navigation_error = False

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for setting gps coord.
        """
        if self.navigation_error:
            return

        # if vehicle is a VTOL and in fixed wing mode, failsafe because GoToSetpoint doesn't work for non Multicopter controller
        if (
            isinstance(self.vehicle, VTOL)
            and self.vehicle.vtol_vehicle_status is not None
            and self.vehicle.vtol_vehicle_status.vehicle_vtol_state
            == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW
        ):
            self.node.get_logger().error(
                "NavGPSMode goto navigation requires multicopter flight; "
                "VTOL is in fixed-wing mode. Triggering failsafe."
            )
            self.navigation_error = True  # causes check_status() to return "error"
            return

        dist = 0
        if self.index != -1:
            dist = self.vehicle.distance_to_waypoint(self.coordinate_system, self.goal)

        # Determine if we're waiting at a waypoint (lock yaw to prevent spinning)
        at_waypoint = self.index != -1 and dist < self.p.margin
        waiting = at_waypoint and self.wait_time > 0

        # Always publish setpoints to maintain offboard connection (PX4 requires continuous stream)
        if self.target is not None:
            self.vehicle.publish_goto_setpoint(self.target, self.p.max_horizontal_speed, self.p.max_vertical_speed, self.p.max_heading_rate, lock_yaw=waiting)
        elif self.vehicle.local_position:
            # Fallback: maintain current position until first waypoint is set
            self.vehicle.publish_goto_setpoint(
                (
                    self.vehicle.local_position.x,
                    self.vehicle.local_position.y,
                    self.vehicle.local_position.z,
                ),
                max_horizontal_speed=self.p.max_horizontal_speed,
                max_vertical_speed=self.p.max_vertical_speed,
                max_heading_rate=self.p.max_heading_rate,
                lock_yaw=True,
            )

        # Consolidated debug output - single line with all information
        if self.index != -1 and self.target is not None:
            curr_pos = self.vehicle.get_local_position()
            curr_gps = self.vehicle.get_gps()
            yaw = (
                self.vehicle.yaw
                if self.vehicle.yaw is not None
                else (self.vehicle.local_position.heading if self.vehicle.local_position else 0.0)
            )
            vel = (
                (
                    self.vehicle.local_position.vx,
                    self.vehicle.local_position.vy,
                    self.vehicle.local_position.vz,
                )
                if self.vehicle.local_position
                else (0.0, 0.0, 0.0)
            )

            # Build target position string
            if self.coordinate_system == "GPS":
                target_str = f"Target GPS: ({self.goal[0]:.6f}, {self.goal[1]:.6f}, {self.goal[2]:.2f}) | Target LOCAL: ({self.target[0]:.2f}, {self.target[1]:.2f}, {self.target[2]:.2f})"
            else:
                target_str = f"Target LOCAL: ({self.target[0]:.2f}, {self.target[1]:.2f}, {self.target[2]:.2f})"

            # Build consolidated log line
            if curr_pos and curr_gps:
                self.log(
                    f"Dist: {dist:.2f}m | {target_str} | "
                    f"Global: ({curr_gps[0]:.6f}, {curr_gps[1]:.6f}, {curr_gps[2]:.2f}) | "
                    f"Local: ({curr_pos[0]:.2f}, {curr_pos[1]:.2f}, {curr_pos[2]:.2f}) | "
                    f"Yaw: {yaw:.2f} | Vel: ({vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f})"
                )

        if self.goal is None or dist < self.p.margin:
            if self.index == -1 or self.wait_time <= 0:
                self.index += 1
                if self.index >= len(self.p.coordinates):
                    self.log("All waypoints completed")
                    return
                self.goal, self.wait_time, self.coordinate_system = self.p.coordinates[self.index]
                self.target = self.get_local_target()
                self.log(
                    f"Moving to waypoint {self.index + 1}/{len(self.p.coordinates)}: {self.goal} (wait time: {self.wait_time}s)"
                )
            else:
                # Waiting at waypoint
                self.log(
                    f"Waiting at waypoint {self.index + 1}/{len(self.p.coordinates)} - {self.wait_time:.1f}s remaining"
                )
                self.wait_time -= time_delta

    def get_local_target(self) -> tuple[float, float, float]:
        """
        Get the local target of the UAV.

        Returns:
            tuple[float, float, float]: The local target of the UAV.
        """
        if self.coordinate_system == "GPS":
            local_target = self.vehicle.gps_to_local(self.goal)
            return local_target
        elif self.coordinate_system == "LOCAL":
            # LOCAL coordinates are already in NED frame relative to origin
            local_target = tuple(float(x) for x in self.goal)
            return local_target
        else:
            raise ValueError(f"Invalid coordinate system {self.coordinate_system}")

    def check_status(self) -> str:
        """
        Check the status of the mode.

        Returns:
            str: The status of the mode.
        """
        if self.navigation_error:
            return "error"
        if self.index >= len(self.p.coordinates):
            return "complete"
        else:
            return "continue"
