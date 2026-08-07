from typing import override

from px4_msgs.msg import VehicleStatus
from rclpy.node import Node

from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode
from enum import StrEnum


class TakeoffMethod(StrEnum):
    OFFBOARD = "OFFBOARD"  # use offboard mode to control takeoff setpoint
    PX4_AUTO = "PX4_AUTO"  # rely on px4 AUTO_TAKEOFF mode for takeoff


class VerticalTakeoffParams(ParamsBase):
    takeoff_height: float = 5.0
    takeoff_method: str = "PX4_AUTO"


@register_mode(
    id="uav.VerticalTakeoffMode",
    params_cls=VerticalTakeoffParams,
    targets=[UAV],
    transition_labels=["complete"],
)
class VerticalTakeoffMode(Mode):
    """Vertical takeoff for any UAV airframe (multicopter or VTOL)."""

    transition_labels = ("complete",)

    @override
    def initialize(self, node: Node, vehicle: UAV, params: VerticalTakeoffParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params
        self.logger = self.node.get_logger()
        self.elapsed = 0.0

        valid = ", ".join(m.value for m in TakeoffMethod)
        try:
            self.tko_method = TakeoffMethod(params.takeoff_method)
        except ValueError:
            self.logger.error(
                f"takeoff_method: {params.takeoff_method} is not valid. Valid options are: {valid}"
            )

        # px4_auto uses only
        self.sent_px4_tko_command = False

        # used by offboard tko only
        self.tko_waypoint_ned = (0, 0, -abs(params.takeoff_height))
        self.last_arm_attempt_t = 0.0
        self.last_offboard_attempt_t = 0.0

        # not set as input params because PX4_AUTO doesn't need, and these should not be changing per-mission
        self.waypoint_margin = 0.1
        self.arm_freq_s = 1.2
        self.offboard_cmd_freq_s = 1.2

    def on_update(self, time_delta: float) -> None:
        if self.vehicle.local_position is None or self.vehicle.global_position is None:
            self.logger.info("Waiting for position data...", throttle_duration_sec=1.0)
            return

        # important to start tracking time_delta after position checks pass
        self.elapsed += time_delta
        if self.tko_method == TakeoffMethod.PX4_AUTO:
            self.px4_auto_tko()
        elif self.tko_method == TakeoffMethod.OFFBOARD:
            self.offboard_tko()

    def px4_auto_tko(self) -> None:
        """
        Uses PX4 AUTO_TAKEOFF mode to takeoff. Sends a single takeoff commands,
        then listens for a PX4 transition to AUTO_LOITER before transitioning
        to px4 offboard mode and entering the next mode
        """

        if not self.sent_px4_tko_command:
            self.elapsed = 0.0
            self.logger.info("Commanding vertical takeoff.", throttle_duration_sec=1.0)
            self.vehicle.takeoff(self.p.takeoff_height)
            self.sent_px4_tko_command = True
            return

        self.logger.info(" PX4 vertical auto takeoff in progress.", throttle_duration_sec=1.0)

        if (
            self.elapsed >= 1.0
            and self.vehicle.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        ):
            self.log("Takeoff complete. Engaging offboard mode.")
            self.vehicle.engage_offboard_mode()

    def offboard_tko(self) -> None:
        """
        Takes off by manually sending in trajectory setpoints in offboard mode. Does not use
        PX4's auto takeoff mode at all. The requirements are that trajectory setpoints and offboard heartbeats
        should be publishing > 2Hz for at least 1 seconds before a transition to offboard mode should be sent.

        We publish the trajectory setpoint at all times, and if not armed or offboard mode, we send those commands
        based off of self.arm_freq_s and self.offboard_cmd_freq_s

        """
        if (
            self.vehicle.arm_state == VehicleStatus.ARMING_STATE_DISARMED
            and self.elapsed - self.last_arm_attempt_t > self.arm_freq_s
        ):
            self.vehicle.arm()
            self.last_offboard_attempt_t = self.elapsed

        if (
            self.vehicle.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.elapsed - self.last_offboard_attempt_t > self.offboard_cmd_freq_s
        ):
            self.vehicle.engage_offboard_mode()
            self.last_offboard_attempt_t = self.elapsed

        self.vehicle.publish_position_setpoint(self.tko_waypoint_ned)

    def check_status(self) -> str:
        if (
            self.tko_method == TakeoffMethod.PX4_AUTO
            and self.vehicle.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            self.log("In offboard mode — PX4 auto takeoff complete")
            return "complete"

        elif self.tko_method == TakeoffMethod.OFFBOARD:
            if (
                self.vehicle.distance_to_waypoint("LOCAL", self.tko_waypoint_ned)
                < self.waypoint_margin
            ):
                return "complete"

        return "continue"
