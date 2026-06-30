from px4_msgs.msg import VehicleStatus
from rclpy.node import Node

from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.runtime.plugin_loader import register_plugin


@register_plugin(name="uav.VerticalTakeoffMode", base_cls=Mode)
class VerticalTakeoffMode(Mode):
    """Vertical takeoff for any UAV airframe (multicopter or VTOL)."""

    transition_labels = ("complete",)

    def __init__(self, node: Node, vehicle: UAV) -> None:
        super().__init__(node, vehicle)
        self.takeoff_commanded = False
        self.elapsed = 0.0

    def on_update(self, time_delta: float) -> None:
        self.elapsed += time_delta

        if self.vehicle.local_position is None or self.vehicle.global_position is None:
            self.log("Waiting for position data...")
            return

        if not self.takeoff_commanded:
            self.elapsed = 0.0
            self.log("Commanding vertical takeoff.")
            self.vehicle.takeoff()
            self.takeoff_commanded = True
            return

        self.log("Vertical takeoff in progress.")

        if (
            self.elapsed >= 1.0
            and self.vehicle.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        ):
            self.log("Takeoff complete. Engaging offboard mode.")
            self.vehicle.engage_offboard_mode()

    def check_status(self) -> str:
        if self.vehicle.local_position is None or self.vehicle.global_position is None:
            return "continue"
        if self.vehicle.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.log("In offboard mode — takeoff complete.")
            return "complete"
        return "continue"
