from typing import Optional, Tuple, override
from pydantic import BaseModel
from rclpy.node import Node
from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import register_mode


class ServoDropoffParams(BaseModel):
    """
    offsets: Should denote the position of dropoff relative to the center of zone, in meters
        In NED frame: x is forward, y is right, and z is down.
    camera_offsets: Should denote the position of the camera relative to the payload mechanism, in meters
        In NED frame: x is forward, y is right, and z is down.
    """

    offsets: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
    camera_offsets: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)


@register_mode(
    id="uav.ServoDropoffMode",
    params_cls=ServoDropoffParams,
    targets=[UAV],
    transition_labels=["complete"]
)
class ServoDropoffMode(Mode):
    """
    A mode for dropping off the payload.
    """

    transition_labels = ("complete",)

    @override
    def initialize(self, node: Node, vehicle: UAV, params: ServoDropoffParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params

        self.lower_time = 1.87
        self.timer = self.lower_time
        self.done = False
        # 0 for uav centering, 1 for landing, 2 for retracting, 3 for taking off

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for lowering payload and handling obstacles.
        """
        # If UAV is unstable, skip the update
        if self.timer >= 0:
            self.vehicle.drop_payload()
            self.timer -= time_delta
        if self.timer < 0 and self.timer > -self.lower_time:
            self.lowering = False
            self.vehicle.pickup_payload()
            self.timer -= time_delta
        if self.timer < -self.lower_time:
            self.vehicle.disable_servo()
            self.done = True

    def check_status(self) -> str:
        """
        Check the status of the payload lowering.

        Returns:
            str: The status of the payload lowering.
        """
        if self.done:
            return "complete"
        return "continue"
