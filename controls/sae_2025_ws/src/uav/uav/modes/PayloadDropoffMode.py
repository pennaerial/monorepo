from typing import Optional, Tuple, override

import numpy as np
from pydantic import BaseModel
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node

from uav.vehicles.UAV import UAV
from uav.vision_nodes import PayloadTrackingNode
from uav_interfaces.srv import PayloadTracking

from vehicle_common.mode import Mode
from vehicle_common.mode_loader import register_mode


class PayloadDropoffParams(BaseModel):
    """
    offsets: Should denote the position of dropoff relative to the center of zone, in meters
        In NED frame: x is forward, y is right, and z is down.
    """

    offsets: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)


@register_mode(
    id="uav.PayloadDropoffMode",
    targets=[UAV],
    required_vision_nodes=[PayloadTrackingNode],
    transition_labels=["complete"],
)
class PayloadDropoffMode(Mode):
    """
    A mode for dropping off the payload.
    """

    required_vision_nodes = (PayloadTrackingNode,)
    transition_labels = ("complete",)

    @override
    def initialize(
        self, node: Node, vehicle: UAV, params: PayloadDropoffParams
    ) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params

        self.response = None
        self.done = False
        self.camera_offsets = self.vehicle.camera_offsets
        self.mode = (
            0  # 0 for uav centering, 1 for landing, 2 for retracting, 3 for taking off
        )

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for lowering payload and handling obstacles.
        """
        # If UAV is unstable, skip the update
        if self.vehicle.roll > 0.1 or self.vehicle.pitch > 0.1:
            self.log("Roll or pitch detected. Waiting for stabilization.")
            return

        if self.mode == 1:
            if self.vehicle.vehicle_status != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self.mode = 2
            else:
                return
        if self.mode == 2:
            if True:  # TODO: Check if payload is dropped off (servo is fully retracted)
                self.mode = 3
                self.vehicle.attempted_takeoff = False
            else:
                return
        if self.mode == 3:
            if self.vehicle.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self.done = True
            return

        request = PayloadTracking.Request()
        request.altitude = -self.vehicle.get_local_position()[2]
        request.yaw = float(self.vehicle.yaw)
        request.payload_color = "pink"
        response = self.send_request(PayloadTrackingNode, request)

        # If no payload pose is received, exit early
        if response is None:
            return

        direction = [
            -response.direction[1],
            response.direction[0],
            response.direction[2],
        ]

        offsets = (
            tuple(x / request.altitude for x in self.p.offsets)
            if request.altitude > 1
            else self.p.offsets
        )
        camera_offsets = (
            tuple(x / request.altitude for x in self.camera_offsets)
            if request.altitude > 1
            else self.camera_offsets
        )
        direction = [
            x + y + z
            for x, y, z in zip(
                direction, offsets, self.vehicle.uav_to_local(camera_offsets)
            )
        ]

        if request.altitude < 1:
            # If payload pose direction is within a small threshold
            if (
                np.abs(direction[0]) < request.altitude / 25
                and np.abs(direction[1]) < request.altitude / 25
            ):
                self.vehicle.land()
                self.mode = 1
                return
            else:
                direction[2] = 0

        self.log(f"Direction: {direction}")
        self.vehicle.publish_position_setpoint(direction, relative=True)

    def check_status(self) -> str:
        """
        Check the status of the payload lowering.

        Returns:
            str: The status of the payload lowering.
        """
        if self.done:
            return "complete"
        return "continue"

    @classmethod
    @override
    def get_params_cls(cls) -> type[BaseModel]:
        return PayloadDropoffParams
