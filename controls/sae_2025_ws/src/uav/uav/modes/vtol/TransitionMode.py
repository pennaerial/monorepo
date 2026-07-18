import numpy as np
from pydantic import BaseModel
from rclpy.node import Node
from typing import Literal, override

from uav.vehicles.VTOL import VTOL
from uav.vehicles.UAV import UAV

from vehicle_common.mode import Mode

from vehicle_common.mode_loader import register_mode


class TransitionParams(BaseModel):
    to_mode: Literal["MC", "FW"]


@register_mode(
    id="uav.vtol.TransitionMode", targets=[UAV], transition_labels=["complete"]
)
class TransitionMode(Mode):
    """
    A VTOL-only mode for transitioning between multicopter and fixed-wing flight.
    """

    transition_labels = ("complete",)

    @override
    def initialize(self, node: Node, vehicle: VTOL, params: TransitionParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params
        self.transition_commanded = False

    def on_update(self, time_delta: float):
        if self.vehicle.vehicle_type is None:
            self.log("Waiting for VTOL status...")
            if self.vehicle.local_position:
                self.vehicle.publish_position_setpoint(
                    (
                        self.vehicle.local_position.x,
                        self.vehicle.local_position.y,
                        self.vehicle.local_position.z,
                    ),
                    lock_yaw=True,
                )
            return

        if self.vehicle.vehicle_type != self.p.to_mode:
            if not self.transition_commanded:
                self.log(
                    f"Transitioning from {self.vehicle.vehicle_type} to {self.p.to_mode}"
                )
                self.vehicle.vtol_transition_to(self.p.to_mode)
                self.transition_commanded = True

            if self.vehicle.local_position:
                if self.p.to_mode == "FW":
                    if self.vehicle.yaw is not None:
                        ahead_dist = 10.0
                        target_pos = (
                            self.vehicle.local_position.x
                            + np.cos(self.vehicle.yaw) * ahead_dist,
                            self.vehicle.local_position.y
                            + np.sin(self.vehicle.yaw) * ahead_dist,
                            self.vehicle.local_position.z,
                        )
                        self.vehicle.publish_position_setpoint(target_pos)
                    else:
                        self.vehicle.publish_position_setpoint(
                            (
                                self.vehicle.local_position.x,
                                self.vehicle.local_position.y,
                                self.vehicle.local_position.z,
                            ),
                            lock_yaw=True,
                        )
                else:
                    self.vehicle.publish_position_setpoint(
                        (
                            self.vehicle.local_position.x,
                            self.vehicle.local_position.y,
                            self.vehicle.local_position.z,
                        ),
                        lock_yaw=True,
                    )
        else:
            self.log(f"Already in {self.p.to_mode} mode")
            if self.vehicle.local_position:
                if self.p.to_mode == "FW" and self.vehicle.yaw is not None:
                    ahead_dist = 10.0
                    target_pos = (
                        self.vehicle.local_position.x
                        + np.cos(self.vehicle.yaw) * ahead_dist,
                        self.vehicle.local_position.y
                        + np.sin(self.vehicle.yaw) * ahead_dist,
                        self.vehicle.local_position.z,
                    )
                    self.vehicle.publish_position_setpoint(target_pos)
                else:
                    self.vehicle.publish_position_setpoint(
                        (
                            self.vehicle.local_position.x,
                            self.vehicle.local_position.y,
                            self.vehicle.local_position.z,
                        ),
                        lock_yaw=True,
                    )

    def check_status(self):
        if self.vehicle.vehicle_type is None:
            return "continue"
        if self.vehicle.vehicle_type != self.p.to_mode:
            return "continue"
        self.log(f"Transition to {self.p.to_mode} complete")
        return "complete"

    @classmethod
    @override
    def get_params_cls(cls) -> type[BaseModel]:
        return TransitionParams
