"""
PayloadWaitForPlaneMode — waits for the other payload to reach a target substate.

Subscribes to ``/payload_sync`` (directly or via a bash subprocess depending
on ``emits_hotspot``) and transitions with ``"complete"`` once the target
substate string is received.

Transitions:
  "complete" -> next mode configured in the mission YAML
"""

from __future__ import annotations

from rclpy.node import Node

from uav.vehicles.Payload import Payload

from ..Mode import Mode
from .sync import PayloadSyncSubscriberMixin


class PayloadWaitForPlaneMode(PayloadSyncSubscriberMixin, Mode):
    """
    Idle mode that subscribes to the inter-payload sync topic and waits for
    a specific substate from the other payload before transitioning.
    """

    mission_target = "payload"
    required_vision_nodes = ()
    requires_camera = False
    transition_labels = ("complete",)

    def __init__(
        self,
        node: Node,
        vehicle: Payload,
        wait_for_state: str = "DONE",
    ):
        super().__init__(node, vehicle)
        self.vehicle: Payload = vehicle
        self.wait_for_state = str(wait_for_state)

    def on_enter(self) -> None:
        self.vehicle.stop()
        self._sync_sub_enter(self.wait_for_state)
        self.log(f"waiting for sync state '{self.wait_for_state}' from other payload")

    def on_update(self, time_delta: float) -> None:
        reached = self._sync_check_target()
        if not reached and self._sync_latest_state:
            self.log(
                f"latest sync state: '{self._sync_latest_state}' "
                f"(waiting for '{self.wait_for_state}')"
            )

    def check_status(self) -> str:
        return "complete" if self._sync_target_reached else "continue"

    def on_exit(self) -> None:
        self._sync_sub_exit()
