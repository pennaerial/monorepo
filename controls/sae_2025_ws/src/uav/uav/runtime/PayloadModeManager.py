#!/usr/bin/env python3
from time import time

from uav.vehicles.Payload import Payload
from .ModeManager import ModeManager
from .mission_spec import MissionSpec


class PayloadModeManager(ModeManager):
    """Mission manager for a single payload vehicle."""

    def __init__(
        self,
        *,
        mission_spec: MissionSpec,
        payload_name: str,
        node_name: str = "mission",
    ) -> None:
        super().__init__(node_name)
        if not mission_spec.is_payload:
            raise ValueError(
                f"PayloadModeManager requires a payload mission spec, received target '{mission_spec.target}'."
            )

        self.vehicle = Payload(self, str(payload_name))
        self.setup_vision(list(mission_spec.vision_nodes))
        self.setup_modes(mission_spec)
        self.timer = self.create_timer(0.1, self.spin_once)

    def spin_once(self) -> None:
        current_time = time()
        if self.active_mode is None:
            self.switch_mode("start")
        self._run_active_mode(current_time)

    def _stop_vehicle(self) -> None:
        super()._stop_vehicle()
