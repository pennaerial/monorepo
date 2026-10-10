#!/usr/bin/env python3
from time import time

from vehicle_common.mode_manager import ModeManager
from vehicle_common.runtime.mission_loader import RuntimeMission, get_mission_path

from payload.payload import Payload
from payload.payload_mission_parameters import payload_mission


class PayloadModeManager(ModeManager):
    """Mission manager for a single payload vehicle."""

    def __init__(self, node_name: str = "mission") -> None:
        super().__init__(node_name)
        mission_spec = RuntimeMission.load_from_path(
            self.params.mode_map or get_mission_path("basic", "payload")
        )
        if Payload not in mission_spec._targets:
            raise ValueError(
                "PayloadModeManager requires a payload mission spec, received targets "
                f"{sorted(t.__name__ for t in mission_spec._targets)}."
            )

        self.vehicle = Payload(self, self.params.vehicle_name.strip())
        self.setup_modes(mission_spec)
        self.timer = None

    def load_params(self) -> payload_mission.Params:
        return payload_mission.ParamListener(self).get_params()

    def spin_once(self) -> None:
        current_time = time()
        if self.active_mode is None:
            self.switch_mode("start")
        self.run_active_mode(current_time)

    def _auto_launch_ready(self) -> bool:
        if self.vehicle is None:
            return False
        timed_drive_client = getattr(self.vehicle, "timed_drive_client", None)
        if timed_drive_client is None:
            return False
        wait_for_service = getattr(timed_drive_client, "wait_for_service", None)
        if not callable(wait_for_service):
            return False
        return bool(wait_for_service(timeout_sec=0.0))

    def _stop_vehicle(self) -> None:
        super()._stop_vehicle()
