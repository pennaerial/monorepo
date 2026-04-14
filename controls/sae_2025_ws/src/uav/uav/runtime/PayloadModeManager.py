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
        vehicle_name: str,
        auto_launch: bool = True,
        peer_heartbeat_hz: float = 10.0,
        peer_stale_timeout_s: float = 0.5,
        node_name: str = "mission",
    ) -> None:
        super().__init__(
            node_name,
            vehicle_name=vehicle_name,
            auto_launch=auto_launch,
            peer_heartbeat_hz=peer_heartbeat_hz,
            peer_stale_timeout_s=peer_stale_timeout_s,
        )
        if not mission_spec.is_payload:
            raise ValueError(
                f"PayloadModeManager requires a payload mission spec, received target '{mission_spec.target}'."
            )

        self.vehicle = Payload(self, str(vehicle_name))
        self.setup_vision(list(mission_spec.vision_nodes))
        self.configure_peer_vehicle_names(
            getattr(mission_spec, "peer_vehicle_names", ())
        )
        self.setup_modes(mission_spec)
        self.timer = None

    def spin_once(self) -> None:
        current_time = time()
        if self.active_mode is None:
            self.switch_mode("start")
        self._run_active_mode(current_time)

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
