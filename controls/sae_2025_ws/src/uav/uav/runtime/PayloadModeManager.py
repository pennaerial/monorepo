#!/usr/bin/env python3
import os
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
        node_name: str = "mission",
        emits_hotspot: bool = True,
        target_ip: str = "192.168.4.1",
        sync_domain_id: int = -1,
    ) -> None:
        super().__init__(node_name, auto_launch=auto_launch)
        if not mission_spec.is_payload:
            raise ValueError(
                f"PayloadModeManager requires a payload mission spec, received target '{mission_spec.target}'."
            )

        self.emits_hotspot: bool = emits_hotspot
        self.target_ip: str = target_ip

        main_domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
        self.sync_domain_id: int = (
            main_domain if sync_domain_id == -1 else sync_domain_id
        )
        self._main_domain_id: int = main_domain

        self.vehicle = Payload(self, str(vehicle_name))
        self.setup_vision(list(mission_spec.vision_nodes))
        self.setup_modes(mission_spec)
        self.timer = None

    @property
    def sync_is_local(self) -> bool:
        return self.sync_domain_id == self._main_domain_id

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
