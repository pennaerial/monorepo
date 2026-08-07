from __future__ import annotations

import threading
import time
from typing import Any, Callable, Sequence


def _default_log(message: str) -> None:
    print(message, flush=True)


class Deadline:
    def __init__(self, timeout_s: float) -> None:
        self.timeout_s = float(timeout_s)
        self._started_at = time.time()
        self._expires_at = self._started_at + self.timeout_s

    def elapsed_s(self) -> float:
        return time.time() - self._started_at

    def remaining_s(self) -> float:
        return max(0.0, self._expires_at - time.time())


class HeadlessGroundStation:
    """Tiny MAVLink ground station for headless PX4 simulation checks."""

    def __init__(
        self,
        endpoint: str = "udpin:0.0.0.0:14550",
        *,
        heartbeat_period_s: float = 0.5,
        log: Callable[[str], None] = _default_log,
        mavutil_module: Any | None = None,
    ) -> None:
        self.endpoint = endpoint
        self.heartbeat_period_s = heartbeat_period_s
        self._log = log
        self._mavutil = mavutil_module
        self._master: Any | None = None
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread: threading.Thread | None = None
        self._ever_armed = False
        self._ever_airborne = False
        self._landed_state: int | None = None
        self._tracking_vtol_states = False
        self._pending_vtol_states: list[int] = []

    def __enter__(self) -> "HeadlessGroundStation":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def connect(self, timeout_s: float = 60.0) -> Any | None:
        mavutil = self._load_mavutil()
        self._master = mavutil.mavlink_connection(self.endpoint)

        self._log("waiting for PX4 heartbeat ...")
        heartbeat = self._master.wait_heartbeat(timeout=timeout_s)
        if heartbeat is not None:
            self._log(
                f"connected: sys {self._master.target_system} comp {self._master.target_component}"
            )
        return heartbeat

    def start_heartbeats(self) -> None:
        master = self._require_master()
        mavlink = self._load_mavutil().mavlink
        if self._heartbeat_thread is not None and self._heartbeat_thread.is_alive():
            return

        self._heartbeat_stop.clear()

        def heartbeat_loop() -> None:
            while not self._heartbeat_stop.is_set():
                master.mav.heartbeat_send(
                    mavlink.MAV_TYPE_GCS,
                    mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                time.sleep(self.heartbeat_period_s)

        self._heartbeat_thread = threading.Thread(target=heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

    def stop_heartbeats(self) -> None:
        self._heartbeat_stop.set()
        thread = self._heartbeat_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=self.heartbeat_period_s + 0.1)
        self._heartbeat_thread = None

    def request_message_interval(self, message_id: int, interval_us: int) -> None:
        master = self._require_master()
        mavlink = self._load_mavutil().mavlink
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    def request_extended_sys_state(self, interval_us: int = 200_000) -> None:
        mavlink = self._load_mavutil().mavlink
        self.request_message_interval(
            mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
            interval_us,
        )

    def deadline(self, timeout_s: float) -> Deadline:
        return Deadline(timeout_s)

    def wait_armed(
        self,
        timeout_s: float | None = None,
        *,
        deadline: Deadline | None = None,
    ) -> None:
        self._wait_for(
            lambda _msg: self._ever_armed,
            timeout_s=timeout_s,
            deadline=deadline,
            timeout_message=lambda active_deadline: self._mission_timeout_message(active_deadline),
        )

    def wait_airborne(
        self,
        timeout_s: float | None = None,
        *,
        deadline: Deadline | None = None,
    ) -> None:
        self._wait_for(
            lambda _msg: self._ever_airborne,
            timeout_s=timeout_s,
            deadline=deadline,
            timeout_message=lambda active_deadline: self._mission_timeout_message(active_deadline),
        )

    def wait_landed(
        self,
        timeout_s: float | None = None,
        *,
        deadline: Deadline | None = None,
    ) -> list[int]:
        mavlink = self._load_mavutil().mavlink
        elapsed = self._wait_for(
            lambda msg: (
                msg.get_type() == "EXTENDED_SYS_STATE"
                and msg.landed_state == mavlink.MAV_LANDED_STATE_ON_GROUND
            ),
            timeout_s=timeout_s,
            deadline=deadline,
            timeout_message=lambda active_deadline: self._mission_timeout_message(active_deadline),
        )

        pending_states = self.pending_vtol_states()
        if not pending_states:
            self._log(f"[{elapsed:6.1f}s] LANDED -- mission complete")
        return pending_states

    def expect_vtol_states(self, expected_states: Sequence[int]) -> None:
        self._tracking_vtol_states = True
        self._pending_vtol_states = list(expected_states)

    def pending_vtol_states(self) -> list[int]:
        return list(self._pending_vtol_states)

    def close(self) -> None:
        self.stop_heartbeats()
        if self._master is None:
            return
        close = getattr(self._master, "close", None)
        if close is not None:
            close()
        self._master = None

    def _wait_for(
        self,
        predicate: Callable[[Any], bool],
        *,
        timeout_s: float | None = None,
        deadline: Deadline | None = None,
        timeout_message: Callable[[Deadline], str],
    ) -> float:
        active_deadline = self._resolve_deadline(timeout_s, deadline)
        while active_deadline.remaining_s() > 0.0:
            msg = self._recv_relevant(timeout_s=min(1.0, active_deadline.remaining_s()))
            if msg is None:
                continue

            elapsed = active_deadline.elapsed_s()
            self._observe_message(msg, elapsed)
            if predicate(msg):
                return elapsed
        raise TimeoutError(timeout_message(active_deadline))

    def _observe_message(
        self,
        msg: Any,
        elapsed_s: float,
    ) -> None:
        mavlink = self._load_mavutil().mavlink
        msg_type = msg.get_type()

        if msg_type == "HEARTBEAT" and msg.type != mavlink.MAV_TYPE_GCS:
            is_armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if is_armed and not self._ever_armed:
                self._ever_armed = True
                self._log(f"[{elapsed_s:6.1f}s] ARMED")
            return

        if msg_type != "EXTENDED_SYS_STATE":
            return

        if self._pending_vtol_states and msg.vtol_state == self._pending_vtol_states[0]:
            reached = self._pending_vtol_states.pop(0)
            self._log(f"[{elapsed_s:6.1f}s] VTOL_STATE -> {reached}")

        self._landed_state = msg.landed_state
        if msg.landed_state == mavlink.MAV_LANDED_STATE_IN_AIR and not self._ever_airborne:
            self._ever_airborne = True
            self._log(f"[{elapsed_s:6.1f}s] AIRBORNE (takeoff)")

    def _recv_relevant(self, timeout_s: float) -> Any | None:
        return self._require_master().recv_match(
            type=["HEARTBEAT", "EXTENDED_SYS_STATE"],
            blocking=True,
            timeout=timeout_s,
        )

    def _require_master(self) -> Any:
        if self._master is None:
            raise RuntimeError("HeadlessGroundStation.connect() must be called first.")
        return self._master

    def _resolve_deadline(
        self,
        timeout_s: float | None,
        deadline: Deadline | None,
    ) -> Deadline:
        if deadline is not None:
            return deadline
        if timeout_s is None:
            raise ValueError("timeout_s or deadline must be provided.")
        return Deadline(timeout_s)

    def _mission_timeout_message(
        self,
        deadline: Deadline,
    ) -> str:
        if self._tracking_vtol_states:
            return (
                f"TIMEOUT after {deadline.timeout_s:.0f}s "
                f"(armed={self._ever_armed}, airborne={self._ever_airborne}, "
                f"pending_vtol_states={self._pending_vtol_states})"
            )
        return f"TIMEOUT after {deadline.timeout_s:.0f}s (armed={self._ever_armed}, airborne={self._ever_airborne})"

    def _load_mavutil(self) -> Any:
        if self._mavutil is not None:
            return self._mavutil
        try:
            from pymavlink import mavutil
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "HeadlessGroundStation requires pymavlink. Install pymavlink or run inside the sim CI image."
            ) from exc
        self._mavutil = mavutil
        return self._mavutil
