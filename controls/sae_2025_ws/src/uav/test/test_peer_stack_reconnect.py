# ruff: noqa: E402

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import pytest

if shutil.which("ros2") is None:
    pytest.skip(
        "ros2 CLI is required for live peer stack reconnect tests",
        allow_module_level=True,
    )

rclpy = pytest.importorskip("rclpy")
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger


def _require_uav_package() -> None:
    try:
        get_package_share_directory("uav")
    except PackageNotFoundError as exc:  # pragma: no cover - environment dependent
        pytest.skip(
            f"uav package is not discoverable in the current ROS environment: {exc}"
        )


class _LaunchProcess:
    def __init__(
        self,
        *,
        name: str,
        process: subprocess.Popen[str],
        log_path: Path,
        log_handle,
    ):
        self.name = name
        self.process = process
        self.log_path = log_path
        self._log_handle = log_handle

    def assert_running(self) -> None:
        return_code = self.process.poll()
        if return_code is None:
            return
        raise AssertionError(
            f"{self.name} exited early with code {return_code}.\n"
            f"Last log lines:\n{self.tail_logs()}"
        )

    def stop(self) -> None:
        try:
            if self.process.poll() is None:
                for signum, timeout_sec in (
                    (signal.SIGINT, 10.0),
                    (signal.SIGTERM, 5.0),
                    (signal.SIGKILL, 5.0),
                ):
                    try:
                        os.killpg(self.process.pid, signum)
                    except ProcessLookupError:
                        return

                    deadline = time.monotonic() + timeout_sec
                    while time.monotonic() < deadline:
                        if self.process.poll() is not None:
                            return
                        time.sleep(0.1)
        finally:
            self._log_handle.close()

    def tail_logs(self, lines: int = 40) -> str:
        if not self.log_path.exists():
            return "<no log output>"
        content = self.log_path.read_text(
            encoding="utf-8", errors="replace"
        ).splitlines()
        return "\n".join(content[-lines:]) if content else "<empty log>"


class _PeerStackObserver(Node):
    def __init__(self) -> None:
        super().__init__("peer_stack_reconnect_observer")
        self.status_by_vehicle: dict[str, dict[str, object]] = {}
        self.status_timestamps: dict[str, float] = {}
        self.heartbeat_counts = {"payload_0": 0, "payload_1": 0}
        self.heartbeat_timestamps: dict[str, float] = {}
        self.shared_counts_by_sender: dict[str, int] = {}

        for vehicle_name in ("payload_0", "payload_1"):
            self.create_subscription(
                String,
                f"/{vehicle_name}/peer_test/state",
                lambda message, vehicle_name=vehicle_name: self._on_status(
                    vehicle_name, message
                ),
                10,
            )
            self.create_subscription(
                Empty,
                f"/{vehicle_name}/mode_manager/heartbeat",
                lambda _message, vehicle_name=vehicle_name: self._on_heartbeat(
                    vehicle_name
                ),
                10,
            )

        self.create_subscription(
            String,
            "/shared/peer_test/broadcast",
            self._on_shared,
            10,
        )
        self._start_clients = {
            vehicle_name: self.create_client(
                Trigger, f"/{vehicle_name}/mode_manager/start_mission"
            )
            for vehicle_name in ("payload_0", "payload_1")
        }

    def _on_status(self, vehicle_name: str, message: String) -> None:
        try:
            payload = json.loads(str(message.data))
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        self.status_by_vehicle[vehicle_name] = payload
        self.status_timestamps[vehicle_name] = time.monotonic()

    def _on_heartbeat(self, vehicle_name: str) -> None:
        self.heartbeat_counts[vehicle_name] = (
            self.heartbeat_counts.get(vehicle_name, 0) + 1
        )
        self.heartbeat_timestamps[vehicle_name] = time.monotonic()

    def _on_shared(self, message: String) -> None:
        try:
            payload = json.loads(str(message.data))
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        sender = str(payload.get("vehicle", "")).strip()
        if not sender:
            return
        self.shared_counts_by_sender[sender] = (
            self.shared_counts_by_sender.get(sender, 0) + 1
        )

    def spin_until(
        self,
        predicate,
        *,
        timeout_sec: float,
        failure_message: str,
        processes: tuple[_LaunchProcess, ...] = (),
    ):
        deadline = time.monotonic() + timeout_sec
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            for process in processes:
                process.assert_running()
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                result = predicate()
            except Exception as exc:  # pragma: no cover - debug path
                last_error = exc
            else:
                if result:
                    return result
            time.sleep(0.05)
        if last_error is not None:
            raise AssertionError(f"{failure_message}: {last_error}") from last_error
        raise AssertionError(failure_message)

    def wait_for_service(
        self,
        vehicle_name: str,
        *,
        timeout_sec: float,
        processes: tuple[_LaunchProcess, ...] = (),
    ) -> None:
        client = self._start_clients[vehicle_name]

        def _service_ready() -> bool:
            return bool(client.wait_for_service(timeout_sec=0.0))

        self.spin_until(
            _service_ready,
            timeout_sec=timeout_sec,
            failure_message=f"{vehicle_name} start_mission service did not become available",
            processes=processes,
        )

    def start_mission(
        self,
        vehicle_name: str,
        *,
        timeout_sec: float,
        processes: tuple[_LaunchProcess, ...] = (),
    ) -> None:
        client = self._start_clients[vehicle_name]
        future = client.call_async(Trigger.Request())

        def _response_ready():
            if not future.done():
                return None
            response = future.result()
            if response is None:
                raise AssertionError(
                    f"{vehicle_name} start_mission returned no response"
                )
            if not bool(response.success):
                raise AssertionError(
                    f"{vehicle_name} start_mission failed: {response.message}"
                )
            return response

        self.spin_until(
            _response_ready,
            timeout_sec=timeout_sec,
            failure_message=f"{vehicle_name} start_mission did not complete",
            processes=processes,
        )

    def wait_for_heartbeat(
        self,
        vehicle_name: str,
        *,
        timeout_sec: float,
        minimum_count: int = 1,
        processes: tuple[_LaunchProcess, ...] = (),
    ) -> None:
        self.spin_until(
            lambda: self.heartbeat_counts.get(vehicle_name, 0) >= minimum_count,
            timeout_sec=timeout_sec,
            failure_message=f"{vehicle_name} heartbeat did not reach count {minimum_count}",
            processes=processes,
        )

    def wait_for_heartbeat_quiet(
        self,
        vehicle_name: str,
        *,
        quiet_sec: float,
        timeout_sec: float,
    ) -> None:
        self.spin_until(
            lambda: (
                vehicle_name in self.heartbeat_timestamps
                and time.monotonic() - self.heartbeat_timestamps[vehicle_name]
                >= quiet_sec
            ),
            timeout_sec=timeout_sec,
            failure_message=f"{vehicle_name} heartbeat did not go quiet",
        )

    def wait_for_status(
        self,
        vehicle_name: str,
        *,
        timeout_sec: float,
        received_after: float | None = None,
        processes: tuple[_LaunchProcess, ...] = (),
        **expected: object,
    ) -> dict[str, object]:
        def _matches() -> dict[str, object] | None:
            payload = self.status_by_vehicle.get(vehicle_name)
            if payload is None:
                return None
            timestamp = self.status_timestamps.get(vehicle_name, 0.0)
            if received_after is not None and timestamp <= received_after:
                return None
            for key, value in expected.items():
                if payload.get(key) != value:
                    return None
            return payload

        return self.spin_until(
            _matches,
            timeout_sec=timeout_sec,
            failure_message=f"{vehicle_name} did not publish expected status {expected}",
            processes=processes,
        )

    def wait_for_status_minimums(
        self,
        vehicle_name: str,
        *,
        timeout_sec: float,
        received_after: float | None = None,
        processes: tuple[_LaunchProcess, ...] = (),
        **minimums: int,
    ) -> dict[str, object]:
        def _matches() -> dict[str, object] | None:
            payload = self.status_by_vehicle.get(vehicle_name)
            if payload is None:
                return None
            timestamp = self.status_timestamps.get(vehicle_name, 0.0)
            if received_after is not None and timestamp <= received_after:
                return None
            for key, minimum in minimums.items():
                value = payload.get(key)
                if not isinstance(value, int) or value < minimum:
                    return None
            return payload

        return self.spin_until(
            _matches,
            timeout_sec=timeout_sec,
            failure_message=(
                f"{vehicle_name} did not reach expected status minimums {minimums}"
            ),
            processes=processes,
        )


def _write_test_mission(tmp_path: Path) -> Path:
    mission_path = tmp_path / "payload_peer_reconnect_test.yaml"
    mission_path.write_text(
        (
            "modes:\n"
            "  start:\n"
            "    class: uav.modes.payload.PayloadPeerFleetTestMode\n"
            "    params:\n"
            "      publish_period_s: 0.2\n"
        ),
        encoding="utf-8",
    )
    return mission_path


def _vehicle_config(*, vehicle_name: str, mission_path: Path) -> dict[str, object]:
    return {
        "vehicle_name": vehicle_name,
        "kind": "payload",
        "mission_path": str(mission_path),
        "sim": False,
        "auto_launch": False,
        "launch_payload_backend": False,
        "force_camera": False,
        "vision_debug": False,
        "debug": False,
    }


def _launch_vehicle_stack(
    *,
    vehicle_name: str,
    mission_path: Path,
    tmp_path: Path,
    env: dict[str, str],
) -> _LaunchProcess:
    log_path = tmp_path / f"{vehicle_name}.log"
    log_file = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "uav",
            "vehicle_stack.launch.py",
            f"vehicle_json:={json.dumps(_vehicle_config(vehicle_name=vehicle_name, mission_path=mission_path), separators=(',', ':'))}",
        ],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        start_new_session=True,
    )
    return _LaunchProcess(
        name=vehicle_name,
        process=process,
        log_path=log_path,
        log_handle=log_file,
    )


@pytest.fixture
def live_ros_environment(monkeypatch):
    _require_uav_package()
    domain_id = str(200 + (os.getpid() % 20))
    ros_log_dir = Path("/tmp/ros_logs")
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    monkeypatch.setenv("ROS_DOMAIN_ID", domain_id)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(ros_log_dir))
    rclpy.init()
    try:
        yield os.environ.copy()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_vehicle_stack_peer_reconnect_recovers_state_and_traffic(
    tmp_path: Path, live_ros_environment: dict[str, str]
) -> None:
    mission_path = _write_test_mission(tmp_path)
    observer = _PeerStackObserver()
    processes: list[_LaunchProcess] = []

    try:
        payload_0 = _launch_vehicle_stack(
            vehicle_name="payload_0",
            mission_path=mission_path,
            tmp_path=tmp_path,
            env=live_ros_environment,
        )
        processes.append(payload_0)
        observer.wait_for_service("payload_0", timeout_sec=30.0, processes=(payload_0,))
        observer.start_mission("payload_0", timeout_sec=15.0, processes=(payload_0,))
        observer.wait_for_heartbeat(
            "payload_0", timeout_sec=15.0, processes=(payload_0,)
        )
        status = observer.wait_for_status(
            "payload_0",
            timeout_sec=15.0,
            processes=(payload_0,),
            state="waiting",
            disconnected_peers=["payload_1"],
        )
        assert status["peer_received_total"] == 0
        assert status["shared_remote_received_total"] == 0

        payload_1 = _launch_vehicle_stack(
            vehicle_name="payload_1",
            mission_path=mission_path,
            tmp_path=tmp_path,
            env=live_ros_environment,
        )
        processes.append(payload_1)
        observer.wait_for_service(
            "payload_1", timeout_sec=30.0, processes=(payload_0, payload_1)
        )
        observer.start_mission(
            "payload_1", timeout_sec=15.0, processes=(payload_0, payload_1)
        )
        observer.wait_for_heartbeat(
            "payload_1", timeout_sec=15.0, processes=(payload_0, payload_1)
        )

        observer.wait_for_status(
            "payload_0",
            timeout_sec=20.0,
            processes=(payload_0, payload_1),
            state="connected",
            disconnected_peers=[],
        )
        observer.wait_for_status(
            "payload_1",
            timeout_sec=20.0,
            processes=(payload_0, payload_1),
            state="connected",
            disconnected_peers=[],
        )
        status_0 = observer.wait_for_status_minimums(
            "payload_0",
            timeout_sec=15.0,
            processes=(payload_0, payload_1),
            peer_received_total=1,
            shared_remote_received_total=1,
        )
        observer.wait_for_status_minimums(
            "payload_1",
            timeout_sec=15.0,
            processes=(payload_0, payload_1),
            peer_received_total=1,
            shared_remote_received_total=1,
        )
        assert observer.shared_counts_by_sender.get("payload_0", 0) > 0
        assert observer.shared_counts_by_sender.get("payload_1", 0) > 0

        payload_0_peer_before_disconnect = int(status_0["peer_received_total"])
        payload_0_shared_before_disconnect = int(
            status_0["shared_remote_received_total"]
        )
        payload_1_shared_events_before_disconnect = (
            observer.shared_counts_by_sender.get("payload_1", 0)
        )

        payload_1.stop()
        observer.wait_for_heartbeat_quiet("payload_1", quiet_sec=1.0, timeout_sec=10.0)
        disconnected_after = time.monotonic()
        waiting_0 = observer.wait_for_status(
            "payload_0",
            timeout_sec=15.0,
            received_after=disconnected_after,
            processes=(payload_0,),
            state="waiting",
            disconnected_peers=["payload_1"],
        )
        assert int(waiting_0["peer_received_total"]) >= payload_0_peer_before_disconnect
        assert (
            int(waiting_0["shared_remote_received_total"])
            >= payload_0_shared_before_disconnect
        )

        payload_1 = _launch_vehicle_stack(
            vehicle_name="payload_1",
            mission_path=mission_path,
            tmp_path=tmp_path,
            env=live_ros_environment,
        )
        processes[-1] = payload_1
        reconnect_started_after = time.monotonic()
        observer.wait_for_service(
            "payload_1", timeout_sec=30.0, processes=(payload_0, payload_1)
        )
        observer.start_mission(
            "payload_1", timeout_sec=15.0, processes=(payload_0, payload_1)
        )
        heartbeat_count_before_reconnect = observer.heartbeat_counts.get("payload_1", 0)
        observer.wait_for_heartbeat(
            "payload_1",
            timeout_sec=15.0,
            minimum_count=heartbeat_count_before_reconnect + 1,
            processes=(payload_0, payload_1),
        )

        observer.wait_for_status(
            "payload_0",
            timeout_sec=20.0,
            received_after=disconnected_after,
            processes=(payload_0, payload_1),
            state="connected",
            disconnected_peers=[],
        )
        observer.wait_for_status(
            "payload_1",
            timeout_sec=20.0,
            received_after=reconnect_started_after,
            processes=(payload_0, payload_1),
            state="connected",
            disconnected_peers=[],
        )
        reconnect_status_0 = observer.wait_for_status_minimums(
            "payload_0",
            timeout_sec=15.0,
            received_after=disconnected_after,
            processes=(payload_0, payload_1),
            peer_received_total=payload_0_peer_before_disconnect + 1,
            shared_remote_received_total=payload_0_shared_before_disconnect + 1,
        )
        reconnect_status_1 = observer.wait_for_status_minimums(
            "payload_1",
            timeout_sec=15.0,
            received_after=reconnect_started_after,
            processes=(payload_0, payload_1),
            peer_received_total=1,
            shared_remote_received_total=1,
        )
        assert (
            int(reconnect_status_0["peer_received_total"])
            > payload_0_peer_before_disconnect
        )
        assert (
            int(reconnect_status_0["shared_remote_received_total"])
            > payload_0_shared_before_disconnect
        )
        assert int(reconnect_status_1["peer_received_total"]) > 0
        assert int(reconnect_status_1["shared_remote_received_total"]) > 0
        assert (
            observer.shared_counts_by_sender.get("payload_1", 0)
            > payload_1_shared_events_before_disconnect
        )
    finally:
        for process in reversed(processes):
            process.stop()
        observer.destroy_node()
