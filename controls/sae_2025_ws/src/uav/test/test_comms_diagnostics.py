from __future__ import annotations

from types import SimpleNamespace

from uav.runtime.comms_diagnostics import (
    log_bootstrap_diagnostics,
    log_comms_diag,
    topic_graph_counts,
)


class _FakeLogger:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def info(self, message: str) -> None:
        self.messages.append(message)


def test_log_comms_diag_prefixes_and_serializes_fields() -> None:
    logger = _FakeLogger()

    log_comms_diag(
        logger,
        "MODE",
        "runtime comm snapshot",
        connection_status={"payload_1": False},
        peer_vehicle_names=("payload_1",),
        ready=False,
    )

    assert len(logger.messages) == 1
    assert logger.messages[0].startswith("COMMS DIAG | MODE | runtime comm snapshot")
    assert 'connection_status={"payload_1": false}' in logger.messages[0]
    assert 'peer_vehicle_names=["payload_1"]' in logger.messages[0]
    assert "ready=false" in logger.messages[0]


def test_log_bootstrap_diagnostics_emits_boot_and_network_snapshots(
    monkeypatch,
) -> None:
    logger = _FakeLogger()

    monkeypatch.setattr("uav.runtime.comms_diagnostics.socket.gethostname", lambda: "test-host")
    monkeypatch.setattr("uav.runtime.comms_diagnostics.os.getpid", lambda: 4242)
    monkeypatch.setattr(
        "uav.runtime.comms_diagnostics._run_snapshot_command",
        lambda command, timeout_sec=1.5: (
            0,
            "snapshot for " + " ".join(command),
        ),
    )

    log_bootstrap_diagnostics(
        logger,
        runtime_kind="payload_mission",
        vehicle_name="payload_0",
        mission_path="/tmp/mission.yaml",
        mission_target="payload",
        auto_launch=True,
        peer_heartbeat_hz=10.0,
        peer_stale_timeout_s=0.5,
        peer_vehicle_names=("payload_1",),
        vision_nodes=(),
    )

    assert len(logger.messages) == 5
    assert logger.messages[0].startswith("COMMS DIAG | BOOT |")
    assert "vehicle_name=payload_0" in logger.messages[0]
    assert "hostname=test-host" in logger.messages[0]
    assert any("COMMS DIAG | NET | snapshot=ip_addr" in message for message in logger.messages)
    assert any(
        "snapshot for nmcli -t -f NAME,TYPE,DEVICE,STATE con show --active" in message
        for message in logger.messages
    )


def test_topic_graph_counts_handles_missing_methods_and_errors() -> None:
    node = SimpleNamespace(
        count_publishers=lambda _topic: 2,
        count_subscribers=lambda _topic: (_ for _ in ()).throw(RuntimeError("boom")),
    )

    counts = topic_graph_counts(node, "/payload_1/mode_manager/heartbeat")

    assert counts["publishers"] == 2
    assert counts["subscribers"] == "error: boom"
