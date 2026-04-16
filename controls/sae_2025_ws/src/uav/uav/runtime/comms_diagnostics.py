from __future__ import annotations

import json
import os
import shlex
import socket
import subprocess
from typing import Any, Mapping, Sequence

_DIAG_PREFIX = "COMMS DIAG"
_ENV_KEYS = (
    "ROS_DOMAIN_ID",
    "ROS_LOCALHOST_ONLY",
    "RMW_IMPLEMENTATION",
    "ROS_DISCOVERY_SERVER",
    "CYCLONEDDS_URI",
    "CYCLONEDDS_HOME",
    "FASTRTPS_DEFAULT_PROFILES_FILE",
    "FASTDDS_DEFAULT_PROFILES_FILE",
)
_NETWORK_COMMANDS = (
    ("ip_addr", ("ip", "-br", "addr")),
    ("ip_route", ("ip", "route")),
    ("nmcli_dev_status", ("nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "dev", "status")),
    (
        "nmcli_active_connections",
        ("nmcli", "-t", "-f", "NAME,TYPE,DEVICE,STATE", "con", "show", "--active"),
    ),
)


def _single_line(value: object) -> str:
    return " ".join(str(value).split())


def _json_ready(value: object) -> object:
    if isinstance(value, Mapping):
        return {
            str(key): _json_ready(inner_value)
            for key, inner_value in sorted(value.items(), key=lambda item: str(item[0]))
        }
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    if isinstance(value, set):
        return [_json_ready(item) for item in sorted(value, key=str)]
    return value


def _serialize(value: object) -> str:
    if isinstance(value, str):
        return _single_line(value)
    if isinstance(value, (dict, list, tuple, set)):
        return json.dumps(_json_ready(value), sort_keys=True, default=str)
    if isinstance(value, bool):
        return "true" if value else "false"
    return _single_line(value)


def log_comms_diag(logger, category: str, message: str | None = None, **fields) -> None:
    parts: list[str] = []
    if message:
        parts.append(_single_line(message))
    for key, value in fields.items():
        if value is None:
            continue
        parts.append(f"{key}={_serialize(value)}")
    payload = " ".join(parts)
    if payload:
        logger.info(f"{_DIAG_PREFIX} | {category} | {payload}")
        return
    logger.info(f"{_DIAG_PREFIX} | {category}")


def _run_snapshot_command(
    command: Sequence[str], *, timeout_sec: float = 1.5
) -> tuple[int | None, str]:
    try:
        result = subprocess.run(
            list(command),
            capture_output=True,
            text=True,
            check=False,
            timeout=timeout_sec,
        )
    except FileNotFoundError:
        return None, "unavailable"
    except subprocess.TimeoutExpired:
        return None, "timeout"
    except Exception as exc:  # pragma: no cover - defensive
        return None, f"error: {exc}"

    output = _single_line(
        result.stdout.strip() or result.stderr.strip() or "<no output>"
    )
    return int(result.returncode), output


def log_bootstrap_diagnostics(
    logger,
    *,
    runtime_kind: str,
    vehicle_name: str,
    mission_path: str,
    mission_target: str,
    auto_launch: bool,
    peer_heartbeat_hz: float,
    peer_stale_timeout_s: float,
    peer_vehicle_names: Sequence[str],
    vision_nodes: Sequence[str] = (),
) -> None:
    log_comms_diag(
        logger,
        "BOOT",
        runtime_kind=runtime_kind,
        vehicle_name=vehicle_name,
        mission_path=mission_path,
        mission_target=mission_target,
        auto_launch=auto_launch,
        peer_heartbeat_hz=peer_heartbeat_hz,
        peer_stale_timeout_s=peer_stale_timeout_s,
        peer_vehicle_names=tuple(peer_vehicle_names),
        vision_nodes=tuple(vision_nodes),
        pid=os.getpid(),
        hostname=socket.gethostname(),
        env={key: os.environ.get(key, "") for key in _ENV_KEYS},
    )
    for label, command in _NETWORK_COMMANDS:
        returncode, output = _run_snapshot_command(command)
        log_comms_diag(
            logger,
            "NET",
            snapshot=label,
            command=shlex.join(command),
            returncode=returncode,
            output=output,
        )


def topic_graph_counts(node, topic_name: str) -> dict[str, object]:
    counts: dict[str, object] = {}
    for label, method_name in (
        ("publishers", "count_publishers"),
        ("subscribers", "count_subscribers"),
    ):
        method = getattr(node, method_name, None)
        if not callable(method):
            counts[label] = None
            continue
        try:
            counts[label] = int(method(topic_name))
        except Exception as exc:  # pragma: no cover - defensive
            counts[label] = f"error: {_single_line(exc)}"
    return counts

