from __future__ import annotations

import os
import subprocess
import textwrap
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "hardware" / "wifi-failover.sh"


def _write_executable(path: Path, body: str) -> None:
    path.write_text(textwrap.dedent(body), encoding="utf-8")
    path.chmod(0o755)


def _make_fake_tools(
    tmp_path: Path,
    *,
    runtime_policy_active: bool,
    runtime_allowed_ap_hosts: list[str] | None = None,
    visible_networks: list[tuple[str, int]] | None = None,
) -> tuple[Path, Path, Path]:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    nmcli_log = tmp_path / "nmcli.log"
    runtime_policy_path = tmp_path / "deploy" / "config" / "network-policy.yaml"

    _write_executable(
        bin_dir / "nmcli",
        """\
        #!/usr/bin/env bash
        set -euo pipefail

        log_file="${NMCLI_LOG:?}"
        printf 'nmcli %s\\n' "$*" >> "$log_file"

        if [[ "${1:-}" == "-t" && "${2:-}" == "-f" && "${3:-}" == "DEVICE,TYPE" && "${4:-}" == "device" && "${5:-}" == "status" ]]; then
            printf 'wlan0:wifi\\n'
            exit 0
        fi

        if [[ "${1:-}" == "-t" && "${2:-}" == "-f" && "${3:-}" == "NAME" && "${4:-}" == "connection" && "${5:-}" == "show" && "${6:-}" == "--active" ]]; then
            exit 0
        fi

        if [[ "${1:-}" == "-t" && "${2:-}" == "-f" && "${3:-}" == "NAME" && "${4:-}" == "connection" && "${5:-}" == "show" ]]; then
            exit 0
        fi

        if [[ "${1:-}" == "-t" && "${2:-}" == "-f" && "${3:-}" == "ACTIVE,SSID" && "${4:-}" == "device" && "${5:-}" == "wifi" && "${6:-}" == "list" && "${7:-}" == "ifname" && "${9:-}" == "--rescan" && "${10:-}" == "no" ]]; then
            exit 0
        fi

        if [[ "${1:-}" == "-t" && "${2:-}" == "-f" && "${3:-}" == "SSID,SIGNAL" && "${4:-}" == "device" && "${5:-}" == "wifi" && "${6:-}" == "list" && "${7:-}" == "ifname" && "${9:-}" == "--rescan" && "${10:-}" == "auto" ]]; then
            while IFS= read -r line; do
                [[ -n "$line" ]] || continue
                printf '%s\\n' "$line"
            done <<<"${FAKE_VISIBLE_NETWORKS:-}"
            exit 0
        fi

        if [[ "${1:-}" == "device" && "${2:-}" == "wifi" && "${3:-}" == "connect" ]]; then
            printf 'CONNECT %s\\n' "$*" >> "$log_file"
            exit 0
        fi

        if [[ "${1:-}" == "connection" && "${2:-}" == "modify" ]]; then
            printf 'MODIFY %s\\n' "$*" >> "$log_file"
            exit 0
        fi

        exit 0
        """,
    )

    _write_executable(
        bin_dir / "systemctl",
        """\
        #!/usr/bin/env bash
        set -euo pipefail

        if [[ "${1:-}" == "is-active" && "${2:-}" == "--quiet" && "${3:-}" == "pennair-autonomy.service" ]]; then
            if [[ "${FAKE_SYSTEMCTL_ACTIVE:-0}" == "1" ]]; then
                mkdir -p "$(dirname "$RUNTIME_POLICY_PATH")"
                {
                    printf 'fallback_ap_prefix: pennair-ap\\n'
                    printf 'network_role: client\\n'
                    printf 'ap_selection: %s\\n' "${FAKE_RUNTIME_AP_SELECTION:-ordered_then_strongest}"
                    printf 'allowed_ap_hosts:\\n'
                    while IFS= read -r host; do
                        [[ -n "$host" ]] || continue
                        printf '  - %s\\n' "$host"
                    done <<<"${FAKE_RUNTIME_ALLOWED_AP_HOSTS:-}"
                } > "$RUNTIME_POLICY_PATH"
                exit 0
            fi
            rm -f "$RUNTIME_POLICY_PATH" 2>/dev/null || true
            exit 1
        fi

        exit 0
        """,
    )

    return bin_dir, nmcli_log, runtime_policy_path


def _run_failover(
    tmp_path: Path,
    *,
    default_policy: str,
    runtime_policy_active: bool,
    runtime_allowed_ap_hosts: list[str] | None = None,
    visible_networks: list[tuple[str, int]] | None = None,
    switching_disabled: bool = False,
) -> subprocess.CompletedProcess[str]:
    bin_dir, nmcli_log, runtime_policy_path = _make_fake_tools(
        tmp_path,
        runtime_policy_active=runtime_policy_active,
        runtime_allowed_ap_hosts=runtime_allowed_ap_hosts,
        visible_networks=visible_networks,
    )

    policy_path = tmp_path / "default-policy.yaml"
    policy_path.write_text(default_policy, encoding="utf-8")
    switching_disable_marker = tmp_path / "runtime" / "disable-switching"
    if switching_disabled:
        switching_disable_marker.parent.mkdir(parents=True, exist_ok=True)
        switching_disable_marker.write_text("disabled\n", encoding="utf-8")

    env = os.environ.copy()
    env.update(
        {
            "PATH": f"{bin_dir}{os.pathsep}{env.get('PATH', '')}",
            "PENNAIR_NETWORK_DEFAULT_POLICY": str(policy_path),
            "PENNAIR_WIFI_SWITCHING_DISABLE_MARKER": str(switching_disable_marker),
            "DEFAULT_DEPLOY_ROOT": str(tmp_path / "deploy"),
            "DEFAULT_HOSTNAME": "air-02",
            "DEFAULT_FALLBACK_PSK": "shared-psk",
            "DEFAULT_FALLBACK_AP_PREFIX": "pennair-ap",
            "DEFAULT_NETWORK_ROLE": "client",
            "DEFAULT_RUNTIME_SERVICE_UNIT": "pennair-autonomy.service",
            "NMCLI_LOG": str(nmcli_log),
            "RUNTIME_POLICY_PATH": str(runtime_policy_path),
            "FAKE_VISIBLE_NETWORKS": "\n".join(
                f"{ssid}:{signal}" for ssid, signal in (visible_networks or [])
            ),
            "FAKE_SYSTEMCTL_ACTIVE": "1" if runtime_policy_active else "0",
            "FAKE_RUNTIME_ALLOWED_AP_HOSTS": "\n".join(runtime_allowed_ap_hosts or []),
            "FAKE_RUNTIME_AP_SELECTION": "ordered_then_strongest",
        }
    )

    return subprocess.run(
        ["bash", str(SCRIPT)],
        capture_output=True,
        text=True,
        check=False,
        env=env,
    )


def _nmcli_log_lines(tmp_path: Path) -> list[str]:
    log_path = tmp_path / "nmcli.log"
    if not log_path.exists():
        return []
    return [
        line.strip()
        for line in log_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def test_empty_default_allowed_ap_hosts_uses_visible_pennair_ap_candidates_and_skips_self(
    tmp_path: Path,
) -> None:
    result = _run_failover(
        tmp_path,
        default_policy="""\
        hostname: air-02
        fallback_ap_prefix: pennair-ap
        fallback_psk: shared-psk
        network_role: client
        ap_selection: ordered_then_strongest
        allowed_ap_hosts: []
        """,
        runtime_policy_active=False,
        visible_networks=[
            ("pennair-ap-air-02", 20),
            ("pennair-ap-air-03", 70),
            ("pennair-ap-air-01", 90),
        ],
    )

    assert result.returncode == 0, result.stderr or result.stdout

    connect_lines = [
        line for line in _nmcli_log_lines(tmp_path) if line.startswith("CONNECT ")
    ]
    assert connect_lines == [
        "CONNECT device wifi connect pennair-ap-air-01 password shared-psk name pennair-fallback-client-air-01 ifname wlan0"
    ]


def test_empty_runtime_allowed_ap_hosts_override_replaces_bootstrap_allowlist(
    tmp_path: Path,
) -> None:
    result = _run_failover(
        tmp_path,
        default_policy="""\
        hostname: air-02
        fallback_ap_prefix: pennair-ap
        fallback_psk: shared-psk
        network_role: client
        ap_selection: strongest
        allowed_ap_hosts:
          - air-03
        """,
        runtime_policy_active=True,
        runtime_allowed_ap_hosts=[],
        visible_networks=[
            ("pennair-ap-air-02", 99),
            ("pennair-ap-air-03", 70),
            ("pennair-ap-air-01", 90),
        ],
    )

    assert result.returncode == 0, result.stderr or result.stdout

    connect_lines = [
        line for line in _nmcli_log_lines(tmp_path) if line.startswith("CONNECT ")
    ]
    assert connect_lines == [
        "CONNECT device wifi connect pennair-ap-air-01 password shared-psk name pennair-fallback-client-air-01 ifname wlan0"
    ]


def test_explicit_allowed_ap_hosts_still_restricts_selection(tmp_path: Path) -> None:
    result = _run_failover(
        tmp_path,
        default_policy="""\
        hostname: air-02
        fallback_ap_prefix: pennair-ap
        fallback_psk: shared-psk
        network_role: client
        ap_selection: strongest
        allowed_ap_hosts:
          - air-03
          - air-01
        """,
        runtime_policy_active=False,
        visible_networks=[
            ("pennair-ap-air-02", 99),
            ("pennair-ap-air-03", 70),
            ("pennair-ap-air-01", 20),
        ],
    )

    assert result.returncode == 0, result.stderr or result.stdout

    connect_lines = [
        line for line in _nmcli_log_lines(tmp_path) if line.startswith("CONNECT ")
    ]
    assert connect_lines == [
        "CONNECT device wifi connect pennair-ap-air-03 password shared-psk name pennair-fallback-client-air-03 ifname wlan0"
    ]


def test_switching_disabled_marker_prevents_automatic_wifi_changes(
    tmp_path: Path,
) -> None:
    result = _run_failover(
        tmp_path,
        default_policy="""\
        hostname: air-02
        fallback_ap_prefix: pennair-ap
        fallback_psk: shared-psk
        network_role: client
        ap_selection: strongest
        allowed_ap_hosts: []
        """,
        runtime_policy_active=False,
        visible_networks=[
            ("pennair-ap-air-01", 90),
        ],
        switching_disabled=True,
    )

    assert result.returncode == 0, result.stderr or result.stdout
    assert "Wi-Fi switching is disabled" in result.stdout
    assert _nmcli_log_lines(tmp_path) == [
        "nmcli -t -f DEVICE,TYPE device status",
        "nmcli -t -f NAME,TYPE connection show --active",
        "nmcli -t -f ACTIVE,SSID device wifi list ifname wlan0 --rescan no",
    ]
