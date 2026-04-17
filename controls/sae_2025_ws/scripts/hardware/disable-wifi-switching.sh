#!/usr/bin/env bash

set -euo pipefail

SWITCHING_DISABLE_MARKER="${PENNAIR_WIFI_SWITCHING_DISABLE_MARKER:-/run/pennair/network/disable-switching}"
FAILOVER_SERVICE="${PENNAIR_WIFI_FAILOVER_SERVICE:-pennair-wifi-failover.service}"
FAILOVER_TIMER="${PENNAIR_WIFI_FAILOVER_TIMER:-pennair-wifi-failover.timer}"

run_root() {
    if [[ "$(id -u)" -eq 0 ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

main() {
    local marker_dir
    marker_dir="$(dirname "$SWITCHING_DISABLE_MARKER")"

    run_root install -d -m 0755 "$marker_dir"
    printf 'disabled_at=%s\n' "$(date -Iseconds)" | run_root tee "$SWITCHING_DISABLE_MARKER" >/dev/null

    run_root systemctl stop "$FAILOVER_TIMER" >/dev/null 2>&1 || true
    run_root systemctl stop "$FAILOVER_SERVICE" >/dev/null 2>&1 || true

    printf 'Wi-Fi switching disabled until reboot or pennair-enable-wifi-switching.\n'
}

main "$@"
