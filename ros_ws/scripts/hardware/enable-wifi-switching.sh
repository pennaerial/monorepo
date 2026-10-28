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
    run_root rm -f "$SWITCHING_DISABLE_MARKER"
    run_root systemctl start "$FAILOVER_TIMER" >/dev/null 2>&1 || true
    run_root systemctl start "$FAILOVER_SERVICE" >/dev/null 2>&1 || true

    printf 'Wi-Fi switching enabled.\n'
}

main "$@"
