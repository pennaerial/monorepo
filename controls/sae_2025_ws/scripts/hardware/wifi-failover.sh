#!/usr/bin/env bash

set -euo pipefail

DEFAULT_POLICY_PATH="${PENNAIR_NETWORK_DEFAULT_POLICY:-/etc/pennair/network-default.yaml}"
STATUS_ONLY=0

if [[ "${1:-}" == "--status" ]]; then
    STATUS_ONLY=1
fi

log() {
    printf '[INFO] %s\n' "$*"
}

warn() {
    printf '[WARN] %s\n' "$*" >&2
}

normalize_hostname() {
    local value="${1:-}"
    value="${value%.local}"
    value="${value%.}"
    printf '%s\n' "${value,,}"
}

load_policy_exports() {
    local policy_path="$1"
    local prefix="$2"

    [[ -f "$policy_path" ]] || return 0

    python3 - "$policy_path" "$prefix" <<'PY'
from __future__ import annotations

import re
import shlex
import sys
from pathlib import Path


def parse_scalar(raw: str) -> str:
    value = raw.strip()
    if not value:
        return ""
    if value[0] == value[-1] and value[0] in {"'", '"'}:
        return value[1:-1]
    return value


path = Path(sys.argv[1])
prefix = sys.argv[2]
data: dict[str, object] = {}
current_list_key: str | None = None

for raw_line in path.read_text(encoding="utf-8").splitlines():
    stripped = raw_line.strip()
    if not stripped or stripped.startswith("#"):
        continue

    if stripped.startswith("- "):
        if current_list_key is None:
            continue
        items = data.setdefault(current_list_key, [])
        assert isinstance(items, list)
        items.append(parse_scalar(stripped[2:]))
        continue

    current_list_key = None
    if ":" not in stripped:
        continue

    key, value = stripped.split(":", 1)
    key = re.sub(r"[^A-Za-z0-9]+", "_", key.strip()).upper()
    value = value.strip()
    if value:
        if value == "[]":
            data[key] = []
        else:
            data[key] = parse_scalar(value)
    else:
        data[key] = []
        current_list_key = key

for key, value in data.items():
    variable_name = f"{prefix}{key}"
    if isinstance(value, list):
        joined = " ".join(shlex.quote(str(item)) for item in value)
        print(f"{variable_name}=({joined})")
    else:
        print(f"{variable_name}={shlex.quote(str(value))}")
PY
}

connection_exists() {
    local connection_name="$1"
    nmcli -t -f NAME connection show | grep -Fxq -- "$connection_name"
}

wifi_device() {
    nmcli -t -f DEVICE,TYPE device status | awk -F: '$2 == "wifi" { print $1; exit }'
}

active_wifi_connection() {
    nmcli -t -f NAME,TYPE connection show --active | awk -F: '$2 == "802-11-wireless" { print $1; exit }'
}

active_wifi_ssid() {
    local device_name="$1"
    nmcli -t -f ACTIVE,SSID device wifi list ifname "$device_name" --rescan no 2>/dev/null | awk -F: '$1 == "yes" { print $2; exit }'
}

scan_visible_networks() {
    local device_name="$1"
    nmcli -t -f SSID,SIGNAL device wifi list ifname "$device_name" --rescan auto 2>/dev/null || true
}

policy_has_key() {
    local policy_path="$1"
    local key="$2"
    grep -Eq "^[[:space:]]*${key}[[:space:]]*:" "$policy_path"
}

ensure_travel_router_connection() {
    local device_name="$1"
    local profile_name="$2"
    local ssid="$3"
    local password="${4:-}"

    if connection_exists "$profile_name"; then
        nmcli connection up "$profile_name" ifname "$device_name" >/dev/null
    else
        if [[ -n "$password" ]]; then
            nmcli device wifi connect "$ssid" password "$password" name "$profile_name" ifname "$device_name" >/dev/null
        else
            nmcli device wifi connect "$ssid" name "$profile_name" ifname "$device_name" >/dev/null
        fi
        nmcli connection modify "$profile_name" connection.autoconnect yes connection.autoconnect-priority 100 >/dev/null
    fi
}

ensure_client_connection() {
    local device_name="$1"
    local profile_name="$2"
    local ssid="$3"
    local password="$4"

    if connection_exists "$profile_name"; then
        if ! nmcli connection up "$profile_name" ifname "$device_name" >/dev/null 2>&1; then
            nmcli connection delete "$profile_name" >/dev/null 2>&1 || true
        else
            return 0
        fi
    fi

    nmcli device wifi connect "$ssid" password "$password" name "$profile_name" ifname "$device_name" >/dev/null
    nmcli connection modify "$profile_name" connection.autoconnect no >/dev/null
}

derive_ap_ssid() {
    local prefix="$1"
    local host_name="$2"
    printf '%s-%s\n' "$prefix" "$(normalize_hostname "$host_name")"
}

choose_fallback_host() {
    local selection="${1:-strongest}"
    shift || true
    local ordered_hosts=("$@")
    local best_host=""
    local best_signal=-1
    local host_name ssid signal

    if [[ "$selection" == "ordered_then_strongest" ]]; then
        for host_name in "${ordered_hosts[@]}"; do
            [[ -n "$host_name" ]] || continue
            ssid="$(derive_ap_ssid "$DEFAULT_FALLBACK_AP_PREFIX" "$host_name")"
            if [[ -n "${VISIBLE_SIGNAL[$ssid]:-}" ]]; then
                printf '%s\n' "$host_name"
                return 0
            fi
        done
    fi

    for host_name in "${ordered_hosts[@]}"; do
        [[ -n "$host_name" ]] || continue
        ssid="$(derive_ap_ssid "$DEFAULT_FALLBACK_AP_PREFIX" "$host_name")"
        signal="${VISIBLE_SIGNAL[$ssid]:--1}"
        if (( signal > best_signal )); then
            best_signal="$signal"
            best_host="$host_name"
        fi
    done

    printf '%s\n' "$best_host"
}

discover_visible_fallback_hosts() {
    local prefix="$1"
    local self_host="$2"
    local ssid host_name

    for ssid in "${!VISIBLE_SIGNAL[@]}"; do
        [[ -n "$ssid" ]] || continue
        [[ "$ssid" == "${prefix}-"* ]] || continue
        host_name="${ssid#${prefix}-}"
        host_name="$(normalize_hostname "$host_name")"
        [[ -n "$host_name" ]] || continue
        [[ "$host_name" == "$self_host" ]] && continue
        printf '%s\n' "$host_name"
    done
}

declare -A VISIBLE_SIGNAL=()
DEFAULT_ALLOWED_AP_HOSTS=()
RUNTIME_ALLOWED_AP_HOSTS=()

if [[ ! -f "$DEFAULT_POLICY_PATH" ]]; then
    exit 0
fi

eval "$(load_policy_exports "$DEFAULT_POLICY_PATH" "DEFAULT_")"

DEFAULT_HOSTNAME="$(normalize_hostname "${DEFAULT_HOSTNAME:-$(hostname)}")"
DEFAULT_TRAVEL_ROUTER_PROFILE="${DEFAULT_TRAVEL_ROUTER_PROFILE:-pennair-travel-router}"
DEFAULT_TRAVEL_ROUTER_SSID="${DEFAULT_TRAVEL_ROUTER_SSID:-}"
DEFAULT_TRAVEL_ROUTER_PSK="${DEFAULT_TRAVEL_ROUTER_PSK:-}"
DEFAULT_LOCAL_AP_PROFILE="${DEFAULT_LOCAL_AP_PROFILE:-pennair-local-ap}"
DEFAULT_LOCAL_AP_SSID="${DEFAULT_LOCAL_AP_SSID:-$(derive_ap_ssid "${DEFAULT_FALLBACK_AP_PREFIX:-pennair-ap}" "$DEFAULT_HOSTNAME")}"
DEFAULT_FALLBACK_AP_PREFIX="${DEFAULT_FALLBACK_AP_PREFIX:-pennair-ap}"
DEFAULT_FALLBACK_PSK="${DEFAULT_FALLBACK_PSK:-pennair123!}"
DEFAULT_NETWORK_ROLE="${DEFAULT_NETWORK_ROLE:-client}"
DEFAULT_AP_SELECTION="${DEFAULT_AP_SELECTION:-strongest}"
DEFAULT_RUNTIME_SERVICE_UNIT="${DEFAULT_RUNTIME_SERVICE_UNIT:-pennair-autonomy.service}"
DEFAULT_MISSION_STARTED_MARKER="${DEFAULT_MISSION_STARTED_MARKER:-/run/pennair/network/mission-started}"
DEFAULT_DEPLOY_ROOT="${DEFAULT_DEPLOY_ROOT:-/home/penn/pennair-deploy}"

WIFI_DEVICE="$(wifi_device)"
if [[ -z "$WIFI_DEVICE" ]]; then
    warn "No Wi-Fi device detected; skipping Wi-Fi failover."
    exit 0
fi

RUNTIME_POLICY_PATH="${DEFAULT_DEPLOY_ROOT%/}/config/network-policy.yaml"
RUNTIME_ACTIVE=0
SERVICE_ACTIVE=0
if systemctl is-active --quiet "$DEFAULT_RUNTIME_SERVICE_UNIT" 2>/dev/null; then
    SERVICE_ACTIVE=1
fi
if [[ "$SERVICE_ACTIVE" == "1" && -f "$RUNTIME_POLICY_PATH" ]]; then
    eval "$(load_policy_exports "$RUNTIME_POLICY_PATH" "RUNTIME_")"
    RUNTIME_ACTIVE=1
fi

RUNTIME_ALLOWED_AP_HOSTS_CONFIGURED=0
if [[ "$RUNTIME_ACTIVE" == "1" ]] && policy_has_key "$RUNTIME_POLICY_PATH" "allowed_ap_hosts"; then
    RUNTIME_ALLOWED_AP_HOSTS_CONFIGURED=1
fi

EFFECTIVE_ROLE="$DEFAULT_NETWORK_ROLE"
EFFECTIVE_AP_SELECTION="$DEFAULT_AP_SELECTION"
EFFECTIVE_ALLOWED_AP_HOSTS=("${DEFAULT_ALLOWED_AP_HOSTS[@]}")
if [[ "$RUNTIME_ACTIVE" == "1" ]]; then
    if [[ -n "${RUNTIME_NETWORK_ROLE:-}" && "${RUNTIME_NETWORK_ROLE}" != "default" ]]; then
        EFFECTIVE_ROLE="$RUNTIME_NETWORK_ROLE"
    fi
    if [[ -n "${RUNTIME_AP_SELECTION:-}" ]]; then
        EFFECTIVE_AP_SELECTION="$RUNTIME_AP_SELECTION"
    fi
    if [[ "$RUNTIME_ALLOWED_AP_HOSTS_CONFIGURED" == "1" ]]; then
        EFFECTIVE_ALLOWED_AP_HOSTS=("${RUNTIME_ALLOWED_AP_HOSTS[@]}")
    fi
fi

MISSION_STARTED=0
if [[ "$SERVICE_ACTIVE" == "1" && -f "$DEFAULT_MISSION_STARTED_MARKER" ]]; then
    MISSION_STARTED=1
fi

CURRENT_CONNECTION="$(active_wifi_connection)"
if [[ -n "$CURRENT_CONNECTION" && "$CURRENT_CONNECTION" == "$DEFAULT_TRAVEL_ROUTER_PROFILE" ]]; then
    CURRENT_MODE="travel-router"
elif [[ -n "$CURRENT_CONNECTION" && "$CURRENT_CONNECTION" == "$DEFAULT_LOCAL_AP_PROFILE" ]]; then
    CURRENT_MODE="local-ap"
else
    CURRENT_MODE="other"
fi

while IFS=: read -r ssid signal; do
    [[ -n "$ssid" ]] || continue
    signal="${signal:-0}"
    if [[ -z "${VISIBLE_SIGNAL[$ssid]:-}" || "${VISIBLE_SIGNAL[$ssid]}" -lt "$signal" ]]; then
        VISIBLE_SIGNAL["$ssid"]="$signal"
    fi
done < <(scan_visible_networks "$WIFI_DEVICE")

TRAVEL_ROUTER_ALLOWED=1
if [[ "$MISSION_STARTED" == "1" ]]; then
    TRAVEL_ROUTER_ALLOWED=0
fi

ACTIVE_WIFI_SSID="$(active_wifi_ssid "$WIFI_DEVICE")"

if [[ "$STATUS_ONLY" == "1" ]]; then
    export PENNAIR_WIFI_STATUS_CURRENT_CONNECTION="${CURRENT_CONNECTION:-}"
    export PENNAIR_WIFI_STATUS_CURRENT_MODE="$CURRENT_MODE"
    export PENNAIR_WIFI_STATUS_CURRENT_WIFI="${ACTIVE_WIFI_SSID:-}"
    export PENNAIR_WIFI_STATUS_IS_HOTSPOT="$([[ "$CURRENT_MODE" == "local-ap" ]] && printf 'true' || printf 'false')"
    export PENNAIR_WIFI_STATUS_EFFECTIVE_ROLE="$EFFECTIVE_ROLE"
    export PENNAIR_WIFI_STATUS_POLICY_SOURCE="$([[ "$RUNTIME_ACTIVE" == "1" ]] && printf 'runtime_override' || printf 'bootstrap')"
    export PENNAIR_WIFI_STATUS_RUNTIME_ACTIVE="$([[ "$SERVICE_ACTIVE" == "1" ]] && printf 'true' || printf 'false')"
    export PENNAIR_WIFI_STATUS_MISSION_STARTED="$([[ "$MISSION_STARTED" == "1" ]] && printf 'true' || printf 'false')"
    export PENNAIR_WIFI_STATUS_TRAVEL_ROUTER_LOCKED="$([[ "$TRAVEL_ROUTER_ALLOWED" == "1" ]] && printf 'false' || printf 'true')"
    export PENNAIR_WIFI_STATUS_LOCAL_AP_PROFILE="$DEFAULT_LOCAL_AP_PROFILE"
    export PENNAIR_WIFI_STATUS_TRAVEL_ROUTER_PROFILE="$DEFAULT_TRAVEL_ROUTER_PROFILE"
    export PENNAIR_WIFI_STATUS_ALLOWED_AP_HOSTS="$(printf '%s\n' "${EFFECTIVE_ALLOWED_AP_HOSTS[@]:-}")"
    python3 - <<'PY'
from __future__ import annotations

import json
import os

allowed = [line for line in os.environ.get("PENNAIR_WIFI_STATUS_ALLOWED_AP_HOSTS", "").splitlines() if line]
payload = {
    "current_connection": os.environ.get("PENNAIR_WIFI_STATUS_CURRENT_CONNECTION", ""),
    "current_mode": os.environ.get("PENNAIR_WIFI_STATUS_CURRENT_MODE", ""),
    "current_wifi": os.environ.get("PENNAIR_WIFI_STATUS_CURRENT_WIFI", "") or None,
    "is_hotspot": os.environ.get("PENNAIR_WIFI_STATUS_IS_HOTSPOT", "false") == "true",
    "effective_role": os.environ.get("PENNAIR_WIFI_STATUS_EFFECTIVE_ROLE", ""),
    "policy_source": os.environ.get("PENNAIR_WIFI_STATUS_POLICY_SOURCE", ""),
    "runtime_active": os.environ.get("PENNAIR_WIFI_STATUS_RUNTIME_ACTIVE", "false") == "true",
    "mission_started": os.environ.get("PENNAIR_WIFI_STATUS_MISSION_STARTED", "false") == "true",
    "travel_router_locked": os.environ.get("PENNAIR_WIFI_STATUS_TRAVEL_ROUTER_LOCKED", "false") == "true",
    "local_ap_profile": os.environ.get("PENNAIR_WIFI_STATUS_LOCAL_AP_PROFILE", "") or None,
    "travel_router_profile": os.environ.get("PENNAIR_WIFI_STATUS_TRAVEL_ROUTER_PROFILE", "") or None,
    "allowed_ap_hosts": allowed,
}
print(json.dumps(payload))
PY
    exit 0
fi

if [[ "$TRAVEL_ROUTER_ALLOWED" == "1" && -n "$DEFAULT_TRAVEL_ROUTER_SSID" && -n "${VISIBLE_SIGNAL[$DEFAULT_TRAVEL_ROUTER_SSID]:-}" ]]; then
    if [[ "$CURRENT_MODE" != "travel-router" ]]; then
        log "Activating travel router profile ${DEFAULT_TRAVEL_ROUTER_PROFILE}"
        ensure_travel_router_connection \
            "$WIFI_DEVICE" \
            "$DEFAULT_TRAVEL_ROUTER_PROFILE" \
            "$DEFAULT_TRAVEL_ROUTER_SSID" \
            "$DEFAULT_TRAVEL_ROUTER_PSK"
    fi
    exit 0
fi

if [[ "$EFFECTIVE_ROLE" == "ap" ]]; then
    if [[ "$CURRENT_MODE" != "local-ap" ]]; then
        log "Activating fallback AP ${DEFAULT_LOCAL_AP_PROFILE}"
        nmcli connection up "$DEFAULT_LOCAL_AP_PROFILE" ifname "$WIFI_DEVICE" >/dev/null
    fi
    exit 0
fi

if [[ "$EFFECTIVE_ROLE" != "client" ]]; then
    warn "Unknown network role '${EFFECTIVE_ROLE}', skipping Wi-Fi failover."
    exit 0
fi

if [[ -z "$DEFAULT_FALLBACK_PSK" ]]; then
    warn "Fallback PSK is not configured; cannot join fallback APs."
    exit 0
fi

FILTERED_ALLOWED_AP_HOSTS=()
DISCOVERY_MODE=0
for host_name in "${EFFECTIVE_ALLOWED_AP_HOSTS[@]}"; do
    host_name="$(normalize_hostname "$host_name")"
    [[ -n "$host_name" ]] || continue
    [[ "$host_name" == "$DEFAULT_HOSTNAME" ]] && continue
    FILTERED_ALLOWED_AP_HOSTS+=("$host_name")
done

if [[ "${#FILTERED_ALLOWED_AP_HOSTS[@]}" -eq 0 ]]; then
    DISCOVERY_MODE=1
    mapfile -t FILTERED_ALLOWED_AP_HOSTS < <(
        discover_visible_fallback_hosts "$DEFAULT_FALLBACK_AP_PREFIX" "$DEFAULT_HOSTNAME"
    )
    if [[ "${#FILTERED_ALLOWED_AP_HOSTS[@]}" -eq 0 ]]; then
        warn "No visible PennAiR fallback APs matching prefix '${DEFAULT_FALLBACK_AP_PREFIX}' were found."
        exit 0
    fi
fi

SELECTION_MODE="$EFFECTIVE_AP_SELECTION"
if [[ "$DISCOVERY_MODE" == "1" ]]; then
    SELECTION_MODE="strongest"
fi

SELECTED_HOST="$(choose_fallback_host "$SELECTION_MODE" "${FILTERED_ALLOWED_AP_HOSTS[@]}")"
if [[ -z "$SELECTED_HOST" ]]; then
    warn "No allowed fallback APs are currently visible."
    exit 0
fi

SELECTED_SSID="$(derive_ap_ssid "$DEFAULT_FALLBACK_AP_PREFIX" "$SELECTED_HOST")"
SELECTED_PROFILE="pennair-fallback-client-$(normalize_hostname "$SELECTED_HOST")"

if [[ "$CURRENT_MODE" == "other" ]]; then
    if [[ "$ACTIVE_WIFI_SSID" == "$SELECTED_SSID" ]]; then
        exit 0
    fi
fi

log "Activating fallback client connection ${SELECTED_PROFILE} (${SELECTED_SSID})"
ensure_client_connection "$WIFI_DEVICE" "$SELECTED_PROFILE" "$SELECTED_SSID" "$DEFAULT_FALLBACK_PSK"
