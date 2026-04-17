#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "$SCRIPT_DIR/deploy-lib.sh"

SERVICE_NAME="${SERVICE_NAME:-pennair-autonomy}"
DEPLOY_USER="${DEPLOY_USER:-${SUDO_USER:-${USER:-penn}}}"
_default_home="$(getent passwd "$DEPLOY_USER" | awk -F: 'NR==1 {print $6}')"
if [[ -z "${_default_home:-}" ]]; then
    _default_home="${HOME:-/home/$DEPLOY_USER}"
fi
DEPLOY_ROOT_EXPLICIT=0
if [[ -n "${DEPLOY_ROOT:-}" ]]; then
    DEPLOY_ROOT_EXPLICIT=1
fi
DEPLOY_ROOT="${DEPLOY_ROOT:-${_default_home%/}/pennair-deploy}"
ENABLE_SERVICE="${ENABLE_SERVICE:-1}"
START_SERVICE="${START_SERVICE:-0}"
INSTALL_PIGPIO="${INSTALL_PIGPIO:-1}"
INSTALL_RUNTIME_HELPER="${INSTALL_RUNTIME_HELPER:-1}"
BOOTSTRAP_FLEET_CONFIG="${BOOTSTRAP_FLEET_CONFIG:-}"
INSTALL_WIFI_FAILOVER="${INSTALL_WIFI_FAILOVER:-1}"
TRAVEL_ROUTER_SSID="${TRAVEL_ROUTER_SSID:-pennair_5G}"
TRAVEL_ROUTER_PSK="${TRAVEL_ROUTER_PSK:-pennair123!}"
DEFAULT_NETWORK_ROLE="${DEFAULT_NETWORK_ROLE:-client}"
ALLOWED_AP_HOSTS="${ALLOWED_AP_HOSTS:-}"
AP_SELECTION="${AP_SELECTION:-strongest}"
FALLBACK_AP_PREFIX="${FALLBACK_AP_PREFIX:-pennair-ap}"
FALLBACK_PSK="${FALLBACK_PSK:-pennair123!}"
LOCAL_AP_SSID="${LOCAL_AP_SSID:-}"
WIFI_FAILOVER_INTERVAL="${WIFI_FAILOVER_INTERVAL:-15}"
TRAVEL_ROUTER_PROFILE_NAME="${TRAVEL_ROUTER_PROFILE_NAME:-pennair-travel-router}"
LOCAL_AP_PROFILE_NAME="${LOCAL_AP_PROFILE_NAME:-pennair-local-ap}"
MISSION_STARTED_MARKER="${MISSION_STARTED_MARKER:-}"
NETWORK_DEFAULT_POLICY_PATH="${NETWORK_DEFAULT_POLICY_PATH:-/etc/pennair/network-default.yaml}"

usage() {
    cat <<EOF
Usage: $0 [OPTIONS]

Install hardware prereqs, the runtime helper, and the systemd unit on a Pi.

Options:
  --deploy-root PATH    Deploy root to manage (default: $DEPLOY_ROOT)
  --user NAME           Run the service as this user (default: $DEPLOY_USER)
  --service NAME        Systemd service name (default: $SERVICE_NAME)
  --config PATH         Copy a runtime_fleet.yaml into the deploy root config dir
  --travel-router-ssid SSID
                        Configure the preferred travel-router Wi-Fi SSID
  --travel-router-psk PASS
                        Password for the travel-router Wi-Fi profile
  --network-role ROLE   Default role when the router is absent: ap or client
                        (default: $DEFAULT_NETWORK_ROLE)
  --allowed-ap-hosts CSV
                        Optional fallback AP hostnames for client mode.
                        Empty means discover any visible PennAiR fallback AP
                        matching the configured prefix
  --ap-selection MODE   Fallback AP choice: strongest or ordered_then_strongest
                        (default: $AP_SELECTION)
  --fallback-psk PASS   Shared PSK for Pi-hosted fallback APs
  --fallback-ap-prefix PREFIX
                        Prefix for Pi-hosted fallback SSIDs (default: $FALLBACK_AP_PREFIX)
  --local-ap-ssid SSID  Override the local fallback AP SSID
  --no-wifi-failover    Skip NetworkManager failover profile installation
  --no-pigpio           Skip pigpio/pigpiod installation
  --no-runtime-helper   Skip installing the runtime_fleet helper script
  --enable / --no-enable
  --start / --no-start
  -h, --help            Show this help
EOF
}

run_root() {
    if [[ "$(id -u)" -eq 0 ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

install_prereqs() {
    deploy_info "Installing system prerequisites"
    deploy_preflight_time_sync run_root
    deploy_apt_update run_root
    run_root apt-get install -y curl jq tar rsync git
    deploy_ensure_uav_python_runtime run_root "$DEPLOY_USER"
}

install_wifi_prereqs() {
    if command -v nmcli >/dev/null 2>&1; then
        deploy_info "NetworkManager CLI already available"
    else
        deploy_info "Installing NetworkManager"
        deploy_preflight_time_sync run_root
        deploy_apt_update run_root
        run_root apt-get install -y network-manager
    fi

    run_root systemctl enable --now NetworkManager
}

install_pigpio() {
    if ldconfig -p 2>/dev/null | grep -q 'pigpiod_if2'; then
        deploy_info "pigpio runtime libraries already available"
        return
    fi

    deploy_info "Building and installing pigpio from source"
    local tmpdir
    tmpdir="$(mktemp -d)"
    trap 'run_root rm -rf -- '"$(printf '%q' "$tmpdir")"' >/dev/null 2>&1 || rm -rf -- '"$(printf '%q' "$tmpdir")"' >/dev/null 2>&1 || true' EXIT

    git clone --depth 1 https://github.com/joan2937/pigpio.git "$tmpdir/pigpio"
    pushd "$tmpdir/pigpio" >/dev/null
    cmake . -DBUILD_SHARED_LIBS=ON
    make -j"$(nproc)"
    run_root make install
    run_root ldconfig
    popd >/dev/null

    if ! systemctl list-unit-files pigpiod.service >/dev/null 2>&1; then
        deploy_info "Installing pigpiod systemd unit"
        run_root tee /etc/systemd/system/pigpiod.service >/dev/null <<'EOF_UNIT'
[Unit]
Description=Daemon required to control GPIO pins via pigpio

[Service]
Type=forking
ExecStart=/usr/local/bin/pigpiod
ExecStop=/bin/systemctl kill pigpiod

[Install]
WantedBy=multi-user.target
EOF_UNIT
    fi

    run_root systemctl daemon-reload
    run_root systemctl enable --now pigpiod
}

install_runtime_helper() {
    deploy_prepare_root
    deploy_install_runtime_helper "$SCRIPT_DIR/runtime_fleet.sh"
    if [[ -n "$BOOTSTRAP_FLEET_CONFIG" ]]; then
        install -m 0644 "$BOOTSTRAP_FLEET_CONFIG" "$CONFIG_DIR/runtime_fleet.yaml"
    else
        install -d "$CONFIG_DIR"
        if [[ ! -f "$CONFIG_DIR/runtime_fleet.yaml" ]]; then
            cat > "$CONFIG_DIR/runtime_fleet.yaml" <<'EOF_CONFIG'
# runtime_fleet.yaml
# Populate this with the hardware fleet/runtime contract once the backend writes
# the matching deploy-root configuration.
backend:
  kind: hardware
defaults: {}
vehicles: []
EOF_CONFIG
        fi
    fi
}

normalize_hostname() {
    local value="${1:-}"
    value="${value%.local}"
    value="${value%.}"
    printf '%s\n' "${value,,}"
}

validate_network_role() {
    case "$1" in
        ap|client)
            ;;
        *)
            deploy_error "Invalid network role '$1'. Use 'ap' or 'client'."
            ;;
    esac
}

validate_ap_selection() {
    case "$1" in
        strongest|ordered_then_strongest)
            ;;
        *)
            deploy_error "Invalid AP selection '$1'. Use 'strongest' or 'ordered_then_strongest'."
            ;;
    esac
}

wifi_device_name() {
    nmcli -t -f DEVICE,TYPE device status | awk -F: '$2 == "wifi" { print $1; exit }'
}

connection_exists() {
    local connection_name="$1"
    nmcli -t -f NAME connection show | grep -Fxq -- "$connection_name"
}

configure_travel_router_profile() {
    local device_name="$1"
    if connection_exists "$TRAVEL_ROUTER_PROFILE_NAME"; then
        deploy_info "Updating travel router profile $TRAVEL_ROUTER_PROFILE_NAME"
    else
        deploy_info "Creating travel router profile $TRAVEL_ROUTER_PROFILE_NAME"
        run_root nmcli connection add type wifi ifname "$device_name" con-name "$TRAVEL_ROUTER_PROFILE_NAME" ssid "$TRAVEL_ROUTER_SSID" >/dev/null
    fi

    run_root nmcli connection modify "$TRAVEL_ROUTER_PROFILE_NAME" \
        connection.autoconnect yes \
        connection.autoconnect-priority 100 \
        802-11-wireless.mode infrastructure \
        802-11-wireless.ssid "$TRAVEL_ROUTER_SSID" \
        ipv4.method auto \
        ipv6.method auto >/dev/null

    if [[ -n "$TRAVEL_ROUTER_PSK" ]]; then
        run_root nmcli connection modify "$TRAVEL_ROUTER_PROFILE_NAME" \
            wifi-sec.key-mgmt wpa-psk \
            wifi-sec.psk "$TRAVEL_ROUTER_PSK" >/dev/null
    fi
}

configure_local_ap_profile() {
    local device_name="$1"
    if connection_exists "$LOCAL_AP_PROFILE_NAME"; then
        deploy_info "Updating local AP profile $LOCAL_AP_PROFILE_NAME"
    else
        deploy_info "Creating local AP profile $LOCAL_AP_PROFILE_NAME"
        run_root nmcli connection add type wifi ifname "$device_name" con-name "$LOCAL_AP_PROFILE_NAME" ssid "$LOCAL_AP_SSID" >/dev/null
    fi

    run_root nmcli connection modify "$LOCAL_AP_PROFILE_NAME" \
        connection.autoconnect no \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        802-11-wireless.ssid "$LOCAL_AP_SSID" \
        ipv4.method shared \
        ipv6.method ignore >/dev/null

    run_root nmcli connection modify "$LOCAL_AP_PROFILE_NAME" \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "$FALLBACK_PSK" >/dev/null
}

write_network_default_policy() {
    local tmpfile
    local normalized_host
    normalized_host="$(normalize_hostname "$(hostname)")"
    tmpfile="$(mktemp)"

    {
        printf 'hostname: %s\n' "$normalized_host"
        printf 'deploy_root: %s\n' "$DEPLOY_ROOT"
        printf 'runtime_service_unit: %s\n' "${SERVICE_NAME}.service"
        printf 'mission_started_marker: %s\n' "$MISSION_STARTED_MARKER"
        printf 'travel_router_profile: %s\n' "$TRAVEL_ROUTER_PROFILE_NAME"
        printf 'travel_router_ssid: %s\n' "$TRAVEL_ROUTER_SSID"
        printf 'travel_router_psk: %s\n' "$TRAVEL_ROUTER_PSK"
        printf 'local_ap_profile: %s\n' "$LOCAL_AP_PROFILE_NAME"
        printf 'local_ap_ssid: %s\n' "$LOCAL_AP_SSID"
        printf 'fallback_ap_prefix: %s\n' "$FALLBACK_AP_PREFIX"
        printf 'fallback_psk: %s\n' "$FALLBACK_PSK"
        printf 'network_role: %s\n' "$DEFAULT_NETWORK_ROLE"
        printf 'ap_selection: %s\n' "$AP_SELECTION"
        printf 'allowed_ap_hosts:\n'
        if [[ -n "$ALLOWED_AP_HOSTS" ]]; then
            local host_name
            IFS=',' read -r -a _allowed_hosts <<<"$ALLOWED_AP_HOSTS"
            for host_name in "${_allowed_hosts[@]}"; do
                host_name="$(normalize_hostname "$host_name")"
                [[ -n "$host_name" ]] || continue
                printf '  - %s\n' "$host_name"
            done
        fi
    } > "$tmpfile"

    run_root install -d -m 0755 /etc/pennair
    run_root install -m 0600 "$tmpfile" "$NETWORK_DEFAULT_POLICY_PATH"
    rm -f "$tmpfile"
}

install_wifi_failover_helper() {
    local helper_path="/usr/local/bin/pennair-wifi-failover"
    local disable_path="/usr/local/bin/pennair-disable-wifi-switching"
    local enable_path="/usr/local/bin/pennair-enable-wifi-switching"
    local service_path="/etc/systemd/system/pennair-wifi-failover.service"
    local timer_path="/etc/systemd/system/pennair-wifi-failover.timer"

    deploy_info "Installing Wi-Fi failover helper"
    run_root install -m 0755 "$SCRIPT_DIR/wifi-failover.sh" "$helper_path"
    run_root install -m 0755 "$SCRIPT_DIR/disable-wifi-switching.sh" "$disable_path"
    run_root install -m 0755 "$SCRIPT_DIR/enable-wifi-switching.sh" "$enable_path"

    run_root tee "$service_path" >/dev/null <<EOF_SERVICE
[Unit]
Description=PennAiR Wi-Fi failover policy
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=oneshot
ExecStart=$helper_path
EOF_SERVICE

    run_root tee "$timer_path" >/dev/null <<EOF_TIMER
[Unit]
Description=Run PennAiR Wi-Fi failover policy periodically

[Timer]
OnBootSec=20
OnUnitActiveSec=${WIFI_FAILOVER_INTERVAL}
Unit=pennair-wifi-failover.service

[Install]
WantedBy=timers.target
EOF_TIMER

    run_root systemctl daemon-reload
    run_root systemctl enable --now pennair-wifi-failover.timer
    run_root systemctl start pennair-wifi-failover.service
}

configure_wifi_failover() {
    validate_network_role "$DEFAULT_NETWORK_ROLE"
    validate_ap_selection "$AP_SELECTION"

    if [[ -z "$TRAVEL_ROUTER_SSID" ]]; then
        deploy_warn "Skipping Wi-Fi failover setup because no travel-router SSID is configured."
        return 0
    fi

    install_wifi_prereqs

    local device_name
    device_name="$(wifi_device_name)"
    if [[ -z "$device_name" ]]; then
        deploy_error "No Wi-Fi device detected. Cannot configure Wi-Fi failover."
    fi

    if [[ -z "$LOCAL_AP_SSID" ]]; then
        LOCAL_AP_SSID="${FALLBACK_AP_PREFIX}-$(normalize_hostname "$(hostname)")"
    fi

    configure_travel_router_profile "$device_name"
    configure_local_ap_profile "$device_name"
    write_network_default_policy
    install_wifi_failover_helper
}

install_systemd_unit() {
    deploy_prepare_root
    local unit_path="/etc/systemd/system/${SERVICE_NAME}.service"
    deploy_info "Installing systemd unit ${unit_path}"
    run_root tee "$unit_path" >/dev/null <<EOF_UNIT
[Unit]
Description=SAE hardware runtime fleet
Wants=network-online.target pigpiod.service
After=network-online.target pigpiod.service

[Service]
Type=simple
User=${DEPLOY_USER}
Group=${DEPLOY_USER}
WorkingDirectory=${DEPLOY_ROOT}/current
Environment=DEPLOY_ROOT=${DEPLOY_ROOT}
Environment=RUNTIME_FLEET_CONFIG=${DEPLOY_ROOT}/config/runtime_fleet.yaml
Environment=PENNAIR_MISSION_STARTED_MARKER_PATH=${MISSION_STARTED_MARKER}
ExecStart=${DEPLOY_ROOT}/bin/runtime_fleet
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF_UNIT

    run_root systemctl daemon-reload
    if [[ "$ENABLE_SERVICE" == "1" ]]; then
        run_root systemctl enable "${SERVICE_NAME}.service"
    fi
    if [[ "$START_SERVICE" == "1" ]]; then
        run_root systemctl restart "${SERVICE_NAME}.service"
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --deploy-root)
            DEPLOY_ROOT="$2"
            DEPLOY_ROOT_EXPLICIT=1
            shift 2
            ;;
        --user)
            DEPLOY_USER="$2"
            shift 2
            ;;
        --service)
            SERVICE_NAME="$2"
            shift 2
            ;;
        --config)
            BOOTSTRAP_FLEET_CONFIG="$2"
            shift 2
            ;;
        --travel-router-ssid)
            TRAVEL_ROUTER_SSID="$2"
            shift 2
            ;;
        --travel-router-psk)
            TRAVEL_ROUTER_PSK="$2"
            shift 2
            ;;
        --network-role)
            DEFAULT_NETWORK_ROLE="$2"
            shift 2
            ;;
        --allowed-ap-hosts)
            ALLOWED_AP_HOSTS="$2"
            shift 2
            ;;
        --ap-selection)
            AP_SELECTION="$2"
            shift 2
            ;;
        --fallback-psk)
            FALLBACK_PSK="$2"
            shift 2
            ;;
        --fallback-ap-prefix)
            FALLBACK_AP_PREFIX="$2"
            shift 2
            ;;
        --local-ap-ssid)
            LOCAL_AP_SSID="$2"
            shift 2
            ;;
        --no-wifi-failover)
            INSTALL_WIFI_FAILOVER=0
            shift
            ;;
        --no-pigpio)
            INSTALL_PIGPIO=0
            shift
            ;;
        --no-runtime-helper)
            INSTALL_RUNTIME_HELPER=0
            shift
            ;;
        --enable)
            ENABLE_SERVICE=1
            shift
            ;;
        --no-enable)
            ENABLE_SERVICE=0
            shift
            ;;
        --start)
            START_SERVICE=1
            shift
            ;;
        --no-start)
            START_SERVICE=0
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            deploy_error "Unknown argument: $1"
            ;;
    esac
done

if [[ "$DEPLOY_ROOT_EXPLICIT" != "1" ]]; then
    _default_home="$(getent passwd "$DEPLOY_USER" | awk -F: 'NR==1 {print $6}')"
    if [[ -z "${_default_home:-}" ]]; then
        _default_home="${HOME:-/home/$DEPLOY_USER}"
    fi
    DEPLOY_ROOT="${_default_home%/}/pennair-deploy"
fi

if [[ -z "$MISSION_STARTED_MARKER" ]]; then
    MISSION_STARTED_MARKER="${DEPLOY_ROOT%/}/state/mission-started"
fi

deploy_init_paths

install_prereqs

if [[ "$INSTALL_WIFI_FAILOVER" == "1" ]]; then
    configure_wifi_failover
fi

if [[ "$INSTALL_PIGPIO" == "1" ]]; then
    install_pigpio
fi

if [[ "$INSTALL_RUNTIME_HELPER" == "1" ]]; then
    install_runtime_helper
fi

install_systemd_unit

deploy_info "Bootstrap complete"
deploy_status_lines
