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

usage() {
    cat <<EOF
Usage: $0 [OPTIONS]

Install hardware prereqs, the runtime helper, and the systemd unit on a Pi.

Options:
  --deploy-root PATH    Deploy root to manage (default: $DEPLOY_ROOT)
  --user NAME           Run the service as this user (default: $DEPLOY_USER)
  --service NAME        Systemd service name (default: $SERVICE_NAME)
  --config PATH         Copy a runtime_fleet.yaml into the deploy root config dir
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
    run_root apt-get install -y curl jq tar rsync git python3-pip build-essential cmake

    if ! deploy_python_has_apriltag || ! deploy_python_has_pydantic_v2; then
        deploy_info "Installing UAV Python runtime packages into the user site"
        python3 -m pip install --user --upgrade apriltag "pydantic>=2,<3"
    fi
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

deploy_init_paths

if [[ "$INSTALL_PIGPIO" == "1" ]]; then
    install_prereqs
    install_pigpio
fi

if [[ "$INSTALL_RUNTIME_HELPER" == "1" ]]; then
    install_runtime_helper
fi

install_systemd_unit

deploy_info "Bootstrap complete"
deploy_status_lines
