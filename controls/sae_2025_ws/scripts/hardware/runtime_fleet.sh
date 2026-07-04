#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${DEPLOY_ROOT:-}" ]]; then
    DEPLOY_ROOT="${DEPLOY_ROOT%/}"
else
    DEPLOY_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
    if [[ ! -d "$DEPLOY_ROOT/current" && ! -d "$DEPLOY_ROOT/releases" ]]; then
        DEPLOY_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
    fi
fi

CURRENT_RELEASE="${CURRENT_RELEASE:-$DEPLOY_ROOT/current}"
RUNTIME_CONFIG="${RUNTIME_FLEET_CONFIG:-$DEPLOY_ROOT/config/runtime_fleet.yaml}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

info() {
    printf '[INFO] %s\n' "$*"
}

error() {
    printf '[ERROR] %s\n' "$*" >&2
    exit 1
}

if [[ ! -f "$CURRENT_RELEASE/install/setup.bash" ]]; then
    error "Current release is missing: $CURRENT_RELEASE/install/setup.bash"
fi

if [[ ! -f "$RUNTIME_CONFIG" ]]; then
    error "Runtime fleet config not found: $RUNTIME_CONFIG"
fi

if [[ -f "$ROS_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
fi

# shellcheck disable=SC1090
source "$CURRENT_RELEASE/install/setup.bash"

if [[ -n "${RUNTIME_FLEET_CMD:-}" ]]; then
    info "Starting configured runtime command"
    exec bash -lc "$RUNTIME_FLEET_CMD"
fi

if ros2 pkg executables uav 2>/dev/null | awk '$1 == "runtime_fleet" { found = 1 } END { exit(found ? 0 : 1) }'; then
    info "Starting runtime_fleet ROS entrypoint"
    exec ros2 run uav runtime_fleet "$@"
fi

info "Falling back to fleet.launch.py with ${RUNTIME_CONFIG}"
exec ros2 launch uav fleet.launch.py fleet_file:="$RUNTIME_CONFIG" "$@"
