#!/usr/bin/env bash
# Syncs ROS2 source packages to the Pi over rsync.
#
# Usage:
#   ./sync-src.sh <user@host>                                  # Deploy default packages
#   ./sync-src.sh <user@host> --packages-select pkg_a pkg_b   # Deploy specific packages
#   ./sync-src.sh <user@host> --password                       # Prompt for SSH password
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SRC_DIR="$SCRIPT_DIR/../src"

# Packages copied in a normal deploy — no sim, no Gazebo, no bridges.
DEPLOY_PACKAGES=(
    actuator_msgs
    payload
    payload_interfaces
    px4_msgs
    uav
    uav_interfaces
)

# --- Argument parsing ---
if [[ $# -eq 0 || "$1" == --* ]]; then
    echo "Usage: $0 <user@host> [--packages-select pkg...] [--password]" >&2
    exit 1
fi
REMOTE="$1"
shift

PACKAGES=("${DEPLOY_PACKAGES[@]}")
SSH_OPTS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --packages-select)
            shift
            PACKAGES=()
            while [[ $# -gt 0 && "$1" != --* ]]; do
                PACKAGES+=("$1")
                shift
            done
            ;;
        --password)
            if ! command -v sshpass &>/dev/null; then
                echo "sshpass is not installed. Install it with: sudo apt install sshpass" >&2
                exit 1
            fi
            read -rsp "SSH password for $REMOTE: " SSH_PASSWORD
            echo
            SSH_OPTS=(-e "sshpass -p '$SSH_PASSWORD' ssh")
            shift
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

if [[ ${#PACKAGES[@]} -eq 0 ]]; then
    echo "No packages selected." >&2
    exit 1
fi

# Build source list, verifying each package exists locally.
SOURCES=()
for pkg in "${PACKAGES[@]}"; do
    pkg_path="$SRC_DIR/$pkg"
    if [[ ! -d "$pkg_path" ]]; then
        echo "Package not found: $pkg_path" >&2
        exit 1
    fi
    SOURCES+=("$pkg_path")
done

echo "Syncing packages: ${PACKAGES[*]}"

rsync -rv \
    --no-times \
    --no-perms --no-owner --no-group \
    --delete \
    "${SSH_OPTS[@]}" \
    "${SOURCES[@]}" \
    "$REMOTE:~/monorepo/controls/sae_2025_ws/src/"
