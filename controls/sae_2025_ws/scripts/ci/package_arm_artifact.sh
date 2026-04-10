#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
. "$SCRIPT_DIR/common.sh"

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(ci_workspace_root)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SHORT_SHA="${SHORT_SHA:-$(git -C "$WORKSPACE_ROOT" rev-parse --short HEAD 2>/dev/null || echo "${GITHUB_SHA:0:7}")}"
BUILD_TIME="${BUILD_TIME:-$(date -u +%Y%m%d-%H%M%S)}"
BRANCH_NAME="${BRANCH_NAME:-$(git -C "$WORKSPACE_ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "${GITHUB_REF_NAME:-unknown}")}"
ARTIFACT_NAME="${ARTIFACT_NAME:-ros2-build-${SHORT_SHA}.tar.gz}"
DEPLOY_DIR="${DEPLOY_DIR:-$WORKSPACE_ROOT/deploy}"
INSTALL_DIR="${INSTALL_DIR:-$WORKSPACE_ROOT/install}"

cd "$WORKSPACE_ROOT"

if [[ ! -d "$INSTALL_DIR" ]]; then
    echo "Expected install tree at '$INSTALL_DIR'." >&2
    exit 1
fi

ci_log "Writing build metadata"
cat > "$INSTALL_DIR/BUILD_INFO.txt" <<EOF
Build Information
=================
Commit SHA: ${GITHUB_SHA:-$(git -C "$WORKSPACE_ROOT" rev-parse HEAD)}
Short SHA: ${SHORT_SHA}
Build Time: ${BUILD_TIME}
Branch: ${BRANCH_NAME}
ROS Distro: ${ROS_DISTRO}
Target Architecture: arm64 (Raspberry Pi compatible)

Deployment Instructions:
1. Extract to your Raspberry Pi
2. cd to the extracted directory
3. Install runtime deps:
   sudo apt-get update
   sudo apt-get install -y python3-pip build-essential cmake git
   python3 -m pip install --user apriltag
4. Install pigpio and start pigpiod:
   cd /tmp
   git clone --depth 1 https://github.com/joan2937/pigpio.git
   cd pigpio
   cmake . -DBUILD_SHARED_LIBS=ON
   make -j$(nproc)
   sudo make install
   sudo ldconfig
   sudo /usr/local/bin/pigpiod
5. Source the setup: source install/setup.bash
6. Run: ros2 launch uav main.launch.py

Configuration:
- Integration edits the installed launch params and mission YAMLs under install/uav/share/uav/
- Launch overrides still use ROS 2 launch arguments such as mission_name:=..., vehicle_name:=..., and px4_path:=...
EOF

ci_log "Preparing runtime files"
rm -rf "$DEPLOY_DIR"
mkdir -p "$DEPLOY_DIR"
cp -a "$INSTALL_DIR" "$DEPLOY_DIR/"
cp "$WORKSPACE_ROOT/src/uav/launch/launch_params_hardware.yaml" \
    "$DEPLOY_DIR/install/uav/share/uav/launch/launch_params.yaml"
if [[ -f "$WORKSPACE_ROOT/scripts/deploy.sh" ]]; then
    cp "$WORKSPACE_ROOT/scripts/deploy.sh" "$DEPLOY_DIR/"
fi

ci_log "Packaging build artifact"
cd "$DEPLOY_DIR"
tar -czvf "$WORKSPACE_ROOT/$ARTIFACT_NAME" .

echo "$WORKSPACE_ROOT/$ARTIFACT_NAME"
