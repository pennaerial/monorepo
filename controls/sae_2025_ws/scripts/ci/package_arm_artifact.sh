#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
. "$SCRIPT_DIR/common.sh"

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(ci_workspace_root)}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
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
3. Bootstrap the Pi once:
   scripts/hardware/bootstrap-pi.sh
4. Install the release into a deploy root:
   scripts/deploy.sh
5. Launch the hardware runtime:
   ~/pennair-deploy/bin/runtime_fleet

Configuration:
- Runtime config lives outside the artifact under DEPLOY_ROOT/config/
- The hardware entrypoint is fleet-based: runtime_fleet -> ros2 launch uav fleet.launch.py fleet_file:=...
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
if [[ -d "$WORKSPACE_ROOT/scripts/hardware" ]]; then
    mkdir -p "$DEPLOY_DIR/hardware"
    cp -a "$WORKSPACE_ROOT/scripts/hardware/." "$DEPLOY_DIR/hardware/"
fi

ci_log "Packaging build artifact"
cd "$DEPLOY_DIR"
tar -czvf "$WORKSPACE_ROOT/$ARTIFACT_NAME" .

echo "$WORKSPACE_ROOT/$ARTIFACT_NAME"
