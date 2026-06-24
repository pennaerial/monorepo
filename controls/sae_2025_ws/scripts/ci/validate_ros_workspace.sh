#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
. "$SCRIPT_DIR/common.sh"

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(ci_workspace_root)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
INSTALL_DEPS="${INSTALL_DEPS:-1}"

WORKSPACE_PACKAGES=(
    vehicle_common payload_interfaces uav_interfaces udp_bridge
    px4_msgs actuator_msgs
    payload payload_controller uav tools sim
)
VENDORED_PACKAGES=(px4_msgs actuator_msgs)

cd "$WORKSPACE_ROOT"

if [[ "$INSTALL_DEPS" != "0" ]]; then
    "$SCRIPT_DIR/install_ros_ci_deps.sh"
fi

ci_ensure_pydantic_v2

ci_source_ros

ci_log "Installing workspace dependencies via rosdep"
ci_refresh_apt_lists
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install -r -i -y --rosdistro "$ROS_DISTRO" \
    --from-paths src/uav src/uav_interfaces src/px4_msgs src/actuator_msgs \
    src/payload src/payload_interfaces src/tools src/vehicle_common \
    src/payload_controller src/sim src/udp_bridge

ci_log "Building workspace"
colcon build --packages-select "${WORKSPACE_PACKAGES[@]}"

ci_log "Compiling Python sources"
mapfile -t python_files < <(ci_py_files)
python3 -m py_compile "${python_files[@]}"

ci_log "Checking committed mode schema registry"
ci_source_workspace "$WORKSPACE_ROOT"
SCHEMA_REGISTRY_PATH="$WORKSPACE_ROOT/src/vehicle_common/vehicle_common/runtime/mode_registry.json"
PYTHONPATH="$WORKSPACE_ROOT/src/uav:${PYTHONPATH:-}" \
    python3 -m vehicle_common.runtime.schema_generator \
    --check \
    --output "$SCHEMA_REGISTRY_PATH"

ci_log "Running colcon test"
ci_source_workspace "$WORKSPACE_ROOT"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

# Test what we built minus vendored deps; exclude live tests by marker.
colcon test \
    --packages-select "${WORKSPACE_PACKAGES[@]}" \
    --packages-skip "${VENDORED_PACKAGES[@]}" \
    --event-handlers console_direct+ \
    --return-code-on-test-failure \
    --pytest-args -m "not live"
colcon test-result --verbose
