#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
. "$SCRIPT_DIR/common.sh"

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(ci_workspace_root)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
TEST_PACKAGES="${TEST_PACKAGES:-vehicle_common payload uav sim}"
INSTALL_DEPS="${INSTALL_DEPS:-1}"

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

ci_log "Building shared hardware dependencies"
colcon build \
    --packages-select payload_interfaces px4_msgs uav_interfaces actuator_msgs udp_bridge vehicle_common

ci_log "Building hardware payload package"
ci_source_workspace "$WORKSPACE_ROOT"
colcon build \
    --packages-select payload payload_controller \
    --cmake-args -DBUILD_SIM=OFF

ci_log "Building uav package"
ci_source_workspace "$WORKSPACE_ROOT"
colcon build --packages-select uav

ci_log "Building tools package"
ci_source_workspace "$WORKSPACE_ROOT"
colcon build --packages-select tools

ci_log "Building sim package"
ci_source_ros
colcon build --packages-select sim

ci_log "Compiling Python sources"
mapfile -t python_files < <(ci_py_files)
python3 -m py_compile "${python_files[@]}"

ci_log "Checking committed mode schema registry"
ci_source_workspace "$WORKSPACE_ROOT"
PYTHONPATH="$WORKSPACE_ROOT/src/uav:${PYTHONPATH:-}" \
    python3 -m vehicle_common.runtime.schema_generator
PYTHONPATH="$WORKSPACE_ROOT/src/uav:${PYTHONPATH:-}" \
    python3 -m vehicle_common.runtime.schema_generator --check

ci_log "Running colcon test"
ci_source_workspace "$WORKSPACE_ROOT"
# colcon discovers packages' tests, exclude live tests by marker.
colcon test \
    --packages-select $TEST_PACKAGES \
    --event-handlers console_direct+ \
    --return-code-on-test-failure \
    --pytest-args -m "not live"
colcon test-result --verbose
