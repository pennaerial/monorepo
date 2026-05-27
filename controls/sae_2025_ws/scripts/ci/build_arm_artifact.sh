#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
. "$SCRIPT_DIR/common.sh"

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(ci_workspace_root)}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

cd "$WORKSPACE_ROOT"

ci_ensure_pydantic_v2
ci_source_ros

ci_log "Installing workspace dependencies via rosdep"
ci_refresh_apt_lists
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install -r -i -y --rosdistro "$ROS_DISTRO" \
    --from-paths src/uav src/uav_interfaces src/px4_msgs src/actuator_msgs \
    src/payload src/payload_interfaces src/tools src/vehicle_common src/payload_controller

ci_log "Building shared hardware dependencies"
colcon build \
    --packages-select payload_interfaces px4_msgs uav_interfaces actuator_msgs vehicle_common

ci_log "Building payload packages"
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
