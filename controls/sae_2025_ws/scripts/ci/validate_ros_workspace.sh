#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
. "$SCRIPT_DIR/common.sh"

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(ci_workspace_root)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
PYTEST_TARGETS="${PYTEST_TARGETS:-src/uav/test/test_mission_spec.py src/uav/test/test_schema_models.py src/uav/test/test_schema_registry_runtime.py src/uav/test/test_auto_launch.py src/uav/test/test_runtime_behavior.py src/uav/test/test_launch_helpers.py src/uav/test/test_fleet_launch.py src/sim/test/test_orchestration.py src/integration/test/test_config.py src/integration/test/test_deploy.py src/integration/test/test_schema.py}"
LIVE_PYTEST_TARGETS="${LIVE_PYTEST_TARGETS:-src/uav/test/test_peer_stack_reconnect.py}"
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
    src/payload src/payload_interfaces src/tools src/udp_bridge

ci_log "Building shared hardware dependencies"
colcon build \
    --packages-select payload_interfaces px4_msgs uav_interfaces actuator_msgs

ci_log "Building hardware payload package"
ci_source_workspace "$WORKSPACE_ROOT"
colcon build \
    --packages-select payload \
    --cmake-args -DBUILD_SIM=OFF

ci_log "Building udp_bridge package"
ci_source_workspace "$WORKSPACE_ROOT"
colcon build --packages-select udp_bridge

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
    python3 -m uav.runtime.schema_generator --check

ci_log "Running pytest suite"
ci_source_workspace "$WORKSPACE_ROOT"
# Ignore globally installed pytest entrypoints from unrelated packages like anyio.
PYTHONPATH="$WORKSPACE_ROOT/src/uav:$WORKSPACE_ROOT/src/sim:${PYTHONPATH:-}" \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    python3 -m pytest $PYTEST_TARGETS

ci_log "Running live peer reconnect pytest suite"
ci_source_workspace "$WORKSPACE_ROOT"
PYTHONPATH="$WORKSPACE_ROOT/src/uav:$WORKSPACE_ROOT/src/sim:${PYTHONPATH:-}" \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    python3 -m pytest $LIVE_PYTEST_TARGETS
