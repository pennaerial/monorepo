#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
. "$SCRIPT_DIR/common.sh"

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(ci_workspace_root)}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# LIVE_PYTEST_TARGETS="${LIVE_PYTEST_TARGETS:-src/uav/test/test_peer_stack_reconnect.py}"
INSTALL_DEPS="${INSTALL_DEPS:-1}"

# px4_msgs is left off because it is prebuilt into CI image
WORKSPACE_PACKAGES=(
    vehicle_common payload_interfaces uav_interfaces udp_bridge
    actuator_msgs
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

  ci_log "Resolving prebuilt px4_msgs"
  PX4_MSGS_PREFIX="$(ci_px4_msgs_prefix)"
  BUILD_PX4_MSGS=1
# checks if the file /opt/px4_msgs/PINNED_COMMIT exists because that is where we stored the px4_msgs commit of the image
  if [[ -f "$PX4_MSGS_PREFIX/PINNED_COMMIT" ]]; then
      built_commit="$(cat "$PX4_MSGS_PREFIX/PINNED_COMMIT")"
      pinned_commit="$(git -C src/px4_msgs rev-parse HEAD 2>/dev/null || true)"
      if [[ -n "$pinned_commit" && "$built_commit" != "$pinned_commit" ]]; then
          ci_log "ERROR: prebuilt px4_msgs ($built_commit) does not match the checked-out submodule ($pinned_commit)."
          ci_log "Update PX4_MSGS_COMMIT in scripts/ci/install_ros_ci_deps.sh and rebuild the CI image,"
          ci_log "or run 'git submodule update --init src/px4_msgs' to match the pin."
          exit 1
      fi
      ci_log "Using prebuilt px4_msgs ($built_commit)"
      ci_source_setup "$PX4_MSGS_PREFIX/setup.sh"
      BUILD_PX4_MSGS=0
  else
      ci_log "No prebuilt px4_msgs found; building it from source"
  fi


  ci_log "Building workspace"
  build_packages=("${WORKSPACE_PACKAGES[@]}")
  if [[ "$BUILD_PX4_MSGS" == "1" ]]; then
      build_packages+=(px4_msgs)
  fi
  colcon build --packages-select "${build_packages[@]}"


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
