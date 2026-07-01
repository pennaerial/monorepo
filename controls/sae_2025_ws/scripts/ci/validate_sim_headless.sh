#!/usr/bin/env bash

# Headless-sim CI validation: launch the full UAV sim stack with no display,
# assert the vehicle arms / takes off / lands via sim_smoke_test.py, then tear
# everything down. Intended as the single entrypoint a sim-validate workflow
# calls, mirroring how validate_ros_workspace.sh is the entrypoint for unit CI.

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
. "$SCRIPT_DIR/common.sh"

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(ci_workspace_root)}"

cd "$WORKSPACE_ROOT"
ci_source_ros

# px4_msgs is prebuilt into the image as a separate overlay, not the workspace.
ci_source_setup "$(ci_px4_msgs_prefix)/setup.sh"

# sim_interfaces (needs ros_gz) is used by the world/scoring nodes
colcon build --packages-select sim_interfaces
ci_source_workspace "$WORKSPACE_ROOT"

# Prefer the PX4 SITL baked into the sim image; build from source if absent.
PX4_PREFIX="$(ci_px4_prefix)"
PX4_SUBMODULE="$(git rev-parse --show-toplevel)/PX4-Autopilot"
if [[ -f "$PX4_PREFIX/PINNED_COMMIT" ]]; then
    built_commit="$(cat "$PX4_PREFIX/PINNED_COMMIT")"
    pinned_commit="$(git -C "$PX4_SUBMODULE" rev-parse HEAD 2>/dev/null || true)"
    if [[ -n "$pinned_commit" && "$built_commit" != "$pinned_commit" ]]; then
        ci_log "ERROR: prebuilt PX4 ($built_commit) does not match the checked-out submodule ($pinned_commit)."
        ci_log "Update PX4_REF in .github/ci/ros-jazzy-sim-ci/Dockerfile and rebuild the sim CI image."
        exit 1
    fi
    ci_log "Using prebuilt PX4 SITL ($built_commit)"
    PX4_PATH="$PX4_PREFIX"
else
    ci_log "No prebuilt PX4 SITL found; building it from source"
    ci_build_px4 "$PX4_SUBMODULE"
    PX4_PATH="$PX4_SUBMODULE"
fi

# Run Gazebo/PX4 with no GUI and software rendering (handled in sim.launch.py).
export SAE_SIM_HEADLESS=1

LAUNCH_LOG="$(mktemp)"
LAUNCH_PGID=""

cleanup() {
    ci_log "Tearing down sim"
    # Kill the whole launch process group: ros2 launch spawns gz, PX4, the DDS
    # agent, and node processes as children.
    if [[ -n "$LAUNCH_PGID" ]]; then
        kill -- "-$LAUNCH_PGID" 2>/dev/null || true
    fi
    # Safety net for anything that detached from the group.
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "px4_sitl_default/bin/px4" 2>/dev/null || true
    pkill -f "MicroXRCEAgent" 2>/dev/null || true
}
trap cleanup EXIT

ci_log "Launching headless sim (log: $LAUNCH_LOG)"
# setsid puts the launch in its own process group so cleanup can kill it whole.
# xvfb-run gives gz's ogre render engine an X display in this headless env.
setsid xvfb-run -a ros2 launch uav main.launch.py px4_path:="$PX4_PATH" >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PGID=$(ps -o pgid= "$!" | tr -d ' ')

ci_log "Running sim smoke test"
if python3 "$SCRIPT_DIR/sim_smoke_test.py"; then
    ci_log "Sim smoke test PASSED"
else
    status=$?
    ci_log "Sim smoke test FAILED (exit $status). Launch log follows:"
    cat "$LAUNCH_LOG" >&2
    exit "$status"
fi
