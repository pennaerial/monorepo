#!/usr/bin/env bash

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

# Use the PX4 SITL baked into the sim image.
PX4_PREFIX="$(ci_px4_prefix)"
REPO_ROOT="$(git rev-parse --show-toplevel)"
if [[ ! -f "$PX4_PREFIX/PINNED_COMMIT" ]]; then
    ci_log "ERROR: prebuilt PX4 SITL not found at $PX4_PREFIX."
    ci_log "Use the sim CI image or rebuild it from .github/ci/ros-jazzy-sim-ci/Dockerfile."
    exit 1
fi
built_commit="$(cat "$PX4_PREFIX/PINNED_COMMIT")"
pinned_commit="$(git -C "$REPO_ROOT" ls-tree HEAD -- PX4-Autopilot \
    | awk '$1 == "160000" && $2 == "commit" { print $3; exit }')"
if [[ -n "$pinned_commit" && "$built_commit" != "$pinned_commit" ]]; then
    ci_log "ERROR: prebuilt PX4 ($built_commit) does not match the submodule pin ($pinned_commit)."
    ci_log "Update PX4_REF in .github/ci/ros-jazzy-sim-ci/Dockerfile and rebuild the sim CI image."
    exit 1
fi
ci_log "Using prebuilt PX4 SITL ($built_commit)"
PX4_PATH="$PX4_PREFIX"

# Run Gazebo/PX4 with no GUI and software rendering (handled in sim.launch.py).
export SAE_SIM_HEADLESS=1
export PX4_PATH

ci_log "Running headless sim launch_testing gate"
# xvfb-run gives gz's ogre render engine an X display in this headless env.
# TODO: revisit explicit-list vs. marker selection once a 3rd sim test exists.
xvfb-run -a python3 -m pytest -sx "$WORKSPACE_ROOT/src/uav/test/test_sim_headless.py"
xvfb-run -a python3 -m pytest -sx "$WORKSPACE_ROOT/src/uav/test/test_sim_headless_tailsitter.py"
