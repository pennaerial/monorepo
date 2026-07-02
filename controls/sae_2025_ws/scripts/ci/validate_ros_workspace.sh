#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
. "$SCRIPT_DIR/common.sh"

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(ci_workspace_root)}"

VENDORED_PACKAGES=(px4_msgs)

cd "$WORKSPACE_ROOT"

# shellcheck disable=SC1091
. "$SCRIPT_DIR/build_ros_workspace.sh"

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
