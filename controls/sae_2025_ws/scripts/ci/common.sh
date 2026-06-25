#!/usr/bin/env bash

set -euo pipefail

ci_script_dir() {
    cd "$(dirname "${BASH_SOURCE[0]}")" && pwd
}

ci_workspace_root() {
    cd "$(ci_script_dir)/../.." && pwd
}

ci_ros_distro() {
    echo "${ROS_DISTRO:-humble}"
}

ci_log() {
    echo "[ci] $*"
}

ci_root_or_sudo() {
    if [[ "$(id -u)" -eq 0 ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

ci_source_setup() {
    local setup_path="$1"
    local had_nounset=0

    case $- in
        *u*)
            had_nounset=1
            set +u
            ;;
    esac

    # shellcheck disable=SC1090
    . "$setup_path"

    if [[ "$had_nounset" == "1" ]]; then
        set -u
    fi
}

ci_source_ros() {
    ci_source_setup "/opt/ros/$(ci_ros_distro)/setup.sh"
}

ci_source_workspace() {
    local workspace_root="${1:-$(ci_workspace_root)}"
    ci_source_setup "$workspace_root/install/setup.sh"
}

ci_install_system_packages() {
    ci_log "Installing system packages"
    ci_root_or_sudo apt-get update
    ci_root_or_sudo apt-get install -y --no-install-recommends "$@"
}

ci_refresh_apt_lists() {
    ci_log "Refreshing apt package lists"
    ci_root_or_sudo apt-get update
}

ci_install_apriltag() {
    ci_log "Installing apriltag Python package"
    python3 -m pip install --no-cache-dir apriltag
    python3 -c "import apriltag; print(apriltag.__file__)"
}

ci_ensure_pip() {
    if python3 -m pip --version >/dev/null 2>&1; then
        return
    fi

    ci_log "Installing python3-pip"
    ci_install_system_packages python3-pip
}

ci_has_pydantic_v2() {
    python3 - <<'PY' >/dev/null 2>&1
try:
    import pydantic

    version = getattr(pydantic, "__version__", "0")
    major = int(version.split(".", 1)[0])
except Exception:
    raise SystemExit(1)

raise SystemExit(0 if major >= 2 else 1)
PY
}

ci_ensure_pydantic_v2() {
    if ci_has_pydantic_v2; then
        ci_log "pydantic>=2 already available"
        return
    fi

    ci_ensure_pip
    ci_log "Installing pydantic>=2 Python package"
    python3 -m pip install --no-cache-dir --upgrade "pydantic>=2,<3"
    python3 - <<'PY'
import pydantic

print(pydantic.__version__)
PY
}

ci_install_pigpio() {
    local pigpio_dir="/tmp/pigpio"

    ci_log "Building pigpio from source"
    rm -rf "$pigpio_dir"
    git clone --depth 1 https://github.com/joan2937/pigpio.git "$pigpio_dir"
    cd "$pigpio_dir"
    cmake . -DBUILD_SHARED_LIBS=ON
    make -j"$(nproc)"
    ci_root_or_sudo make install
    ci_root_or_sudo ldconfig
}

ci_py_files() {
    find src/uav src/payload src/vehicle_common src/sim src/tools -name '*.py' | sort
}

ci_px4_msgs_prefix() {
    echo "/opt/px4_msgs"
}

# Build px4_msgs commit $1 (a commit SHA), and echo the exact commit baked so validation can detect submodule drift.
ci_build_px4_msgs() {
    local commit="$1"
    local prefix=$(ci_px4_msgs_prefix)
    local ws="/tmp/px4_msgs_ws"

    ci_log "Building px4_msgs ($commit) into $prefix"
    rm -rf "$ws"
    git clone https://github.com/PX4/px4_msgs.git "$ws/src/px4_msgs"
    git -C "$ws/src/px4_msgs" checkout "$commit" #checks out the specified commit in detached head state

    (
        cd "$ws"
        ci_source_ros
        colcon build --merge-install --install-base "$prefix"
    )

    echo "$commit" > "$prefix/PINNED_COMMIT"
    rm -rf "$ws"
}
