#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
. "$SCRIPT_DIR/common.sh"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

ci_install_system_packages \
    build-essential \
    cmake \
    git \
    libgflags-dev \
    python3-colcon-common-extensions \
    python3-dev \
    python3-pip \
    python3-pytest \
    python3-rosdep

rosdep init 2>/dev/null || true
rosdep update --rosdistro "$ROS_DISTRO"

ci_install_apriltag
ci_install_pigpio
ci_ensure_pydantic_v2

python3 -m pip install --no-cache-dir \
    "fastapi[standard]" \
    httpx \
    python-multipart

# Prebuild px4_msgs so per-PR validation does not recompile it.
# Keep this commit in sync with the px4_msgs submodule gitlink
PX4_MSGS_COMMIT="86d8239e962f6939e05c3737784f60c02fa884db"
ci_build_px4_msgs "$PX4_MSGS_COMMIT"
