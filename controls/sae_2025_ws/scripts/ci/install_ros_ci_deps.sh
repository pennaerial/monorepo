#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
. "$SCRIPT_DIR/common.sh"

ROS_DISTRO="${ROS_DISTRO:-humble}"

ci_install_system_packages \
    build-essential \
    cmake \
    git \
    libgflags-dev \
    python3-colcon-common-extensions \
    python3-dev \
    python3-opencv \
    python3-pip \
    python3-pytest \
    python3-rosdep \
    ros-"$ROS_DISTRO"-cv-bridge \
    ros-"$ROS_DISTRO"-gps-msgs \
    ros-"$ROS_DISTRO"-pluginlib \
    ros-"$ROS_DISTRO"-vision-msgs

rosdep init 2>/dev/null || true
rosdep update --rosdistro "$ROS_DISTRO"

ci_install_apriltag
ci_install_pigpio

python3 -m pip install --no-cache-dir \
    "fastapi[standard]" \
    httpx \
    pydantic \
    python-multipart
