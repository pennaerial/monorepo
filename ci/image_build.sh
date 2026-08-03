#!/usr/bin/env bash

# script that builds all 3rd party submodule dependencies from source, PX4 SITL, and ROS workspace

set -euo pipefail

#### build Dependencies modules
bash ${PENNAIR_DEPENDENCIES_PATH}/build_all.sh

#### build PX4 SITL
cd ${PENNAIR_PX4_PATH}
make px4_sitl

#### clean build of SAE 2025 workspace, build only 3rd party ROS packages
bash ${PENNAIR_SAE_WS_PATH}/scripts/ci/build_ros.sh --build-mode clean --third
