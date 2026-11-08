#!/usr/bin/env bash
# Fails if the committed ros_interfaces crate is out of date with its IDL sources
# (gcs/idl/ plus the monorepo interface packages in ros_ws/install).
#
# Needs ROS sourced and the interface packages built:
#   cd ros_ws && colcon build --packages-select payload_interfaces uav_interfaces \
#     sim_interfaces pennair_planner
# Extra arguments are passed to generate_rs.py (e.g. --ros-ws-install <path>).
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

python3 gcs/generate_rs.py "$@"

changes="$(git status --porcelain -- gcs/crates/ros_interfaces gcs/idl)"
if [[ -n "$changes" ]]; then
    echo "ros_interfaces is out of date with its IDLs:" >&2
    echo "$changes" >&2
    git --no-pager diff --stat -- gcs/crates/ros_interfaces >&2
    echo >&2
    echo "Run python3 gcs/generate_rs.py and commit gcs/idl/ and gcs/crates/ros_interfaces/." >&2
    exit 1
fi
echo "ros_interfaces is up to date."
