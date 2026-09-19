#!/bin/bash
set -e

# Install mesa-utils so glxinfo is available for the postStartCommand check
apt-get update && apt-get install -y mesa-utils

# Tell git to trust the mounted repo
git config --global --add safe.directory '*'

# Fixes "bad subsitution" errors when colcon build (dash->bash)
ln -sf /bin/bash /bin/sh

# Auto-source ROS + dev_env.sh in every new bash shell (idempotent)
grep -qxF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc \
  || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

grep -qxF 'source ~/monorepo/dev_env.sh' ~/.bashrc \
  || echo 'source ~/monorepo/dev_env.sh' >> ~/.bashrc