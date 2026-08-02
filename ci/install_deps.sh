#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ci.conf"

apt-get update

#### install uv
echo "Installing uv..."
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"

#### install rosdeps
sudo rosdep init || true # || true guards against already initialized rosdep
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install -r -i -y --rosdistro "$ROS_DISTRO" \
    --from-paths ${PENNAIR_SAE_WS_PATH}/src

#### Install all apt packages (non rosdep)
apt-get install -y --no-install-recommends "${APT_PACKAGES[@]}"
apt-get clean
rm -rf /var/lib/apt/lists/*

#### install python dependencies into system using uv
echo "Installing pyproject dependencies globally..."
uv pip install --system --break-system-packages --no-cache -r ${PENNAIR_MONOREPO_PATH}/pyproject.toml

