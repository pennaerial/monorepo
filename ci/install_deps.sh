#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ci.conf"

# quick install skips the heavy dependency building and only installs
# additional ci.conf apt packages and global npm packages, pyproject.toml packages
# Also installs additional rosdeps
# A quick install is meant for a quick install step in case dependency lists change before the nightly build catches up
QUICK=false

for arg in "$@"; do
	case "$arg" in
	--quick)
		QUICK=true
		;;
	esac
done

sudo apt-get update

if [[ "$QUICK" == false ]]; then
	echo "Running full install..."

	#### install uv
	echo "Installing uv..."
	curl -LsSf https://astral.sh/uv/install.sh | sh

	#### install Node.js LTS
	echo "Installing Node.js LTS..."
	curl -fsSL https://deb.nodesource.com/setup_lts.x | bash -
	sudo apt-get install -y --no-install-recommends nodejs

    #### install PX4 dependencies (include gz, but no nuttx hardware compiler)
    echo "Running PX4 ubuntu.sh install script..."
    bash ${PENNAIR_PX4_PATH}/Tools/setup/ubuntu.sh --no-nuttx # RUNS_IN_DOCKER=true set in Dockerfile
else
    echo "Running quick install..."
fi

export PATH="$HOME/.local/bin:$PATH"

#### install rosdeps
echo "Installing rosdeps..."
sudo rosdep init || true # || true guards against already initialized rosdep
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install -r -i -y --rosdistro "$ROS_DISTRO" \
	--from-paths ${PENNAIR_SAE_WS_PATH}/src

#### Install all apt packages (non rosdep)
echo "Installing ci.conf apt packages... ${APT_PACKAGES[@]}"
sudo apt-get install -y --no-install-recommends "${APT_PACKAGES[@]}"

#### install python dependencies into system using uv
echo "Installing pyproject dependencies globally..."
uv pip install --system --break-system-packages --no-cache -r ${PENNAIR_MONOREPO_PATH}/pyproject.toml

#### install global npm packages
echo "Installing ci.conf global npm packages... ${GLOBAL_NPM[@]}"
npm install -g ${GLOBAL_NPM[@]}

#### clean up any apt package caches to reduce image size
sudo apt-get clean
rm -rf /var/lib/apt/lists/*
