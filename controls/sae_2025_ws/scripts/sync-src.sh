#!/usr/bin/env bash
# Dev-only helper that syncs ROS2 source packages to one or more Pis over rsync.
# This does not create a release-managed deployment under ~/pennair-deploy and is
# not the supported production deploy path.
#
# Usage:
#   ./sync-src.sh <user@host> [<user@host2> ...]                          # Deploy default packages
#   ./sync-src.sh <user@host> --packages-select pkg_a pkg_b               # Deploy specific packages
#   ./sync-src.sh <user@host> --password                                   # Prompt for SSH password
#   ./sync-src.sh <user@host> --sync-time                                  # Sync Pi clock from laptop before deploy
#   ./sync-src.sh pi@10.42.0.2 pi@10.42.0.3 --sync-time                   # Sync to multiple Pis in parallel
set -euo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SRC_DIR="$SCRIPT_DIR/../src"

# Packages copied in a normal deploy — no sim, no Gazebo, no bridges.
DEPLOY_PACKAGES=(
	actuator_msgs
	payload
	payload_interfaces
	px4_msgs
	uav
	uav_interfaces
	udp_bridge
)

# rsync exclude patterns — mirrors .gitignore "noise" without skipping
# anything source-meaningful (config files, YAMLs, launch files, etc.)
RSYNC_EXCLUDES=(
	--exclude='__pycache__/'
	--exclude='*.pyc'
	--exclude='*.pyo'
	--exclude='*.egg-info/'
	--exclude='.pytest_cache/'
	--exclude='.ruff_cache/'
	--exclude='*.egg-link'
	--exclude='.eggs/'
	--exclude='dist/'
	--exclude='build/'
	--exclude='.git/'
	--exclude='.venv/'
	--exclude='venv/'
	--exclude='env/'
	--exclude='.env'
	--exclude='*.log'
	--exclude='.DS_Store'
	--exclude='*.swp'
	--exclude='*.swo'
)

# --- Argument parsing ---
REMOTES=()
PACKAGES=("${DEPLOY_PACKAGES[@]}")
SSH_PASS=""
SYNC_TIME=false

if [[ $# -eq 0 ]]; then
	echo "Usage: $0 <user@host> [<user@host2> ...] [--packages-select pkg...] [--password] [--sync-time]" >&2
	exit 1
fi

# Collect all leading positional args as remote targets (before any -- flag)
while [[ $# -gt 0 && "$1" != --* ]]; do
	REMOTES+=("$1")
	shift
done

if [[ ${#REMOTES[@]} -eq 0 ]]; then
	echo "Error: at least one <user@host> target is required." >&2
	exit 1
fi

while [[ $# -gt 0 ]]; do
	case "$1" in
	--packages-select)
		shift
		PACKAGES=()
		while [[ $# -gt 0 && "$1" != --* ]]; do
			PACKAGES+=("$1")
			shift
		done
		;;
	--password)
		if ! command -v sshpass &>/dev/null; then
			echo "sshpass is not installed. Install it with: sudo apt install sshpass" >&2
			exit 1
		fi
		read -rsp "SSH password (used for all targets): " SSH_PASS
		echo
		shift
		;;
	--sync-time)
		SYNC_TIME=true
		shift
		;;
	*)
		echo "Unknown argument: $1" >&2
		exit 1
		;;
	esac
done

if [[ ${#PACKAGES[@]} -eq 0 ]]; then
	echo "No packages selected." >&2
	exit 1
fi

# Build source list, verifying each package exists locally.
SOURCES=()
for pkg in "${PACKAGES[@]}"; do
	pkg_path="$SRC_DIR/$pkg"
	if [[ ! -d "$pkg_path" ]]; then
		echo "Package not found: $pkg_path" >&2
		exit 1
	fi
	SOURCES+=("$pkg_path")
done

# Build rsync command prefix (with or without sshpass)
_rsync_to() {
	local remote="$1"
	local rsync_ssh_opt=()
	if [[ -n "$SSH_PASS" ]]; then
		rsync_ssh_opt=(-e "sshpass -p '$SSH_PASS' ssh")
	fi

	rsync -rvc \
		--no-perms --no-owner --no-group \
		--delete \
		"${RSYNC_EXCLUDES[@]}" \
		${rsync_ssh_opt[@]+"${rsync_ssh_opt[@]}"} \
		"${SOURCES[@]}" \
		"$remote:~/monorepo/controls/sae_2025_ws/src/"
}

_sync_time_to() {
	local remote="$1"
	echo "[$remote] Syncing clock..."
	ssh -t "$remote" "sudo date -s '$(date)'" \
		|| echo "Warning: [$remote] clock sync failed (ensure passwordless sudo for /bin/date on Pi)" >&2
}

echo "Targets:  ${REMOTES[*]}"
echo "Packages: ${PACKAGES[*]}"

# Single target — run inline so output streams directly.
if [[ ${#REMOTES[@]} -eq 1 ]]; then
	remote="${REMOTES[0]}"
	[[ "$SYNC_TIME" == true ]] && _sync_time_to "$remote"
	_rsync_to "$remote"
	exit 0
fi

# Multiple targets — run in parallel, prefix each line with the remote name.
PIDS=()
TMPDIR_LOG=$(mktemp -d)

for remote in "${REMOTES[@]}"; do
	(
		[[ "$SYNC_TIME" == true ]] && _sync_time_to "$remote"
		_rsync_to "$remote"
	) > "$TMPDIR_LOG/$remote.log" 2>&1 &
	PIDS+=($!)
	echo "[$remote] started (pid $!)"
done

# Wait for all and report results.
FAILED=()
for i in "${!REMOTES[@]}"; do
	remote="${REMOTES[$i]}"
	pid="${PIDS[$i]}"
	if wait "$pid"; then
		echo "[$remote] done ✓"
	else
		echo "[$remote] FAILED ✗"
		FAILED+=("$remote")
	fi
	echo "--- [$remote] output ---"
	cat "$TMPDIR_LOG/$remote.log"
done

rm -rf "$TMPDIR_LOG"

if [[ ${#FAILED[@]} -gt 0 ]]; then
	echo "Failed targets: ${FAILED[*]}" >&2
	exit 1
fi
