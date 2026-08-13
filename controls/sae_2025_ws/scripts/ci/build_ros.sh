#!/usr/bin/env bash

# script for building the SAE 2025 ROS Workspace
# assumes all required dependencies have been installed already and ROS sourced

set -euo pipefail

# 'clean': triggers a clean rebuild of all ros packages in src/, including 3rd party ros submodules (px4_msgs)
# 'quick': tries to use build cache of all 3rd party submodules if git commit refs match up.
#          if commit refs of a 3rd party submodule doesn't match up it triggers a clean build. Non 3rd party submodules (if built)
#          behave the same as in 'clean' mode.
BUILD_MODE="quick"
BUILD_THIRD=false # if flag set, only builds THIRD_ROS_PACKAGES

# Parse arguments. use via: ./build_ros_new --build-mode [BUILD_MODE]
while [[ $# -gt 0 ]]; do
	case "$1" in
	--build-mode)
		case "$2" in
		clean)
			BUILD_MODE="clean"
			;;
		quick)
			BUILD_MODE="quick"
			;;
		*)
			BUILD_MODE="quick"
			;;
		esac
		shift 2
		;;
	--third)
		BUILD_THIRD=true
		shift
		;;
	*)
		shift
		;;
	esac
done

echo "[build_ros.sh] Starting..."
echo "[build_ros.sh] BUILD_MODE: $BUILD_MODE"

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ros_ci_conf="${SCRIPT_DIR}/ros_ci.conf"
source ${ros_ci_conf} # imports configuration constants
echo "[build_ros.sh] using conf file: ${ros_ci_conf}"

SRC_DIR="${PENNAIR_SAE_WS_PATH}/src"
BUILD_DIR="${PENNAIR_SAE_WS_PATH}/build"
INSTALL_DIR="${PENNAIR_SAE_WS_PATH}/install"

# --- Get the full list of package names in the workspace ---
mapfile -t all_packages < <(colcon list -n --base-paths "${PENNAIR_SAE_WS_PATH}")

# --- Precompute the list of non-third-party packages ---
non_third_party_packages=()
for pkg in "${all_packages[@]}"; do
	is_third_party="false"
	for third_party_pkg in "${THIRD_ROS_PACKAGES[@]}"; do
		if [[ "${pkg}" == "${third_party_pkg}" ]]; then
			is_third_party="true"
			break
		fi
	done
	if [[ "${is_third_party}" == "false" ]]; then
		non_third_party_packages+=("${pkg}")
	fi
done

echo "[build_ros.sh] 3rd party packages: ${THIRD_ROS_PACKAGES[*]}"
echo "[build_ros.sh] Non 3rd party packages: ${non_third_party_packages[*]}"

if [[ "$BUILD_MODE" == "clean" ]]; then
	rm -rf "${BUILD_DIR}" "${INSTALL_DIR}"
elif [[ "$BUILD_MODE" == "quick" ]]; then
	# --- Handle third-party packages: only wipe build/install if commit SHA changed ---
	for pkg in "${THIRD_ROS_PACKAGES[@]}"; do
		pkg_src_dir="${SRC_DIR}/${pkg}"
		commit_tag_file="${BUILD_DIR}/${pkg}/COMMIT_TAG"
		stale="false"
		if [[ ! -d "${pkg_src_dir}" ]]; then # 3rd-party pkg doesn't exist
			echo "[build_ros.sh] WARNING: src dir for '${pkg}' not found at ${pkg_src_dir}, treating as stale"
			stale="true"
		elif [[ ! -f "${commit_tag_file}" ]]; then # commit tag file doesn't exist
			echo "[build_ros.sh] No COMMIT_TAG found for '${pkg}', treating as stale"
			stale="true"
		else
			cached_sha=$(<"${commit_tag_file}")
			current_sha=$(git -C "${pkg_src_dir}" rev-parse HEAD 2>/dev/null || echo "")
			if [[ -z "${current_sha}" ]]; then # no sha detected
				echo "[build_ros.sh] Could not resolve current SHA for '${pkg}', treating as stale"
				stale="true"
			elif [[ "${cached_sha}" != "${current_sha}" ]]; then # sha mismatch
				echo "[build_ros.sh] SHA mismatch for '${pkg}' (cached: ${cached_sha}, current: ${current_sha}), treating as stale"
				stale="true"
			fi
		fi
		if [[ "${stale}" == "true" ]]; then
			echo "[build_ros.sh] Removing build/install for '${pkg}'"
			rm -rf "${BUILD_DIR}/${pkg}" "${INSTALL_DIR}/${pkg}"
		fi
	done

	# --- Non-third-party packages always get a clean build/install ---
	for pkg in "${non_third_party_packages[@]}"; do
		rm -rf "${BUILD_DIR}/${pkg}" "${INSTALL_DIR}/${pkg}"
	done
fi

cd "${PENNAIR_SAE_WS_PATH}"
set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

if [[ "${BUILD_THIRD}" == "true" ]]; then
	echo "[build_ros.sh] Building only third-party packages: ${THIRD_ROS_PACKAGES[*]}"
    colcon \
        --log-base "${PENNAIR_SAE_WS_PATH}/log" \
        build \
        --base-paths "${PENNAIR_SAE_WS_PATH}/src" \
        --build-base "${PENNAIR_SAE_WS_PATH}/build" \
        --install-base "${PENNAIR_SAE_WS_PATH}/install" \
        --packages-select "${THIRD_ROS_PACKAGES[@]}"
else
	echo "[build_ros.sh] Building All Packages..."
    colcon \
        --log-base "${PENNAIR_SAE_WS_PATH}/log" \
        build \
        --base-paths "${PENNAIR_SAE_WS_PATH}/src" \
        --build-base "${PENNAIR_SAE_WS_PATH}/build" \
        --install-base "${PENNAIR_SAE_WS_PATH}/install"
fi

if [[ ${#THIRD_ROS_PACKAGES[@]} -gt 0 ]]; then
	# --- Write COMMIT_TAG for each third-party package after a successful build ---
	for pkg in "${THIRD_ROS_PACKAGES[@]}"; do
		pkg_src_dir="${SRC_DIR}/${pkg}"
		pkg_build_dir="${BUILD_DIR}/${pkg}"
		if [[ -d "${pkg_build_dir}" ]]; then
			echo "[build_ros.sh] Writing COMMIT_TAG at ${pkg_build_dir}/COMMIT_TAG"
			git -C "${pkg_src_dir}" rev-parse HEAD >"${pkg_build_dir}/COMMIT_TAG"
		else
			echo "[build_ros.sh] WARNING: expected build dir for '${pkg}' not found at ${pkg_build_dir}, skipping COMMIT_TAG write"
		fi
	done
fi
