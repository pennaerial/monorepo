#!/usr/bin/env bash
# push-deploy.sh - Deploy a hardware release into one or more remote deploy roots
#
# Usage:
#   ./push-deploy.sh pi@192.168.1.50              # Deploy latest to one Pi
#   ./push-deploy.sh pi@drone1 pi@drone2          # Deploy to multiple Pis
#   ./push-deploy.sh --build abc123f pi@drone     # Deploy specific build
#   ./push-deploy.sh --local ./build.tar.gz pi@drone
#   ./push-deploy.sh --list                       # List available builds
#   ./push-deploy.sh --status pi@drone            # Show remote release state
#
# Environment:
#   GITHUB_REPO    Repository (default: auto-detect from git remote)
#   DEPLOY_ROOT    Remote deploy root (default: ~/pennair-deploy)
#   SSH_KEY        Path to SSH key (optional)
#   SSH_PASS       SSH password (optional, requires sshpass)
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "$SCRIPT_DIR/hardware/deploy-lib.sh"

REMOTE_DEPLOY_ROOT="${DEPLOY_ROOT:-${REMOTE_DEPLOY_ROOT:-~/pennair-deploy}}"
BUILD_SHA=""
LOCAL_FILE=""
LOCAL_RELEASE_NAME=""
SSH_PASS="${SSH_PASS:-}"
TARGETS=()
INSTALL_RUNTIME_HELPER="${INSTALL_RUNTIME_HELPER:-1}"
STATUS_MODE=0

info() { deploy_info "$*"; }
warn() { deploy_warn "$*"; }
error() { deploy_error "$*"; }
step() { printf '[STEP] %s\n' "$*"; }

ssh_opts() {
    local opts="-o StrictHostKeyChecking=accept-new -o ConnectTimeout=10"
    if [[ -n "${SSH_KEY:-}" ]]; then
        opts="$opts -i $SSH_KEY"
    fi
    printf '%s\n' "$opts"
}

run_ssh() {
    if [[ -n "$SSH_PASS" ]]; then
        sshpass -p "$SSH_PASS" ssh $(ssh_opts) "$@"
    else
        ssh $(ssh_opts) "$@"
    fi
}

run_scp() {
    if [[ -n "$SSH_PASS" ]]; then
        sshpass -p "$SSH_PASS" scp $(ssh_opts) "$@"
    else
        scp $(ssh_opts) "$@"
    fi
}

print_release_table() {
    local releases="$1"
    if command -v column >/dev/null 2>&1; then
        printf '%s\n' "$releases" | column -t -s $'\t'
    else
        printf 'TAG\tPUBLISHED\tNAME\n'
        printf '%s\n' "$releases"
    fi
}

list_builds() {
    local repo releases
    repo="$(deploy_detect_repo)"
    info "Fetching builds from $repo"
    releases="$(deploy_fetch_release_list "$repo" | deploy_release_list_from_json)"
    if [[ -z "$releases" ]]; then
        warn "No build releases found"
        return 0
    fi
    print_release_table "$releases"
}

status_target() {
    local target="$1"
    [[ -n "$target" ]] || error "Status mode requires at least one target"
    step "Status for $target"
    run_ssh "$target" "DEPLOY_ROOT=$(printf '%q' "$REMOTE_DEPLOY_ROOT") bash -s" <<'REMOTE_STATUS'
set -euo pipefail
deploy_root="${DEPLOY_ROOT%/}"
deploy_root="${deploy_root/#\~/$HOME}"
current="${deploy_root}/current"
previous="${deploy_root}/previous"
config="${deploy_root}/config/runtime_fleet.yaml"
runtime="${deploy_root}/bin/runtime_fleet"
systemd="/etc/systemd/system/pennair-autonomy.service"

current_target=""
previous_target=""
if [ -L "$current" ]; then
    current_target="$(readlink -f "$current" 2>/dev/null || true)"
elif [ -e "$current" ]; then
    current_target="$current"
fi
if [ -L "$previous" ]; then
    previous_target="$(readlink -f "$previous" 2>/dev/null || true)"
elif [ -e "$previous" ]; then
    previous_target="$previous"
fi

printf 'Deploy root: %s\n' "$deploy_root"
printf 'Current release: %s\n' "${current_target:-<none>}"
printf 'Previous release: %s\n' "${previous_target:-<none>}"
printf 'Runtime config: %s\n' "$config"
printf 'Runtime helper: %s\n' "$runtime"
printf 'Systemd unit: %s\n' "$systemd"
REMOTE_STATUS
}

prepare_remote_root() {
    local target="$1"
    local remote_root
    remote_root="$(printf '%q' "$REMOTE_DEPLOY_ROOT")"
    run_ssh "$target" "mkdir -p ${remote_root}/releases ${remote_root}/config ${remote_root}/bin ${remote_root}/systemd"
}

infer_release_name_from_tarball() {
    local file="$1"
    local base
    base="$(basename "$file")"
    base="${base%.tar.gz}"
    base="${base#ros2-}"
    base="${base#release-}"
    if [[ "$base" == build-* ]]; then
        printf '%s\n' "$base"
    elif [[ "$base" == *-* ]]; then
        printf '%s\n' "$base"
    else
        printf 'build-%s\n' "$base"
    fi
}

get_build_url() {
    local repo sha release_name
    repo="$(deploy_detect_repo)"
    release_name="$(deploy_release_name_for_sha "${1:-}")"
    deploy_fetch_release_by_tag "$repo" "$release_name" | deploy_release_download_url_from_json
}

download_build() {
    local url="$1"
    local dest="$2"
    [[ -n "$url" && "$url" != "null" ]] || error "No build found. Run with --list to see available builds."
    info "Downloading: $url"
    curl -fL "$url" -o "$dest"
}

copy_runtime_helper() {
    local target="$1"
    local helper="$SCRIPT_DIR/hardware/runtime_fleet.sh"
    if [[ "$INSTALL_RUNTIME_HELPER" != "1" || ! -f "$helper" ]]; then
        return 0
    fi

    prepare_remote_root "$target"
    run_scp "$helper" "$target:$REMOTE_DEPLOY_ROOT/bin/runtime_fleet"
}

deploy_to_target() {
    local target="$1"
    local tarball="$2"
    local release_name="$3"
    local archive_remote remote_cmd

    archive_remote="/tmp/${release_name}.tar.gz"

    step "Deploying to $target"
    info "Checking connection"
    if ! run_ssh "$target" "echo connected" >/dev/null 2>&1; then
        error "Cannot connect to $target"
    fi

    prepare_remote_root "$target"
    copy_runtime_helper "$target"

    info "Copying build artifact"
    run_scp "$tarball" "$target:$archive_remote"

    remote_cmd=$(printf 'DEPLOY_ROOT=%q RELEASE_NAME=%q ARCHIVE=%q bash -s' "$REMOTE_DEPLOY_ROOT" "$release_name" "$archive_remote")
    info "Activating release ${release_name}"
    run_ssh "$target" "$remote_cmd" <<'REMOTE_INSTALL'
set -euo pipefail

deploy_root="${DEPLOY_ROOT/#\~/$HOME}"
deploy_root="${deploy_root%/}"
release_name="${RELEASE_NAME}"
archive="${ARCHIVE}"

releases_dir="${deploy_root}/releases"
release_dir="${releases_dir}/${release_name}"
current_link="${deploy_root}/current"
previous_link="${deploy_root}/previous"
stage_dir="$(mktemp -d "${releases_dir}/.staging.${release_name}.XXXXXX")"

mkdir -p "$releases_dir" "${deploy_root}/config" "${deploy_root}/bin" "${deploy_root}/systemd"
tar -xzf "$archive" -C "$stage_dir"
rm -f "$archive"

rm -rf "$release_dir"
mv "$stage_dir" "$release_dir"

current_target=""
if [ -L "$current_link" ]; then
    current_target="$(readlink -f "$current_link" 2>/dev/null || true)"
elif [ -e "$current_link" ]; then
    rm -rf "$previous_link"
    mv "$current_link" "$previous_link"
fi

if [ -n "$current_target" ] && [ "$current_target" != "$release_dir" ]; then
    ln -sfn "$current_target" "$previous_link"
fi

ln -sfn "$release_dir" "$current_link"
chmod +x "${deploy_root}/bin/runtime_fleet" 2>/dev/null || true

if command -v sudo >/dev/null 2>&1 && sudo -n systemctl list-unit-files pennair-autonomy.service >/dev/null 2>&1; then
    if sudo -n systemctl restart pennair-autonomy.service >/dev/null 2>&1; then
        printf 'Activated pennair-autonomy.service\n'
    else
        printf 'Release installed, but automatic restart of pennair-autonomy.service failed\n' >&2
    fi
else
    printf 'Release installed, but pennair-autonomy.service is not installed or sudo is unavailable\n' >&2
fi

printf 'Deployed %s\n' "$release_name"
REMOTE_INSTALL

    info "Deployed to $target"
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --list|-l)
                list_builds
                exit 0
                ;;
            --status|-s)
                STATUS_MODE=1
                shift
                while [[ $# -gt 0 && "$1" != --* ]]; do
                    TARGETS+=("$1")
                    shift
                done
                ;;
            --build|-b)
                BUILD_SHA="${2:-}"
                shift 2
                ;;
            --local|-f)
                LOCAL_FILE="${2:-}"
                shift 2
                ;;
            --deploy-root)
                REMOTE_DEPLOY_ROOT="${2:-}"
                shift 2
                ;;
            --no-runtime-helper)
                INSTALL_RUNTIME_HELPER=0
                shift
                ;;
            --password|-p)
                SSH_PASS="${2:-}"
                shift 2
                ;;
            --help|-h)
                cat <<EOF
Usage: $0 [OPTIONS] TARGET [TARGET...]

Deploy hardware release artifacts to one or more Pis via SSH.

Options:
  --list, -l              List available builds
  --status, -s [TARGET]    Show deploy-root release state on a target
  --build, -b SHA         Deploy a specific build by short SHA
  --local, -f FILE        Deploy a local .tar.gz file
  --deploy-root PATH      Remote deploy root (default: ~/pennair-deploy)
  --password, -p PASS     SSH password (requires sshpass)
  --no-runtime-helper     Skip copying the runtime_fleet helper
  --help, -h              Show this help

Examples:
  $0 pi@192.168.1.50                    # Latest build to one Pi
  $0 pi@drone1 pi@drone2                # Latest to multiple Pis
  $0 --build abc123f pi@drone           # Specific build
  $0 --local ./my-build.tar.gz pi@drone # Local file
  $0 --status pi@drone                  # Remote status

Bootstrap the Pi once with:
  scripts/hardware/bootstrap-pi.sh

Environment:
  GITHUB_REPO   Repository (auto-detected from git)
  DEPLOY_ROOT   Remote deploy root (default: ~/pennair-deploy)
  SSH_KEY       Path to SSH key file
EOF
                exit 0
                ;;
            -*)
                error "Unknown option: $1"
                ;;
            *)
                TARGETS+=("$1")
                shift
                ;;
        esac
    done

    if [[ "$STATUS_MODE" == "1" && ${#TARGETS[@]} -eq 0 ]]; then
        error "Status mode requires at least one target"
    fi
}

main() {
    parse_args "$@"

    if [[ "$STATUS_MODE" == "1" ]]; then
        local target
        for target in "${TARGETS[@]}"; do
            status_target "$target"
        done
        return 0
    fi

    local tarball cleanup=false release_name
    if [[ -n "$LOCAL_FILE" ]]; then
        [[ -f "$LOCAL_FILE" ]] || error "Local file not found: $LOCAL_FILE"
        tarball="$LOCAL_FILE"
        release_name="${LOCAL_RELEASE_NAME:-$(infer_release_name_from_tarball "$tarball")}"
        info "Using local file: $tarball"
    else
        local url
        if [[ -n "$BUILD_SHA" ]]; then
            url="$(get_build_url "$BUILD_SHA")"
            release_name="$(deploy_release_name_for_sha "$BUILD_SHA")"
        else
            local repo release_json
            repo="$(deploy_detect_repo)"
            release_json="$(deploy_fetch_release_list "$repo" | jq -r '[.[] | select(.tag_name | startswith("build-"))][0]')"
            release_name="$(printf '%s' "$release_json" | deploy_release_name_from_json)"
            url="$(printf '%s' "$release_json" | deploy_release_download_url_from_json)"
        fi
        if [[ -z "$release_name" || "$release_name" == "null" ]]; then
            error "Unable to resolve a build release name"
        fi
        if [[ -z "$url" || "$url" == "null" ]]; then
            error "Unable to resolve a build download URL"
        fi
        tarball="/tmp/${release_name}.tar.gz"
        cleanup=true
        download_build "$url" "$tarball"
    fi

    if [[ ${#TARGETS[@]} -eq 0 ]]; then
        error "No target specified. Usage: $0 [OPTIONS] pi@hostname"
    fi

    local failed=()
    local target
    for target in "${TARGETS[@]}"; do
        if ! deploy_to_target "$target" "$tarball" "$release_name"; then
            failed+=("$target")
        fi
    done

    if [[ "$cleanup" == true ]]; then
        rm -f "$tarball"
    fi

    printf '\n'
    info "===== Deployment Summary ====="
    info "Successful: $((${#TARGETS[@]} - ${#failed[@]}))/${#TARGETS[@]}"
    if [[ ${#failed[@]} -gt 0 ]]; then
        warn "Failed: ${failed[*]}"
        exit 1
    fi

    printf '\n'
    info "To start the hardware runtime on a Pi:"
    printf '  %s/bin/runtime_fleet\n' "$REMOTE_DEPLOY_ROOT"
    printf '  sudo systemctl restart pennair-autonomy.service\n'
    printf '  sudo systemctl status pennair-autonomy.service\n'
}

main "$@"
