#!/usr/bin/env bash
# deploy.sh - Deploy a hardware release into a local deploy root
#
# Usage:
#   ./deploy.sh                 # Deploy latest build
#   ./deploy.sh abc123f         # Deploy specific build by short SHA
#   ./deploy.sh --list          # List available builds
#   ./deploy.sh --status        # Show current/previous release state
#
# Environment:
#   GITHUB_REPO    Repository (default: auto-detect from git remote)
#   DEPLOY_ROOT    Deploy root (default: ~/pennair-deploy)
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "$SCRIPT_DIR/hardware/deploy-lib.sh"

check_deps() {
    deploy_require_cmds curl jq tar python3
}

run_root() {
    if [[ "$(id -u)" -eq 0 ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

print_help() {
    cat <<EOF
Usage: $0 [OPTIONS] [SHORT_SHA]

Deploy a build artifact into the local deploy root.

Options:
  --list, -l     List available builds
  --status, -s   Show deploy-root release state
  --help, -h     Show this help

Examples:
  $0             Deploy latest build
  $0 abc123f     Deploy specific build
  $0 --list      List available builds
  $0 --status    Show current and previous release links

Bootstrap on the Pi with:
  scripts/hardware/bootstrap-pi.sh

Environment:
  GITHUB_REPO    Repository (default: auto-detected from git)
  DEPLOY_ROOT    Deploy root (default: ~/pennair-deploy)
EOF
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
    deploy_info "Fetching builds from $repo"
    releases="$(deploy_fetch_release_list "$repo" | deploy_release_list_from_json)"
    if [[ -z "$releases" ]]; then
        deploy_warn "No build releases found"
        return 0
    fi
    print_release_table "$releases"
}

show_status() {
    deploy_init_paths
    deploy_prepare_root
    deploy_status_lines
}

get_latest_release_json() {
    local repo
    repo="$(deploy_detect_repo)"
    deploy_fetch_release_list "$repo" | jq -r '[.[] | select(.tag_name | startswith("build-"))][0]'
}

get_release_json_by_sha() {
    local repo release_name
    repo="$(deploy_detect_repo)"
    release_name="$(deploy_release_name_for_sha "${1:-}")"
    deploy_fetch_release_by_tag "$repo" "$release_name"
}

install_release_from_json() {
    local release_json="$1"
    local release_tag asset_name download_url release_dir stage_dir tarball_path

    release_tag="$(printf '%s' "$release_json" | deploy_release_name_from_json)"
    asset_name="$(printf '%s' "$release_json" | deploy_release_asset_name_from_json)"
    download_url="$(printf '%s' "$release_json" | deploy_release_download_url_from_json)"

    if [[ -z "$release_tag" || "$release_tag" == "null" || -z "$asset_name" || "$asset_name" == "null" || -z "$download_url" || "$download_url" == "null" ]]; then
        deploy_error "Release metadata is missing an artifact download URL"
    fi

    deploy_init_paths
    deploy_prepare_root

    release_dir="$(deploy_release_dir_for_name "$release_tag")"
    stage_dir="$(mktemp -d "${RELEASES_DIR}/.staging.${release_tag}.XXXXXX")"
    tarball_path="$stage_dir/$asset_name"

    deploy_info "Deploying $release_tag into $release_dir"
    deploy_info "Downloading $asset_name"
    curl -fL "$download_url" -o "$tarball_path"

    deploy_info "Extracting artifact"
    tar -xzf "$tarball_path" -C "$stage_dir"
    rm -f "$tarball_path"

    rm -rf "$release_dir"
    mv "$stage_dir" "$release_dir"
    deploy_update_release_links "$release_dir"

    if [[ -f "$SCRIPT_DIR/hardware/runtime_fleet.sh" ]]; then
        deploy_install_runtime_helper "$SCRIPT_DIR/hardware/runtime_fleet.sh"
    fi

    deploy_ensure_uav_python_runtime run_root

    if command -v systemctl >/dev/null 2>&1 && systemctl list-unit-files pennair-autonomy.service >/dev/null 2>&1; then
        if command -v sudo >/dev/null 2>&1; then
            if sudo -n systemctl restart pennair-autonomy.service >/dev/null 2>&1; then
                deploy_info "Activated pennair-autonomy.service"
            else
                deploy_warn "Release installed, but automatic restart of pennair-autonomy.service failed"
            fi
        else
            deploy_warn "Release installed, but sudo is unavailable for restarting pennair-autonomy.service"
        fi
    else
        deploy_warn "Release installed, but pennair-autonomy.service is not installed yet"
    fi

    deploy_info "Deployment complete"
    deploy_status_lines
    printf '\n'
    deploy_info "Next steps:"
    printf '  %s/bin/runtime_fleet\n' "$DEPLOY_ROOT"
    printf '  sudo systemctl restart pennair-autonomy.service\n'
}

main() {
    case "${1:-}" in
        --help|-h)
            print_help
            ;;
        --status|-s)
            show_status
            ;;
        --list|-l)
            check_deps
            list_builds
            ;;
        "")
            check_deps
            deploy_info "Fetching latest build"
            install_release_from_json "$(get_latest_release_json)"
            ;;
        *)
            check_deps
            deploy_info "Fetching build ${1}"
            install_release_from_json "$(get_release_json_by_sha "$1")"
            ;;
    esac
}

main "$@"
