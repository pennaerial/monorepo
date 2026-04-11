#!/usr/bin/env bash

deploy_root_default() {
    printf '%s\n' "${DEPLOY_ROOT_DEFAULT:-$HOME/pennair-deploy}"
}

deploy_init_paths() {
    DEPLOY_ROOT="${DEPLOY_ROOT:-$(deploy_root_default)}"
    RELEASES_DIR="$DEPLOY_ROOT/releases"
    CURRENT_LINK="$DEPLOY_ROOT/current"
    PREVIOUS_LINK="$DEPLOY_ROOT/previous"
    CONFIG_DIR="$DEPLOY_ROOT/config"
    BIN_DIR="$DEPLOY_ROOT/bin"
    SYSTEMD_DIR="$DEPLOY_ROOT/systemd"
}

deploy_info() {
    printf '[INFO] %s\n' "$*"
}

deploy_warn() {
    printf '[WARN] %s\n' "$*" >&2
}

deploy_error() {
    printf '[ERROR] %s\n' "$*" >&2
    exit 1
}

deploy_require_cmds() {
    local missing=()
    local cmd
    for cmd in "$@"; do
        if ! command -v "$cmd" >/dev/null 2>&1; then
            missing+=("$cmd")
        fi
    done

    if ((${#missing[@]} > 0)); then
        deploy_error "Missing required commands: ${missing[*]}"
    fi
}

deploy_detect_repo() {
    if [[ -n "${GITHUB_REPO:-}" ]]; then
        printf '%s\n' "$GITHUB_REPO"
        return 0
    fi

    local remote_url
    remote_url="$(git remote get-url origin 2>/dev/null || true)"
    if [[ "$remote_url" =~ github\.com[:/]([^/]+/[^/.]+) ]]; then
        printf '%s\n' "${BASH_REMATCH[1]}"
        return 0
    fi

    deploy_error "Could not detect GITHUB_REPO. Set GITHUB_REPO=org/repo."
}

deploy_github_api_base() {
    local repo="$1"
    printf 'https://api.github.com/repos/%s' "$repo"
}

deploy_fetch_release_list() {
    local repo="$1"
    curl -fsSL "$(deploy_github_api_base "$repo")/releases?per_page=100"
}

deploy_fetch_release_by_tag() {
    local repo="$1"
    local tag="$2"
    curl -fsSL "$(deploy_github_api_base "$repo")/releases/tags/$tag"
}

deploy_release_name_for_sha() {
    local sha="${1:-}"
    if [[ -z "$sha" ]]; then
        deploy_error "A release SHA is required."
    fi
    if [[ "$sha" == build-* ]]; then
        printf '%s\n' "$sha"
    else
        printf 'build-%s\n' "$sha"
    fi
}

deploy_release_name_from_json() {
    jq -r '.tag_name // empty'
}

deploy_release_asset_name_from_json() {
    jq -r '.assets[0].name // empty'
}

deploy_release_download_url_from_json() {
    jq -r '.assets[0].browser_download_url // empty'
}

deploy_release_list_from_json() {
    jq -r '
        .[]
        | select(.tag_name | startswith("build-"))
        | [.tag_name, (.published_at // ""), (.name // .tag_name)]
        | @tsv
    '
}

deploy_release_summary_from_json() {
    jq -r '
        [.tag_name, (.published_at // ""), (.name // .tag_name)]
        | @tsv
    '
}

deploy_prepare_root() {
    mkdir -p "$RELEASES_DIR" "$CONFIG_DIR" "$BIN_DIR" "$SYSTEMD_DIR"
}

deploy_current_target() {
    if [[ -L "$CURRENT_LINK" ]]; then
        readlink -f "$CURRENT_LINK" 2>/dev/null || true
        return 0
    fi

    if [[ -e "$CURRENT_LINK" ]]; then
        printf '%s\n' "$CURRENT_LINK"
    fi
}

deploy_previous_target() {
    if [[ -L "$PREVIOUS_LINK" ]]; then
        readlink -f "$PREVIOUS_LINK" 2>/dev/null || true
        return 0
    fi

    if [[ -e "$PREVIOUS_LINK" ]]; then
        printf '%s\n' "$PREVIOUS_LINK"
    fi
}

deploy_update_release_links() {
    local release_dir="$1"
    local current_target=""

    if [[ -L "$CURRENT_LINK" ]]; then
        current_target="$(readlink -f "$CURRENT_LINK" 2>/dev/null || true)"
    elif [[ -e "$CURRENT_LINK" ]]; then
        rm -rf "$PREVIOUS_LINK"
        mv "$CURRENT_LINK" "$PREVIOUS_LINK"
    fi

    if [[ -n "$current_target" && "$current_target" != "$release_dir" ]]; then
        ln -sfn "$current_target" "$PREVIOUS_LINK"
    fi

    ln -sfn "$release_dir" "$CURRENT_LINK"
}

deploy_release_dir_for_name() {
    local release_name="$1"
    printf '%s/%s' "$RELEASES_DIR" "$release_name"
}

deploy_install_runtime_helper() {
    local source_file="${1:-}"
    if [[ -z "$source_file" ]]; then
        deploy_error "runtime helper source file is required"
    fi
    if [[ ! -f "$source_file" ]]; then
        deploy_error "runtime helper source file not found: $source_file"
    fi

    install -d "$BIN_DIR"
    install -m 0755 "$source_file" "$BIN_DIR/runtime_fleet"
}

deploy_status_lines() {
    local current_target previous_target
    current_target="$(deploy_current_target)"
    previous_target="$(deploy_previous_target)"

    printf 'Deploy root: %s\n' "$DEPLOY_ROOT"
    printf 'Current release: %s\n' "${current_target:-<none>}"
    printf 'Previous release: %s\n' "${previous_target:-<none>}"
    printf 'Runtime config: %s\n' "$CONFIG_DIR/runtime_fleet.yaml"
    printf 'Runtime helper: %s\n' "$BIN_DIR/runtime_fleet"
    printf 'Systemd unit: %s\n' "/etc/systemd/system/pennair-autonomy.service"
}
