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

deploy_python_has_apriltag() {
    python3 -c "import apriltag" >/dev/null 2>&1
}

deploy_python_has_pydantic_v2() {
    python3 - <<'PY' >/dev/null 2>&1
try:
    import pydantic

    version = getattr(pydantic, "__version__", "0")
    major = int(version.split(".", 1)[0])
except Exception:
    raise SystemExit(1)

raise SystemExit(0 if major >= 2 else 1)
PY
}

deploy_clock_is_synchronized() {
    if ! command -v timedatectl >/dev/null 2>&1; then
        return 1
    fi

    local key value
    for key in NTPSynchronized SystemClockSynchronized; do
        value="$(timedatectl show -p "$key" --value 2>/dev/null || true)"
        if [[ "$value" == "yes" ]]; then
            return 0
        fi
    done

    return 1
}

deploy_fetch_http_time_epoch() {
    local url="$1"

    if command -v python3 >/dev/null 2>&1; then
        python3 - "$url" <<'PY'
import sys
import urllib.request
from email.utils import parsedate_to_datetime

url = sys.argv[1]
req = urllib.request.Request(url, method="GET")
with urllib.request.urlopen(req, timeout=5) as resp:
    date_header = resp.headers.get("Date")
    if not date_header:
        raise SystemExit(1)
    dt = parsedate_to_datetime(date_header)
    print(int(dt.timestamp()))
PY
        return $?
    fi

    return 1
}

deploy_http_time_fallback() {
    local runner="${1:-}"
    local urls="${HTTP_TIME_URLS:-http://connectivity-check.ubuntu.com/ http://ports.ubuntu.com/ubuntu-ports/ http://archive.ubuntu.com/ubuntu/}"
    local url epoch

    for url in $urls; do
        epoch="$(deploy_fetch_http_time_epoch "$url" 2>/dev/null || true)"
        if [[ -z "$epoch" ]]; then
            continue
        fi

        deploy_info "Setting system clock from HTTP Date header: $url"
        if [[ -n "$runner" ]]; then
            "$runner" timedatectl set-ntp false >/dev/null 2>&1 || true
            "$runner" date -u -s "@$epoch" >/dev/null
            "$runner" hwclock --systohc >/dev/null 2>&1 || true
            "$runner" timedatectl set-ntp true >/dev/null 2>&1 || true
            "$runner" systemctl restart systemd-timesyncd >/dev/null 2>&1 || true
        else
            timedatectl set-ntp false >/dev/null 2>&1 || true
            date -u -s "@$epoch" >/dev/null
            hwclock --systohc >/dev/null 2>&1 || true
            timedatectl set-ntp true >/dev/null 2>&1 || true
            systemctl restart systemd-timesyncd >/dev/null 2>&1 || true
        fi
        return 0
    done

    return 1
}

deploy_preflight_time_sync() {
    local runner="${1:-}"
    if ! command -v timedatectl >/dev/null 2>&1; then
        return 0
    fi

    deploy_info "Ensuring system clock is synchronized before apt operations"
    if [[ -n "$runner" ]]; then
        "$runner" timedatectl set-ntp true >/dev/null 2>&1 || true
        "$runner" systemctl restart systemd-timesyncd >/dev/null 2>&1 || true
    else
        timedatectl set-ntp true >/dev/null 2>&1 || true
        systemctl restart systemd-timesyncd >/dev/null 2>&1 || true
    fi

    local _attempt
    for _attempt in {1..10}; do
        if deploy_clock_is_synchronized; then
            deploy_info "System clock synchronized"
            return 0
        fi
        sleep 2
    done

    if deploy_http_time_fallback "$runner"; then
        deploy_warn "NTP did not synchronize the clock, but an HTTP Date fallback was applied. Continuing with the corrected system time."
        return 0
    fi

    deploy_warn "System clock is not yet reported as synchronized; apt may still fail until time is corrected."
    return 0
}

deploy_report_apt_failure() {
    local log_file="$1"
    local hints=()

    if grep -q "not valid yet" "$log_file"; then
        hints+=("Clock skew detected. On the Pi, run 'sudo timedatectl set-ntp true' and wait for time sync, or set the clock manually if the Pi has no NTP access.")
    fi

    if grep -q "NO_PUBKEY" "$log_file"; then
        hints+=("A third-party apt repository has a missing or stale signing key. On hardware Pis, sim-only repos such as Gazebo can usually be disabled if they are not needed.")
    fi

    if grep -q "packages.osrfoundation.org/gazebo" "$log_file"; then
        hints+=("Detected the Gazebo apt repository. If this Pi does not run local simulation, disable '/etc/apt/sources.list.d/gazebo-stable.list' and rerun provisioning.")
    fi

    if ((${#hints[@]} == 0)); then
        deploy_error "apt-get update failed. Review the apt output above and repair the Pi's package sources before rerunning this script."
    fi

    printf '[ERROR] apt-get update failed.\n' >&2
    local hint
    for hint in "${hints[@]}"; do
        printf '[ERROR] %s\n' "$hint" >&2
    done
    exit 1
}

deploy_apt_update() {
    local runner="${1:-}"
    local log_file
    log_file="$(mktemp)"

    if [[ -n "$runner" ]]; then
        if "$runner" apt-get update 2>&1 | tee "$log_file"; then
            rm -f "$log_file"
            return 0
        fi
    else
        if apt-get update 2>&1 | tee "$log_file"; then
            rm -f "$log_file"
            return 0
        fi
    fi

    deploy_report_apt_failure "$log_file"
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
