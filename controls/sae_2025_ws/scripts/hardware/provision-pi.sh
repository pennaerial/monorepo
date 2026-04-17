#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "$SCRIPT_DIR/deploy-lib.sh"

HOSTNAME_VALUE="$(hostname)"
DEPLOY_USER="${DEPLOY_USER:-penn}"
AUTHORIZED_KEY_FILE=""
AUTHORIZED_KEY_TEXT=""
AUTHORIZED_KEY_URL="${AUTHORIZED_KEY_URL:-}"
ALLOW_PLACEHOLDER_HOSTNAME=0
ENABLE_NOPASSWD_SUDO="${ENABLE_NOPASSWD_SUDO:-1}"
RUN_BOOTSTRAP="${RUN_BOOTSTRAP:-1}"
START_SERVICE="${START_SERVICE:-0}"
INSTALL_PIGPIO="${INSTALL_PIGPIO:-1}"
SERVICE_NAME="${SERVICE_NAME:-pennair-autonomy}"
DEPLOY_ROOT="${DEPLOY_ROOT:-}"
TRAVEL_ROUTER_SSID="${TRAVEL_ROUTER_SSID:-pennair_5G}"
TRAVEL_ROUTER_PSK="${TRAVEL_ROUTER_PSK:-pennair123!}"
DEFAULT_NETWORK_ROLE="${DEFAULT_NETWORK_ROLE:-client}"
ALLOWED_AP_HOSTS="${ALLOWED_AP_HOSTS:-}"
AP_SELECTION="${AP_SELECTION:-strongest}"
FALLBACK_AP_PREFIX="${FALLBACK_AP_PREFIX:-pennair-ap}"
FALLBACK_PSK="${FALLBACK_PSK:-pennair123!}"
LOCAL_AP_SSID="${LOCAL_AP_SSID:-}"
INSTALL_WIFI_FAILOVER="${INSTALL_WIFI_FAILOVER:-1}"

usage() {
    cat <<EOF
Usage: $0 [OPTIONS]

Provision a PennAiR hardware Pi for dashboard-managed deploys.

This script is intended to run on the Pi itself. It configures:
  - hostname / mDNS identity
  - openssh-server and avahi-daemon
  - deploy user creation and sudo membership
  - authorized_keys installation
  - optional passwordless sudo for the deploy user
  - the existing hardware bootstrap path

Options:
  --hostname NAME            Hostname to configure (default: current hostname)
  --user NAME                Deploy user to create/use (default: $DEPLOY_USER)
  --authorized-key-file PATH Read one SSH public key from PATH
  --authorized-key TEXT      Install one SSH public key from literal text
  --authorized-key-url URL   Download one SSH public key from URL
  --allow-placeholder-hostname
                             Allow placeholder-style hostnames such as air-0x
  --deploy-root PATH         Deploy root for bootstrap (default: /home/<user>/pennair-deploy)
  --service NAME             Systemd service name for bootstrap (default: $SERVICE_NAME)
  --travel-router-ssid SSID  Preferred travel-router Wi-Fi SSID
  --travel-router-psk PASS   Password for the travel-router Wi-Fi profile
  --network-role ROLE        Default fallback role at bootstrap: ap or client
  --allowed-ap-hosts CSV     Optional fallback AP hostnames for client mode
                             Empty means discover any visible PennAiR fallback AP
                             matching the configured prefix
  --ap-selection MODE        strongest or ordered_then_strongest
  --fallback-psk PASS        Shared PSK for Pi-hosted fallback APs
  --fallback-ap-prefix PREF  Prefix for Pi-hosted fallback AP SSIDs
  --local-ap-ssid SSID       Override the local Pi-hosted fallback AP SSID
  --no-wifi-failover         Skip NetworkManager failover setup during bootstrap
  --no-nopasswd-sudo         Do not install a passwordless sudoers drop-in
  --no-bootstrap             Skip hardware bootstrap after provisioning
  --start                    Start the runtime service during bootstrap
  --no-pigpio                Skip pigpio installation during bootstrap
  -h, --help                 Show this help

Examples:
  $0 --hostname air-01 --authorized-key-file ~/.ssh/pennair_pi_ed25519.pub
  $0 --hostname air-02 --authorized-key-url https://raw.githubusercontent.com/pennaerial/monorepo/main/controls/sae_2025_ws/ops/keys/pennair_pi_ed25519.pub
  $0 --hostname payload-01 --user penn --no-bootstrap
  $0 --hostname air-0x --allow-placeholder-hostname --no-bootstrap
EOF
}

run_root() {
    if [[ "$(id -u)" -eq 0 ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

ensure_root_access() {
    if [[ "$(id -u)" -eq 0 ]]; then
        return 0
    fi
    if ! command -v sudo >/dev/null 2>&1; then
        deploy_error "This script requires root privileges or sudo."
    fi
    if ! sudo -n true >/dev/null 2>&1; then
        deploy_info "sudo access is required for provisioning."
        sudo true
    fi
}

normalized_hostname() {
    local raw="${1:-}"
    raw="${raw%.local}"
    raw="${raw%.}"
    printf '%s\n' "${raw,,}"
}

validate_hostname() {
    local value
    value="$(normalized_hostname "$1")"
    if [[ ! "$value" =~ ^[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?$ ]]; then
        deploy_error "Invalid hostname '$1'. Use lowercase letters, numbers, and dashes only."
    fi
    printf '%s\n' "$value"
}

warn_placeholder_hostname() {
    local hostname="$1"

    if [[ ! "$hostname" =~ (^|-)0x($|-) ]]; then
        return 0
    fi

    if [[ "$ALLOW_PLACEHOLDER_HOSTNAME" == "1" ]]; then
        deploy_warn "Continuing with placeholder-style hostname '$hostname' because --allow-placeholder-hostname was provided."
        return 0
    fi

    deploy_warn "Hostname '$hostname' looks like an example placeholder from the README."
    deploy_warn "Use a real hostname such as 'air-01' or rerun with --allow-placeholder-hostname to bypass this check."
    deploy_error "Refusing to provision with placeholder-style hostname '$hostname'."
}

resolve_user_home() {
    local user_name="$1"
    local home_dir
    home_dir="$(getent passwd "$user_name" | awk -F: 'NR==1 {print $6}')"
    if [[ -n "$home_dir" ]]; then
        printf '%s\n' "$home_dir"
        return 0
    fi
    printf '/home/%s\n' "$user_name"
}

read_authorized_key() {
    local source_count=0
    [[ -n "$AUTHORIZED_KEY_FILE" ]] && ((source_count += 1))
    [[ -n "$AUTHORIZED_KEY_TEXT" ]] && ((source_count += 1))
    [[ -n "$AUTHORIZED_KEY_URL" ]] && ((source_count += 1))

    if [[ "$source_count" -gt 1 ]]; then
        deploy_error "Use exactly one of --authorized-key-file, --authorized-key, or --authorized-key-url."
    fi

    if [[ -n "$AUTHORIZED_KEY_FILE" ]]; then
        if [[ ! -f "$AUTHORIZED_KEY_FILE" ]]; then
            deploy_error "Authorized key file not found: $AUTHORIZED_KEY_FILE"
        fi
        cat "$AUTHORIZED_KEY_FILE"
        return 0
    fi

    if [[ -n "$AUTHORIZED_KEY_TEXT" ]]; then
        printf '%s\n' "$AUTHORIZED_KEY_TEXT"
        return 0
    fi

    if [[ -n "$AUTHORIZED_KEY_URL" ]]; then
        if command -v curl >/dev/null 2>&1; then
            curl -fsSL "$AUTHORIZED_KEY_URL"
            return 0
        fi
        if command -v wget >/dev/null 2>&1; then
            wget -qO- "$AUTHORIZED_KEY_URL"
            return 0
        fi
        deploy_error "Neither curl nor wget is available to fetch $AUTHORIZED_KEY_URL"
    fi

    return 1
}

install_base_services() {
    deploy_info "Installing SSH and mDNS services"
    deploy_preflight_time_sync run_root
    deploy_apt_update run_root
    run_root apt-get install -y sudo curl openssh-server avahi-daemon
    run_root systemctl enable --now ssh
    run_root systemctl enable --now avahi-daemon
}

install_avahi_ssh_service() {
    local service_path="/etc/avahi/services/ssh.service"
    local tmpfile
    tmpfile="$(mktemp)"
    cat > "$tmpfile" <<'EOF'
<?xml version="1.0" standalone='no'?>
<!DOCTYPE service-group SYSTEM "avahi-service.dtd">
<service-group>
  <name replace-wildcards="yes">%h</name>
  <service>
    <type>_ssh._tcp</type>
    <port>22</port>
  </service>
</service-group>
EOF

    deploy_info "Installing Avahi SSH service advertisement"
    run_root install -d -m 0755 /etc/avahi/services
    run_root install -m 0644 "$tmpfile" "$service_path"
    rm -f "$tmpfile"
    run_root systemctl restart avahi-daemon
}

configure_hostname() {
    local new_hostname="$1"
    local current_hostname
    current_hostname="$(hostname)"

    if [[ "$current_hostname" != "$new_hostname" ]]; then
        deploy_info "Setting hostname to $new_hostname"
        run_root hostnamectl set-hostname "$new_hostname"
    else
        deploy_info "Hostname already set to $new_hostname"
    fi

    local hosts_tmp
    hosts_tmp="$(mktemp)"
    awk '$1 != "127.0.1.1"' /etc/hosts > "$hosts_tmp"
    printf '127.0.1.1 %s\n' "$new_hostname" >> "$hosts_tmp"
    run_root install -m 0644 "$hosts_tmp" /etc/hosts
    rm -f "$hosts_tmp"

    run_root systemctl restart avahi-daemon
}

ensure_deploy_user() {
    local user_name="$1"
    if id -u "$user_name" >/dev/null 2>&1; then
        deploy_info "User $user_name already exists"
    else
        deploy_info "Creating user $user_name"
        run_root adduser --disabled-password --gecos "" "$user_name"
    fi
    run_root usermod -aG sudo "$user_name"
}

install_authorized_key() {
    local user_name="$1"
    local key_text="$2"
    local home_dir ssh_dir auth_keys tmpfile

    home_dir="$(resolve_user_home "$user_name")"
    ssh_dir="$home_dir/.ssh"
    auth_keys="$ssh_dir/authorized_keys"

    deploy_info "Installing authorized key for $user_name"
    run_root install -d -m 0700 -o "$user_name" -g "$user_name" "$ssh_dir"

    tmpfile="$(mktemp)"
    printf '%s\n' "$key_text" > "$tmpfile"
    if run_root test -f "$auth_keys"; then
        if run_root grep -qxF -- "$key_text" "$auth_keys"; then
            deploy_info "Authorized key already present"
            rm -f "$tmpfile"
            return 0
        fi
        run_root bash -lc "cat $(printf '%q' "$tmpfile") >> $(printf '%q' "$auth_keys")"
    else
        run_root install -m 0600 -o "$user_name" -g "$user_name" "$tmpfile" "$auth_keys"
        rm -f "$tmpfile"
        return 0
    fi
    run_root chown "$user_name:$user_name" "$auth_keys"
    run_root chmod 0600 "$auth_keys"
    rm -f "$tmpfile"
}

configure_nopasswd_sudo() {
    local user_name="$1"
    local sudoers_path tmpfile

    sudoers_path="/etc/sudoers.d/90-${user_name}-pennair"
    tmpfile="$(mktemp)"
    printf '%s ALL=(ALL) NOPASSWD:ALL\n' "$user_name" > "$tmpfile"

    deploy_info "Installing passwordless sudo policy for $user_name"
    run_root install -m 0440 "$tmpfile" "$sudoers_path"
    run_root visudo -cf "$sudoers_path" >/dev/null
    rm -f "$tmpfile"
}

run_bootstrap() {
    local user_name="$1"
    local deploy_root="$2"
    local bootstrap_args=(
        --user "$user_name"
        --service "$SERVICE_NAME"
        --deploy-root "$deploy_root"
        --enable
    )

    if [[ "$START_SERVICE" == "1" ]]; then
        bootstrap_args+=(--start)
    else
        bootstrap_args+=(--no-start)
    fi

    if [[ "$INSTALL_PIGPIO" != "1" ]]; then
        bootstrap_args+=(--no-pigpio)
    fi

    if [[ -n "$TRAVEL_ROUTER_SSID" ]]; then
        bootstrap_args+=(--travel-router-ssid "$TRAVEL_ROUTER_SSID")
    fi

    if [[ -n "$TRAVEL_ROUTER_PSK" ]]; then
        bootstrap_args+=(--travel-router-psk "$TRAVEL_ROUTER_PSK")
    fi

    bootstrap_args+=(--network-role "$DEFAULT_NETWORK_ROLE")
    bootstrap_args+=(--ap-selection "$AP_SELECTION")

    if [[ -n "$ALLOWED_AP_HOSTS" ]]; then
        bootstrap_args+=(--allowed-ap-hosts "$ALLOWED_AP_HOSTS")
    fi

    if [[ -n "$FALLBACK_PSK" ]]; then
        bootstrap_args+=(--fallback-psk "$FALLBACK_PSK")
    fi

    if [[ -n "$FALLBACK_AP_PREFIX" ]]; then
        bootstrap_args+=(--fallback-ap-prefix "$FALLBACK_AP_PREFIX")
    fi

    if [[ -n "$LOCAL_AP_SSID" ]]; then
        bootstrap_args+=(--local-ap-ssid "$LOCAL_AP_SSID")
    fi

    if [[ "$INSTALL_WIFI_FAILOVER" != "1" ]]; then
        bootstrap_args+=(--no-wifi-failover)
    fi

    deploy_info "Running hardware bootstrap"
    "$SCRIPT_DIR/bootstrap-pi.sh" "${bootstrap_args[@]}"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --hostname)
            HOSTNAME_VALUE="$2"
            shift 2
            ;;
        --user)
            DEPLOY_USER="$2"
            shift 2
            ;;
        --authorized-key-file)
            AUTHORIZED_KEY_FILE="$2"
            shift 2
            ;;
        --authorized-key)
            AUTHORIZED_KEY_TEXT="$2"
            shift 2
            ;;
        --authorized-key-url)
            AUTHORIZED_KEY_URL="$2"
            shift 2
            ;;
        --allow-placeholder-hostname)
            ALLOW_PLACEHOLDER_HOSTNAME=1
            shift
            ;;
        --deploy-root)
            DEPLOY_ROOT="$2"
            shift 2
            ;;
        --service)
            SERVICE_NAME="$2"
            shift 2
            ;;
        --travel-router-ssid)
            TRAVEL_ROUTER_SSID="$2"
            shift 2
            ;;
        --travel-router-psk)
            TRAVEL_ROUTER_PSK="$2"
            shift 2
            ;;
        --network-role)
            DEFAULT_NETWORK_ROLE="$2"
            shift 2
            ;;
        --allowed-ap-hosts)
            ALLOWED_AP_HOSTS="$2"
            shift 2
            ;;
        --ap-selection)
            AP_SELECTION="$2"
            shift 2
            ;;
        --fallback-psk)
            FALLBACK_PSK="$2"
            shift 2
            ;;
        --fallback-ap-prefix)
            FALLBACK_AP_PREFIX="$2"
            shift 2
            ;;
        --local-ap-ssid)
            LOCAL_AP_SSID="$2"
            shift 2
            ;;
        --no-wifi-failover)
            INSTALL_WIFI_FAILOVER=0
            shift
            ;;
        --no-nopasswd-sudo)
            ENABLE_NOPASSWD_SUDO=0
            shift
            ;;
        --no-bootstrap)
            RUN_BOOTSTRAP=0
            shift
            ;;
        --start)
            START_SERVICE=1
            shift
            ;;
        --no-pigpio)
            INSTALL_PIGPIO=0
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            deploy_error "Unknown argument: $1"
            ;;
    esac
done

HOSTNAME_VALUE="$(validate_hostname "$HOSTNAME_VALUE")"
warn_placeholder_hostname "$HOSTNAME_VALUE"
ensure_root_access
ensure_deploy_user "$DEPLOY_USER"

if [[ -z "$DEPLOY_ROOT" ]]; then
    DEPLOY_ROOT="$(resolve_user_home "$DEPLOY_USER")/pennair-deploy"
fi

install_base_services
install_avahi_ssh_service
configure_hostname "$HOSTNAME_VALUE"

if key_text="$(read_authorized_key)"; then
    install_authorized_key "$DEPLOY_USER" "$key_text"
else
    deploy_warn "No SSH authorized key provided. SSH key auth will need to be configured separately."
fi

if [[ "$ENABLE_NOPASSWD_SUDO" == "1" ]]; then
    configure_nopasswd_sudo "$DEPLOY_USER"
fi

if [[ "$RUN_BOOTSTRAP" == "1" ]]; then
    run_bootstrap "$DEPLOY_USER" "$DEPLOY_ROOT"
else
    deploy_info "Skipping hardware bootstrap"
fi

printf '\n'
deploy_info "Provisioning complete"
printf 'Hostname: %s.local\n' "$HOSTNAME_VALUE"
printf 'Deploy user: %s\n' "$DEPLOY_USER"
printf 'Deploy root: %s\n' "$DEPLOY_ROOT"
printf 'SSH target: %s@%s.local\n' "$DEPLOY_USER" "$HOSTNAME_VALUE"
printf '\n'
printf 'Next steps:\n'
printf '  1. Add %s.local to the integration inventory.\n' "$HOSTNAME_VALUE"
printf '  2. Use target_id %s in inventory for a clean hostname-to-target mapping.\n' "$HOSTNAME_VALUE"
printf '  3. Set fleet_file, vehicle_name, and overlay_yaml in the dashboard.\n'
