#!/bin/bash

# Stop if a setup command fails.
# This does not undo commands that already succeeded.
set -e

# Keep output in English for the "Joined IBSS" check below.
export LC_ALL=C

# Network configuration requires administrator privileges.
if [ "$EUID" -ne 0 ]; then
    echo "Run with sudo: sudo bash setp_mesh.sh"
    exit 1
fi

# Read the saved hostname, for example air-06.
PI_HOSTNAME=$(cat /etc/hostname)

if [[ ! "$PI_HOSTNAME" =~ ^air-([0-9]{2,3})$ ]]; then
    echo "Invalid hostname: $PI_HOSTNAME"
    echo "Expected air-01 through air-254."
    exit 1
fi

# Convert 06 to 6, explicitly using decimal.
NODE_ID=$((10#${BASH_REMATCH[1]}))

# Enforce consistent names: air-05, not air-005.
if [[ "$PI_HOSTNAME" != "$(printf 'air-%02d' "$NODE_ID")" ]]; then
    echo "Hostname must use the format air-05, air-06, ... air-254."
    exit 1
fi

# Accept 1-3 digits, without leading zeros.
# Checking the format first makes the numeric comparison safe.
if ! [[ "$NODE_ID" =~ ^[1-9][0-9]{0,2}$ ]]; then
    echo "Use a whole number from 1 to 254, without leading zeros."
    exit 1
fi

if (( NODE_ID > 254 )); then
    echo "Node number must be from 1 to 254."
    exit 1
fi

# Insert the node number into the last part of the address.
MESH_IP="10.67.0.${NODE_ID}/24"



# Check that wlan0 exists before changing the network.
ip link show dev wlan0 > /dev/null


# WORKAROUND FOR NETWORKD RACE CONDITION
# Keep systemd-networkd from changing our mesh interfaces.
NETWORKD_RULE="/etc/systemd/network/01-force-pi-mesh-unmanaged.network"

# Install the rule once. Preserve it if already present.
if [ ! -e "$NETWORKD_RULE" ]; then
    mkdir -p /etc/systemd/network

    cat > "$NETWORKD_RULE" <<'EOF'
[Match]
Name=wlan0 bat0

[Link]
Unmanaged=yes
EOF
fi

# If networkd is running, tell it to read the rule now.
if systemctl is-active --quiet systemd-networkd; then
    networkctl reload

    # Check up to ten times that networkd has released wlan0.
    for attempt in {1..10}; do
        if networkctl list wlan0 --no-legend --no-pager |
            grep -q 'unmanaged'; then
            break
        fi

        if [ "$attempt" -eq 10 ]; then
            echo "networkd has not released wlan0. Stopping mesh setup."
            exit 1
        fi

        sleep 1
    done
fi


echo "Setting up $MESH_IP. Existing Wi-Fi/mesh connections will be interrupted."

# 1. Load the kernel component that handles mesh networking.
modprobe batman-adv

# 2. Tell NetworkManager to leave the Wi-Fi radio alone.
nmcli device set wlan0 managed no

# 3. Remove an existing bat0 so we can rerun this script.
# This also detaches wlan0, but does not delete wlan0.
if ip link show dev bat0 > /dev/null 2>&1; then
    batctl meshif bat0 interface destroy
fi

# Leave an earlier IBSS session if one is already active.
if iw dev wlan0 link | grep -q "Joined IBSS"; then
    iw dev wlan0 ibss leave
fi

# 4. Disable wlan0 and remove addresses from its previous use.
ip link set wlan0 down
ip addr flush dev wlan0

# 5. Change the radio to IBSS mode, then enable it again.
iw dev wlan0 set type ibss
ip link set wlan0 up

# Both Pis must use the same name, frequency, and BSSID.
iw dev wlan0 ibss join pi-mesh-network 2412 fixed-freq 02:12:34:56:78:90

# 6. Create the software mesh interface using BATMAN_IV.
batctl meshif bat0 interface create routing_algo BATMAN_IV

# Attach the Wi-Fi radio to the mesh interface.
# -M prevents automatic creation: we already created bat0.
batctl meshif bat0 interface -M add wlan0

# 7. Leave room for mesh headers and enable bat0.
ip link set bat0 mtu 1400
ip link set bat0 up

# Put this Pi's IP address on bat0.
ip address add "$MESH_IP" dev bat0

# 8. Display the local configuration for inspection.
# The other Pi does not need to be online for setup.
echo "Setup commands completed. Local configuration:"
iw dev wlan0 info
iw dev wlan0 link
ip -br address show bat0
batctl meshif bat0 interface