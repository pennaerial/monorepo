#!/bin/bash
set -euo pipefail

if (( EUID != 0 )); then
    echo "Run with sudo: sudo bash pi-setup/install.sh"
    exit 1
fi

SETUP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

install -D -o root -g root -m 0755 \
    "$SETUP_DIR/setup_mesh.sh" \
    /usr/local/sbin/setup_mesh.sh

install -D -o root -g root -m 0644 \
    "$SETUP_DIR/pi-mesh.service" \
    /etc/systemd/system/pi-mesh.service

systemctl daemon-reload
systemctl enable pi-mesh.service

echo "Installed. Reboot when ready."