#!/bin/bash
# launch.sh — Start the SAE Deploy Dashboard
#
# Usage: ./launch.sh
#
# Just needs `uv` installed: curl -LsSf https://astral.sh/uv/install.sh | sh
#
# Environment variables (optional):
#   INSTALL_DIR       Build install directory (default: ~/ros2_ws)
#   GITHUB_REPO       GitHub repo for build downloads (default: auto-detect)
#   HOTSPOT_CON_NAME  NetworkManager hotspot connection name (default: Hotspot)
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Starting SAE Deploy Dashboard on http://0.0.0.0:8080"
uv run app.py
