#!/usr/bin/env bash
# launch.sh — Start PennAiR Auton Deploy
#
# Usage: ./launch.sh
#
# Dev launcher: starts both backend and frontend together.
#
# Requirements:
#   - One of:
#       - an active conda env with backend deps installed, or
#       - `uv` installed: curl -LsSf https://astral.sh/uv/install.sh | sh
#   - Node.js + npm installed
#   - Frontend deps installed (`cd frontend && npm install`)
#
# Environment variables (optional):
#   INSTALL_DIR       Build install directory (default: ~/ros2_ws)
#   GITHUB_REPO       GitHub repo for build downloads (default: auto-detect)
#   HOTSPOT_CON_NAME  NetworkManager hotspot connection name (default: Hotspot)
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FRONTEND_DIR="$SCRIPT_DIR/frontend"
cd "$SCRIPT_DIR"

if ! command -v npm >/dev/null 2>&1; then
  echo "Error: npm not found. Install Node.js and npm first."
  exit 1
fi

if [ ! -d "$FRONTEND_DIR/node_modules" ]; then
  echo "Frontend dependencies missing. Installing..."
  cd "$FRONTEND_DIR"
  npm install
  cd "$SCRIPT_DIR"
fi

# Ensure required frontend deps are present even if node_modules already exists.
if ! npm --prefix "$FRONTEND_DIR" ls --depth=0 @xterm/xterm >/dev/null 2>&1; then
  echo "Frontend dependencies out of date. Running npm install..."
  npm --prefix "$FRONTEND_DIR" install
fi

if [ -n "${CONDA_PREFIX:-}" ]; then
  BACKEND_CMD=(python app.py)
  BACKEND_LABEL="python app.py (conda env: $(basename "$CONDA_PREFIX"))"
elif command -v uv >/dev/null 2>&1; then
  BACKEND_CMD=(uv run app.py)
  BACKEND_LABEL="uv run app.py"
else
  echo "Error: no backend runner available."
  echo "Activate a conda env with backend deps, or install uv:"
  echo "  curl -LsSf https://astral.sh/uv/install.sh | sh"
  exit 1
fi

echo "Starting backend on http://0.0.0.0:8080 ($BACKEND_LABEL)"
"${BACKEND_CMD[@]}" &
BACKEND_PID=$!

cleanup() {
  if kill -0 "$BACKEND_PID" 2>/dev/null; then
    kill "$BACKEND_PID" 2>/dev/null || true
    wait "$BACKEND_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting frontend on http://0.0.0.0:3000"
cd "$FRONTEND_DIR"
npm run dev -- --host 0.0.0.0 --port 3000
