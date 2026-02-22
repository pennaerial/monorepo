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
#   - Package manager recommended for sshpass auto-install (brew/apt/dnf)
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

ensure_python_backend_deps() {
  local python_bin="${1:-python}"
  local missing_modules=""

  if ! missing_modules="$(
    "$python_bin" - <<'PY'
import importlib.util

required = ["fastapi", "httpx", "multipart", "uvicorn"]
missing = [name for name in required if importlib.util.find_spec(name) is None]
if missing:
    print(" ".join(missing))
    raise SystemExit(1)
PY
  )"; then
    echo "Backend Python dependencies missing (${missing_modules:-unknown}). Installing..."
    if ! "$python_bin" -m pip install "fastapi[standard]" python-multipart httpx; then
      echo "Error: failed to install backend Python dependencies."
      echo "Try manually:"
      echo "  $python_bin -m pip install \"fastapi[standard]\" python-multipart httpx"
      exit 1
    fi
  fi
}

ensure_sshpass() {
  if command -v sshpass >/dev/null 2>&1; then
    return
  fi

  echo "sshpass not found."
  local install_attempted=0
  local install_ok=1

  if command -v brew >/dev/null 2>&1; then
    install_attempted=1
    echo "Installing sshpass with Homebrew..."
    if ! brew list hudochenkov/sshpass/sshpass >/dev/null 2>&1; then
      if ! brew install hudochenkov/sshpass/sshpass; then
        install_ok=0
        echo "Warning: failed to install sshpass via Homebrew."
      fi
    fi
  fi

  if ! command -v sshpass >/dev/null 2>&1 && command -v apt-get >/dev/null 2>&1; then
    install_attempted=1
    echo "Installing sshpass with apt..."
    if [ "$(id -u)" -eq 0 ]; then
      if ! apt-get update || ! apt-get install -y sshpass; then
        install_ok=0
        echo "Warning: failed to install sshpass via apt."
      fi
    elif command -v sudo >/dev/null 2>&1; then
      if ! sudo apt-get update || ! sudo apt-get install -y sshpass; then
        install_ok=0
        echo "Warning: failed to install sshpass via sudo apt."
      fi
    else
      install_ok=0
      echo "Warning: apt is available but sudo is not. Install sshpass manually."
    fi
  fi

  if ! command -v sshpass >/dev/null 2>&1 && command -v dnf >/dev/null 2>&1; then
    install_attempted=1
    echo "Installing sshpass with dnf..."
    if [ "$(id -u)" -eq 0 ]; then
      if ! dnf install -y sshpass; then
        install_ok=0
        echo "Warning: failed to install sshpass via dnf."
      fi
    elif command -v sudo >/dev/null 2>&1; then
      if ! sudo dnf install -y sshpass; then
        install_ok=0
        echo "Warning: failed to install sshpass via sudo dnf."
      fi
    else
      install_ok=0
      echo "Warning: dnf is available but sudo is not. Install sshpass manually."
    fi
  fi

  if ! command -v sshpass >/dev/null 2>&1; then
    if [ "$install_attempted" -eq 0 ]; then
      echo "Warning: no supported package manager found for sshpass auto-install."
    elif [ "$install_ok" -eq 0 ]; then
      echo "Warning: sshpass auto-install failed."
    fi
    echo "Warning: sshpass is still unavailable. SSH key auth will work; password auth may fail."
  fi
}

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
  ensure_python_backend_deps python
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

ensure_sshpass

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
