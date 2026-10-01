#!/usr/bin/env bash
set -euo pipefail

# Launch a gz world, spawn a model from gz-models/models into it, and run the
# SITL payload controller against it, so testing doesn't need sim2.launch.py.
# Usage: scripts/sim.sh <model> [world]
#   HEADLESS=1   run the gz server only (no GUI), e.g. inside a container

MODEL="${1:?Usage: $0 <model> [world]}"
WORLD="${2:-default}"
NAME="${MODEL}_0"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PC_DIR="$(dirname "$SCRIPT_DIR")"
GZ_MODELS="${PENNAIR_GZ_MODELS_PATH:-$PC_DIR/../gz-models}"
ELF="$PC_DIR/build/linux/payload_controller.elf"
MODEL_SDF="$GZ_MODELS/models/$MODEL/model.sdf"
WORLD_SDF="$GZ_MODELS/worlds/$WORLD.sdf"

[[ -f "$MODEL_SDF" ]] || { echo "No model at $MODEL_SDF" >&2; exit 1; }
[[ -f "$WORLD_SDF" ]] || { echo "No world at $WORLD_SDF" >&2; exit 1; }
[[ -x "$ELF" ]] || { echo "No binary at $ELF, run 'make linux' first" >&2; exit 1; }

export GZ_SIM_RESOURCE_PATH="$GZ_MODELS/models:$GZ_MODELS/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
# World sdfs don't declare their own systems; physics, sensors and the
# /world/<world>/create spawn service all come from this server config.
export GZ_SIM_SERVER_CONFIG_PATH="$GZ_MODELS/server.config"

world_ready() { gz service -l 2>/dev/null | grep -qx "/world/$WORLD/create"; }

GZ_PID=""
cleanup() { [[ -n "$GZ_PID" ]] && kill -- -"$GZ_PID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# Reuse a running gz instance if it already has this world, otherwise start our own.
if ! world_ready; then
    GZ_ARGS=(-r "$WORLD_SDF")
    if [[ "${HEADLESS:-0}" == 1 ]]; then
        GZ_ARGS=(-s --headless-rendering "${GZ_ARGS[@]}")
        export LIBGL_ALWAYS_SOFTWARE=1  # camera sensors still render, on the CPU
    fi
    # setsid gives gz its own process group, so cleanup kills both server and GUI
    setsid gz sim "${GZ_ARGS[@]}" &
    GZ_PID=$!

    echo "Waiting for gz world '$WORLD'..."
    for _ in $(seq 60); do world_ready && break; sleep 1; done
    world_ready || { echo "gz did not come up" >&2; exit 1; }
fi

echo "Spawning $MODEL as $NAME"
gz service -s "/world/$WORLD/create" \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$MODEL_SDF\", name: \"$NAME\", pose: {position: {z: 0.1}}"

GZ_MODEL="$NAME" GZ_WORLD="$WORLD" "$ELF"
