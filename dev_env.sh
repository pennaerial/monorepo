#!/usr/bin/env bash

export MONOREPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PENNAIR_GZ_MODELS_PATH="${MONOREPO_ROOT}/gz-models"
export PENNAIR_PX4_PATH="${MONOREPO_ROOT}/PX4-Autopilot"
export PENNAIR_PAYLOAD_CONTROLLER_PATH="${MONOREPO_ROOT}/payload_controller"
