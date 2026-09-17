#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../dev_env.sh"

bash "${SCRIPT_DIR}/apt_sources.sh"
sudo apt-get update
sudo apt-get install -y --no-install-recommends eim-cli direnv

command -v eim

eim install --config "$PENNAIR_PAYLOAD_CONTROLLER_PATH/eim_config.toml"
test -f "$HOME/.espressif/tools/activate_idf_v6.0.2.sh"

cd "$PENNAIR_PAYLOAD_CONTROLLER_PATH"

direnv allow .
direnv exec . command -v idf.py
direnv exec . idf.py --version

direnv exec . make linux
direnv exec . make esp32s3