#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ci.conf"

APT_SOURCES_DIR="/etc/apt/sources.list.d"

for filename in "${!APT_SOURCES[@]}"; do
    filepath="$APT_SOURCES_DIR/$filename"

    echo "Adding APT source [$filepath]: ${APT_SOURCES[$filename]}"
    echo "${APT_SOURCES[$filename]}" | sudo tee "$filepath" > /dev/null
done
