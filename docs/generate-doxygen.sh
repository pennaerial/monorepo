#!/usr/bin/env bash

set -euo pipefail

echo "==> Generating Payload Controller API documentation..."

DOXYFILE="${PENNAIR_PAYLOAD_CONTROLLER_PATH}/Doxyfile"
doxygen "${DOXYFILE}"

echo "Doxygen XML generation complete."
