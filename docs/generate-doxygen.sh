#!/usr/bin/env bash
set -euo pipefail
DOCS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${PENNAIR_PAYLOAD_CONTROLLER_PATH:-}" ]]; then
    source "$DOCS_DIR/../dev_env.sh"
fi
DOXYFILE="${PENNAIR_PAYLOAD_CONTROLLER_PATH}/Doxyfile"
if command -v doxygen >/dev/null 2>&1; then
    echo "Generating Payload Controller API documentation..."
    doxygen "$DOXYFILE"
else
    XML_INDEX="$PENNAIR_PAYLOAD_CONTROLLER_PATH/doxygen/xml/index.xml"
    if [[ -f "$XML_INDEX" ]] && [[ "$DOXYFILE" -ot "$XML_INDEX" ]] &&
       [[ -z "$(find "$PENNAIR_PAYLOAD_CONTROLLER_PATH/main" "$PENNAIR_PAYLOAD_CONTROLLER_PATH/components" -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.c' -o -name '*.h' \) -newer "$XML_INDEX" -print -quit)" ]]; then
        echo "Doxygen is unavailable; reusing existing XML (no newer C/C++ source found)."
    else
        echo "Doxygen is required to create or refresh the C++ reference. On Ubuntu: sudo apt install doxygen" >&2
        exit 1
    fi
fi
