#!/usr/bin/env bash
set -euo pipefail
DOCS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$DOCS_DIR/../dev_env.sh"
cd "$DOCS_DIR"
DOCS_ENV="$DOCS_DIR/.venv"
if [[ ! -x "$DOCS_ENV/bin/python" ]]; then
    if command -v uv >/dev/null 2>&1; then
        uv venv --python 3.12 "$DOCS_ENV"
    else
        python3 -m venv "$DOCS_ENV"
    fi
fi
if [[ ! -f "$DOCS_ENV/.docs-requirements" ]] || ! cmp -s docs-requirements.txt "$DOCS_ENV/.docs-requirements"; then
    if command -v uv >/dev/null 2>&1; then
        uv pip install --python "$DOCS_ENV/bin/python" -r docs-requirements.txt
    else
        "$DOCS_ENV/bin/python" -m pip install -r docs-requirements.txt
    fi
    cp docs-requirements.txt "$DOCS_ENV/.docs-requirements"
fi
export PATH="$DOCS_ENV/bin:$PATH"
export SKIP_AUTOAPI_BUILD="${SKIP_AUTOAPI_BUILD:-1}"
# -E -a rebuilds cached pages, including previews created before Mermaid was added.
exec make "${1:-serve}" SPHINXOPTS="-E -a ${SPHINXOPTS:-}"
