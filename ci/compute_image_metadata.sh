#!/usr/bin/env bash

# Script that computes the current state of monorepo, for version tracking on docker images
# Tracks the most up-to-date main commit SHA, and also tracks the most up-to-date commit SHAs of
# each managed submodule's remote branch (not monorepo's currently pinned tag for that submodule)

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ci.conf"

main_sha=$(git ls-remote origin main | cut -f1)
echo "main branch tag: $main_sha"

# --remote checks out latest commit on submodule's configured branch in monorepo
declare -A managed_submodule_shas
for submodule_path in "${MANAGED_SUBMODULES[@]}"; do
  git submodule update --init --remote -- "$submodule_path"
  sha=$(git -C "$submodule_path" rev-parse HEAD)
  managed_submodule_shas["$submodule_path"]="$sha"
  echo "$submodule_path sha: $sha"
done


# --- Build submodules JSON object ---
submodules_json="{}"
for submodule_path in "${MANAGED_SUBMODULES[@]}"; do
  submodules_json=$(jq -n \
    --argjson existing "$submodules_json" \
    --arg path "$submodule_path" \
    --arg sha "${managed_submodule_shas[$submodule_path]}" \
    '$existing + {($path): $sha}')
done


# --- Combined content-addressed tag: order-independent, count-independent ---
combined_input="main:${main_sha}"$'\n'
for submodule_path in "${MANAGED_SUBMODULES[@]}"; do
  combined_input+="${submodule_path}:${managed_submodule_shas[$submodule_path]}"$'\n'
done
combined_tag=$(echo "$combined_input" | sort | sha256sum | cut -c1-12)
echo "combined tag: $combined_tag"

# --- Write final JSON ---
OUT_JSON="${SCRIPT_DIR}/version.json"
jq -n \
  --arg main_sha "$main_sha" \
  --argjson submodules "$submodules_json" \
  --arg combined_tag "$combined_tag" \
  '{main_sha: $main_sha, managed_submodules: $submodules, combined_tag: $combined_tag}' \
  > "$OUT_JSON"

echo "Wrote $OUT_JSON"

