#!/usr/bin/env bash
set -euo pipefail

CHECK=false
if [[ "${1:-}" == "--check" ]]; then
  CHECK=true
fi

FILES=$(git ls-files | grep -E '\.(cpp|h|hpp|cc)$')

if [[ -z "$FILES" ]]; then
  echo "No matching files found."
  exit 0
fi

if $CHECK; then
  echo "Checking clang-format"
  echo "$FILES" | xargs clang-format --dry-run --Werror --verbose
else
  echo "Applying clang-format..."
  echo "$FILES" | xargs clang-format -i --style=file
fi
