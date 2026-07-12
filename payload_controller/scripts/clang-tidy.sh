#!/usr/bin/env bash
set -euo pipefail

# clang-tidy needs a compile_commands.json whose flags clang itself
# understands. The normal build/$TARGET dir is configured against GCC (or
# xtensa-esp-elf-gcc), which adds GCC-only flags (e.g.
# -fstrict-volatile-bitfields, -fno-tree-switch-conversion) that make
# clang/clang-tidy hard-error with "unknown argument". So this script
# configures a separate build dir with a clang-based compiler instead:
# the host clang for "linux" (ESP-IDF has no toolchain-clang-linux.cmake),
# and IDF_TOOLCHAIN=clang (esp-clang) for real chip targets.

TARGET="${1:-}"

case "$TARGET" in
    linux|esp32s3) ;;
    *)
        echo "Usage: $0 {linux|esp32s3}"
        exit 1
        ;;
esac

cd "$(dirname "$0")/.."

BUILD_DIR="build/${TARGET}-clang-tidy"
SDKCONFIG="sdkconfig.${TARGET}-clang-tidy"

if [[ "$TARGET" == "linux" ]]; then
    CMAKE_ARGS=(-D CMAKE_C_COMPILER=clang-18 -D CMAKE_CXX_COMPILER=clang++-18)
else
    ESP_CLANG_BIN="$(compgen -G "${IDF_TOOLS_PATH}/esp-clang/*/esp-clang/bin" | head -n1)"
    if [[ -z "$ESP_CLANG_BIN" ]]; then
        echo "Could not find esp-clang under \$IDF_TOOLS_PATH ($IDF_TOOLS_PATH)." >&2
        echo "Run 'idf_tools.py install esp-clang' and re-export your environment." >&2
        exit 1
    fi
    export PATH="$ESP_CLANG_BIN:$PATH"
    export IDF_TOOLCHAIN=clang
    CMAKE_ARGS=()
fi

if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]]; then
    echo "Initializing $TARGET clang-tidy build directory..."
    idf.py -B "$BUILD_DIR" -D "SDKCONFIG=$SDKCONFIG" "${CMAKE_ARGS[@]}" \
        $([[ "$TARGET" == "linux" ]] && echo --preview) set-target "$TARGET"
fi

idf.py -B "$BUILD_DIR" -D "SDKCONFIG=$SDKCONFIG" "${CMAKE_ARGS[@]}" reconfigure

PROJECT_ROOT="$(pwd)"

# Only check this project's own sources (main/, src/), not dependencies
# (ESP-IDF components, FreeRTOS, etc.) pulled in via compile_commands.json.
run-clang-tidy -p "$BUILD_DIR" "${PROJECT_ROOT}/(main|src)/"
