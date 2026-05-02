#!/usr/bin/env bash
set -euo pipefail
: "${PICO_SDK_PATH:?Set PICO_SDK_PATH to your pico-sdk path}"
cmake -S . -B build -G Ninja -DPICO_BOARD=pico
cmake --build build --verbose
find build -type f -name "*.uf2" -print
