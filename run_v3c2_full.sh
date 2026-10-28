#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source <(sed -n '/^IP=rtl\/image_processing/,/^echo "== Lint/ { /^echo "== Lint/!p; }' sim/run_safety_tests.sh)
mkdir -p build/v3c2_full
# Same source list as the verified safety integration tests.
# Suppress fatal handling of pre-existing full-top-level lint warnings.
# shellcheck disable=SC2086
verilator --binary --timing -Wall -Wno-fatal --top-module top_multi_bay_tb \
  --Mdir build/v3c2_full $TOP_RTL sim/integration/top_multi_bay_tb.sv \
  > build/v3c2_full/compile.log 2>&1 || {
    tail -n 100 build/v3c2_full/compile.log
    exit 1
  }
./build/v3c2_full/Vtop_multi_bay_tb
