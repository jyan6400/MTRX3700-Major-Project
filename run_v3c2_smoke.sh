#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
# Place this script in repository root; compile with the exact RTL list
# already maintained in run_safety_tests.sh.
source <(sed -n '/^IP=rtl\/image_processing/,/^echo "== Lint/ { /^echo "== Lint/!p; }' sim/run_safety_tests.sh)
mkdir -p build/v3c2_smoke
# TOP_RTL contains only repository-controlled RTL; bench is separately specified.
# shellcheck disable=SC2086
verilator --binary --timing -Wall -Wno-fatal --top-module top_multi_bay_smoke_tb \
  --Mdir build/v3c2_smoke $TOP_RTL \
  sim/integration/top_multi_bay_smoke_tb.sv
./build/v3c2_smoke/Vtop_multi_bay_smoke_tb
