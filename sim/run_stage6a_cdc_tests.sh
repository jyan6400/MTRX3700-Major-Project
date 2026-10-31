#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build/stage6a_cdc
for config in "13 10" "5 17" "17 5" "7 7"; do
  read -r sh dh <<< "$config"
  name="src${sh}_dst${dh}"
  echo "== Stage 6A CDC: $name =="
  verilator --binary --timing -Wno-fatal -O2 \
    --top-module temporal_result_cdc_stage6a_tb \
    -GSRC_HALF="$sh" -GDST_HALF="$dh" \
    --Mdir "build/stage6a_cdc/$name" \
    rtl/integration/temporal_result_cdc.sv \
    sim/integration/temporal_result_cdc_stage6a_tb.sv \
    > "build/stage6a_cdc/$name.compile.log" 2>&1
  "build/stage6a_cdc/$name/Vtemporal_result_cdc_stage6a_tb" \
    > "build/stage6a_cdc/$name.log" 2>&1
  grep '^PASS: temporal_result_cdc_stage6a_tb' "build/stage6a_cdc/$name.log"
done
echo 'PASS: Stage 6A CDC regression completed across four clock ratios'
