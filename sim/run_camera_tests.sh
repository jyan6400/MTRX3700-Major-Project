#!/usr/bin/env bash
# Camera capture / stream tests (M1). Run from the repository root.
# Verilator 5.050. Prints PASS/FAIL per bench and CAUGHT for the broken copy.
# Exit status 0 only if every bench passes and the broken copy is caught.
#
#   ./sim/run_camera_tests.sh
set -u
cd "$(dirname "$0")/.."
B=build/camera_tests
mkdir -p "$B"
status=0

run() {   # run <name> <top> <files...>; returns the simulation's exit status
    local name=$1 top=$2; shift 2
    mkdir -p "$B/$name"
    if ! verilator --binary --timing -Wno-fatal -Wno-CASEINCOMPLETE -O1 \
            --top-module "$top" --Mdir "$B/$name" "$@" > "$B/$name.compile.log" 2>&1; then
        echo "FAIL: $name (compile, see $B/$name.compile.log)"
        return 2
    fi
    "./$B/$name/V$top" > "$B/$name.log" 2>&1
    local rc=$?
    if [ $rc -eq 0 ] && grep -q "^PASS: $top" "$B/$name.log"; then return 0; fi
    return 1
}

DOWNSTREAM="rtl/image_processing/red_mask.sv rtl/perception/led_centroid.sv"

if run adapter_unit camera_stream_adapter_tb \
        rtl/integration/camera_stream_adapter.sv $DOWNSTREAM sim/unit/camera_stream_adapter_tb.sv; then
    echo "PASS: camera_stream_adapter_tb ($(grep -c '^PASS \[' $B/adapter_unit.log) scenarios)"
else echo "FAIL: camera_stream_adapter_tb (see $B/adapter_unit.log)"; status=1; fi

if run capture_vision camera_capture_vision_tb \
        rtl/integration/camera_stream_adapter.sv $DOWNSTREAM \
        vendor/camera/rtl/ov7670_capture.sv vendor/camera/tb/ov7670_model.sv \
        sim/integration/camera_capture_vision_tb.sv; then
    echo "PASS: camera_capture_vision_tb (2 full 640x480 frames from the camera model)"
else echo "FAIL: camera_capture_vision_tb (see $B/capture_vision.log)"; status=1; fi

# Deliberately broken module: the bench must FAIL on it.
run adapter_broken camera_stream_adapter_tb \
        sim/broken/camera_stream_adapter_broken.sv $DOWNSTREAM sim/unit/camera_stream_adapter_tb.sv
rc=$?
if [ $rc -eq 1 ]; then
    echo "CAUGHT: camera_stream_adapter_broken (truncated frame never closed): $(grep -m1 -o 'FAIL[^"]*' $B/adapter_broken.log | head -c 100)"
else
    echo "FAIL: broken camera_stream_adapter was NOT caught"; status=1
fi

exit $status
