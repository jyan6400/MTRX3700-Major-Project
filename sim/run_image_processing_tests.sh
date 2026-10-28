#!/usr/bin/env bash
# Image-processing (M2, Wilson) regression. Run from anywhere; Verilator 5.x.
#
#   ./sim/run_image_processing_tests.sh          # all benches
#   ./sim/run_image_processing_tests.sh quick    # skip the full-top camera-pin bench
#
# Runs the new V2 image-processing benches AND every existing bench that
# instantiates red_mask (directly or through major_project_top), so a change
# to the mask is checked against the whole design. Prints PASS/FAIL per
# bench; logs go to build/image_processing_tests/<name>.log.
# Exit status 0 only if every bench passes, except the KNOWN_FAIL ones:
# benches that already fail on main without this branch (listed below with
# the reason), reported as KNOWN-FAIL so they stay visible.
set -u
cd "$(dirname "$0")/.."
ulimit -s unlimited 2>/dev/null || true
B=build/image_processing_tests
mkdir -p "$B"
status=0
MODE=${1:-all}

# Fails identically on main @058bd48 (before this branch): written for the V1
# led_centroid; Marcus's V2 shape/fill gates (PR #4) now reject its 3-pixel
# scattered target. Owner M3 to update the bench.
KNOWN_FAIL=""

RUNARGS=""   # plusargs for the next run() only
run() {   # run <name> <top> <files...>
    local name=$1 top=$2; shift 2
    local args=$RUNARGS; RUNARGS=""
    mkdir -p "$B/$name"
    if ! verilator --binary --timing -Wno-fatal -Wno-CASEINCOMPLETE -O2 \
            --top-module "$top" --Mdir "$B/$name" "$@" > "$B/$name.compile.log" 2>&1; then
        echo "FAIL: $name (compile, see $B/$name.compile.log)"; status=1; return
    fi
    "./$B/$name/V$top" $args > "$B/$name.log" 2>&1
    local rc=$?
    if [ $rc -eq 0 ] && grep -q "^PASS: $top" "$B/$name.log"; then
        echo "$(grep -m1 "^PASS: $top" "$B/$name.log")"
    elif [[ " $KNOWN_FAIL " == *" $name "* ]]; then
        echo "KNOWN-FAIL: $name (fails on main too, see KNOWN_FAIL above; $B/$name.log)"
    else
        echo "FAIL: $name (exit $rc, see $B/$name.log)"; status=1
    fi
}

IP=rtl/image_processing
MASK="$IP/led_colour_mask.sv $IP/red_mask.sv"
TRI="$IP/led_colour_mask.sv $IP/colour_run_filter.sv rtl/perception/led_centroid.sv $IP/tri_led_detect.sv"

TOP_RTL="rtl/integration/major_project_top.sv rtl/integration/camera_stream_adapter.sv rtl/integration/camera_health_monitor.sv
  rtl/integration/vision_result_cdc.sv rtl/integration/sonar_result_cdc.sv
  $MASK rtl/perception/led_centroid.sv
  rtl/navigation/camera_frame_watchdog.sv rtl/navigation/steering_controller.sv rtl/navigation/docking_fsm.sv
  rtl/actuation/sonar_freshness.sv rtl/actuation/sonar_filter.sv rtl/actuation/ir_safety_latch.sv rtl/actuation/motion_interface.sv rtl/actuation/motion_supervisor.sv
  vendor/camera/rtl/video_pll.sv vendor/camera/rtl/clock_divider.sv vendor/camera/rtl/ov7670_config.sv
  vendor/camera/rtl/ov7670_capture.sv vendor/camera/rtl/i2c_master.sv
  vendor/ultrasonic/rtl/sonar_pll.sv vendor/ultrasonic/rtl/sonar_range.sv
  vendor/ir/rtl/nec_decoder.sv vendor/chassis/rtl/chassis_drive.sv vendor/chassis/rtl/uart_tx.sv"

# Test frames from the lab photo (generated, not committed: ~16 MB).
if [ ! -f sim/data/photo_leds_scene.hex ]; then
    python3 sim/tools/make_led_frames.py > "$B/make_led_frames.log" 2>&1 \
        || echo "NOTE: could not build sim/data frames (needs numpy + Pillow); photo benches will fail"
fi

echo "== New V2 image-processing benches =="
run red_mask_v1_equiv   red_mask_v1_equiv_tb  $MASK sim/unit/red_mask_v1_ref.sv sim/unit/red_mask_v1_equiv_tb.sv
run led_colour_mask     led_colour_mask_tb    $IP/led_colour_mask.sv sim/unit/led_colour_mask_tb.sv
run tri_led_detect      tri_led_detect_tb     $TRI sim/unit/tri_led_detect_tb.sv
mkdir -p "$B/dumps"; RUNARGS="+DUMP_DIR=$B/dumps"
run tri_led_photo       tri_led_photo_tb      $TRI sim/integration/tri_led_photo_tb.sv
if python3 sim/tools/render_led_masks.py "$B/dumps" "$B/tri_led_photo.log" "$B" > "$B/render.log" 2>&1; then
    echo "      mask pictures: $B/photo_leds_masking.png, $B/photo_only_masking.png"
fi

echo "== Existing benches that use red_mask =="
run red_mask            red_mask_tb           $MASK sim/unit/red_mask_tb.sv
run red_vision_pipeline red_vision_pipeline_tb $MASK rtl/perception/led_centroid.sv sim/integration/red_vision_pipeline_tb.sv
run adapter_unit        camera_stream_adapter_tb rtl/integration/camera_stream_adapter.sv $MASK \
    rtl/perception/led_centroid.sv sim/unit/camera_stream_adapter_tb.sv
run capture_vision      camera_capture_vision_tb rtl/integration/camera_stream_adapter.sv $MASK \
    rtl/perception/led_centroid.sv vendor/camera/rtl/ov7670_capture.sv vendor/camera/tb/ov7670_model.sv \
    sim/integration/camera_capture_vision_tb.sv
run major_project_top   major_project_top_tb  $TOP_RTL sim/integration/major_project_top_tb.sv
run top_uart_safety     top_uart_safety_tb    $TOP_RTL vendor/chassis/tb/ugv02_model.sv sim/integration/top_uart_safety_tb.sv
run top_uart_latency    top_uart_latency_tb   $TOP_RTL vendor/chassis/tb/ugv02_model.sv sim/integration/top_uart_latency_tb.sv
run top_r9_bypass      top_r9_bypass_tb    $TOP_RTL sim/integration/top_r9_bypass_tb.sv

if [ "$MODE" != "quick" ]; then
    echo "== Full top level from the camera pins (real vendor models) =="
    run top_led_photo   top_led_photo_tb      $TOP_RTL vendor/camera/tb/ov7670_model.sv \
        vendor/ultrasonic/hcsr04_model.sv vendor/ir/tb/ir_remote_model.sv vendor/chassis/tb/ugv02_model.sv \
        sim/integration/top_led_photo_tb.sv
fi

exit $status
