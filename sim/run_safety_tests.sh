#!/usr/bin/env bash
# Sensors, safety and actuation (M5, Advay) regression. Run from anywhere; Verilator 5.x.
#
#   ./sim/run_safety_tests.sh          # all benches (about 4-5 minutes)
#   ./sim/run_safety_tests.sh quick    # skip the two slow sonar pipeline benches
#
# Runs the M5 benches (V2 sonar_filter, V3 motion_supervisor) AND every existing
# bench on the sonar, IR, motion and chassis-UART path, including the full top
# level, so a change to the safety chain is checked against the whole design.
# Prints PASS/FAIL per bench; logs go to build/safety_tests/<name>.log.
# Exit status 0 only if every bench passes.
set -u
cd "$(dirname "$0")/.."
ulimit -s unlimited 2>/dev/null || true
B=build/safety_tests
mkdir -p "$B"
status=0
MODE=${1:-all}

run() {   # run <name> <top> <files...>
    local name=$1 top=$2; shift 2
    mkdir -p "$B/$name"
    if ! verilator --binary --timing -Wno-fatal -Wno-CASEINCOMPLETE -O2 \
            --top-module "$top" --Mdir "$B/$name" "$@" > "$B/$name.compile.log" 2>&1; then
        echo "FAIL: $name (compile, see $B/$name.compile.log)"; status=1; return
    fi
    "./$B/$name/V$top" > "$B/$name.log" 2>&1
    local rc=$?
    if [ $rc -eq 0 ] && grep -q "^PASS: $top" "$B/$name.log"; then
        echo "$(grep -m1 "^PASS: $top" "$B/$name.log")"
    else
        echo "FAIL: $name (exit $rc, see $B/$name.log)"; status=1
    fi
}

IP=rtl/image_processing
MASK="$IP/led_colour_mask.sv $IP/red_mask.sv"
SONAR="vendor/ultrasonic/rtl/sonar_pll.sv vendor/ultrasonic/rtl/sonar_range.sv
  vendor/ultrasonic/hcsr04_model.sv rtl/integration/sonar_result_cdc.sv rtl/actuation/sonar_freshness.sv"
CHASSIS="vendor/chassis/rtl/chassis_drive.sv vendor/chassis/rtl/uart_tx.sv vendor/chassis/tb/ugv02_model.sv"
MOTION="rtl/actuation/motion_interface.sv rtl/navigation/docking_fsm.sv rtl/navigation/steering_controller.sv"

TOP_RTL="rtl/integration/major_project_top.sv rtl/integration/camera_stream_adapter.sv
  rtl/integration/vision_result_cdc.sv rtl/integration/sonar_result_cdc.sv
  $MASK rtl/perception/led_centroid.sv
  rtl/navigation/camera_frame_watchdog.sv rtl/navigation/steering_controller.sv rtl/navigation/docking_fsm.sv
  rtl/actuation/sonar_freshness.sv rtl/actuation/sonar_filter.sv rtl/actuation/motion_supervisor.sv
  rtl/actuation/ir_safety_latch.sv rtl/actuation/motion_interface.sv
  vendor/camera/rtl/video_pll.sv vendor/camera/rtl/clock_divider.sv vendor/camera/rtl/ov7670_config.sv
  vendor/camera/rtl/ov7670_capture.sv vendor/camera/rtl/i2c_master.sv
  vendor/ultrasonic/rtl/sonar_pll.sv vendor/ultrasonic/rtl/sonar_range.sv
  vendor/ir/rtl/nec_decoder.sv vendor/chassis/rtl/chassis_drive.sv vendor/chassis/rtl/uart_tx.sv"

echo "== Lint (-Wall): M5 RTL =="
for m in sonar_filter motion_supervisor; do
    if verilator --lint-only -Wall --top-module $m rtl/actuation/$m.sv rtl/actuation/sonar_freshness.sv \
            > "$B/lint_$m.log" 2>&1; then
        echo "clean: $m.sv"
    else
        echo "FAIL: lint $m.sv (see $B/lint_$m.log)"; status=1
    fi
done

echo "== V3: reverse supervision (motion_supervisor) =="
run motion_supervisor      motion_supervisor_tb      rtl/actuation/motion_supervisor.sv sim/unit/motion_supervisor_tb.sv
run motion_supervisor_uart motion_supervisor_uart_tb rtl/actuation/motion_supervisor.sv $CHASSIS \
    sim/integration/motion_supervisor_uart_tb.sv

echo "== V2: sonar filter =="
run sonar_filter           sonar_filter_tb           rtl/actuation/sonar_filter.sv rtl/actuation/sonar_freshness.sv \
    sim/unit/sonar_filter_tb.sv
if [ "$MODE" != "quick" ]; then
    run sonar_filter_pipeline sonar_filter_pipeline_tb $SONAR rtl/actuation/sonar_filter.sv \
        sim/integration/sonar_filter_pipeline_tb.sv
fi

echo "== V1 sonar, IR and motion =="
run sonar_freshness        sonar_freshness_tb        rtl/actuation/sonar_freshness.sv sim/unit/sonar_freshness_tb.sv
run sonar_result_cdc       sonar_result_cdc_tb       rtl/integration/sonar_result_cdc.sv sim/unit/sonar_result_cdc_tb.sv
run ir_safety_latch        ir_safety_latch_tb        rtl/actuation/ir_safety_latch.sv sim/unit/ir_safety_latch_tb.sv
run motion_interface       motion_interface_tb       $MOTION sim/unit/motion_interface_tb.sv
if [ "$MODE" != "quick" ]; then
    run sonar_pipeline     sonar_pipeline_tb         $SONAR sim/integration/sonar_pipeline_tb.sv
fi
run motion_uart            motion_uart_tb            $MOTION $CHASSIS sim/integration/motion_uart_tb.sv

echo "== Top level (motor gate, UART, bypass switches) =="
run major_project_top      major_project_top_tb      $TOP_RTL sim/integration/major_project_top_tb.sv
run top_uart_safety        top_uart_safety_tb        $TOP_RTL vendor/chassis/tb/ugv02_model.sv sim/integration/top_uart_safety_tb.sv
run top_uart_latency       top_uart_latency_tb       $TOP_RTL vendor/chassis/tb/ugv02_model.sv sim/integration/top_uart_latency_tb.sv
run top_r9_bypass          top_r9_bypass_tb          $TOP_RTL sim/integration/top_r9_bypass_tb.sv
if grep -q "u_motion_supervisor" rtl/integration/major_project_top.sv; then
    run top_reverse_safety top_reverse_safety_tb     $TOP_RTL sim/integration/top_reverse_safety_tb.sv
else
    echo "SKIP: top_reverse_safety_tb (motion_supervisor not yet integrated in major_project_top)"
fi

echo
grep -h "Worst stop\|STOP LATENCY\|Worst observed\|protocol errors\|UGV02 model:" \
    "$B/motion_supervisor_uart.log" "$B/top_reverse_safety.log" "$B/top_uart_latency.log" "$B/motion_uart.log" 2>/dev/null
[ $status -eq 0 ] && echo "ALL PASS" || echo "SOME FAILED"
exit $status
