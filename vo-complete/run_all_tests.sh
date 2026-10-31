#!/usr/bin/env bash
# MTRX3700 Group 26 - complete regression (R6), Verilator 5.050.
#
#   ./run_all_tests.sh           every module bench, the integration benches, the
#                                system-level simulation, the golden model (R7), the
#                                JTAG photo flow on a simulated board (golden/framegrab),
#                                the beacon visual odometry (golden/vo)
#                                and the deliberately broken modules (must print CAUGHT)
#   ./run_all_tests.sh --full    also the long benches: top_led_photo_tb (whole robot,
#                                lab photo, ~3 min), the full golden fuzz set, the full VO
#                                check and the VO bench build pins to pins (vo_grab_sim)
#   ./run_all_tests.sh -j N      parallel jobs (default: number of CPU cores)
#
# Prints one PASS / FAIL / CAUGHT line per bench and a summary; logs are in
# build/run_all/<bench>.log. Exit status 0 only when every bench passes and every
# broken module is caught. Run from anywhere; it works on a clean copy.
#
# Needs: verilator 5.050 on PATH; python3 with numpy, pandas and Pillow for the
# golden-model and lab-photo benches (pip install numpy pandas pillow).
set -u
cd "$(dirname "$0")"
ROOT=$(pwd)
B=build/run_all
FULL=0
JOBS=$( (nproc || sysctl -n hw.ncpu || echo 2) 2>/dev/null | head -1)
while [ $# -gt 0 ]; do
  case $1 in
    --full) FULL=1 ;;
    -j) JOBS=$2; shift ;;
    -j*) JOBS=${1#-j} ;;
    *) sed -n '2,16p' "$0"; exit 2 ;;
  esac
  shift
done
mkdir -p "$B"
ulimit -s unlimited 2>/dev/null || true
T0=$(date +%s)

if ! command -v verilator >/dev/null; then echo "FAIL: verilator not on PATH"; exit 1; fi
VV=$(verilator --version | awk '{print $2}')
echo "MTRX3700 Group 26 regression: Verilator $VV, $JOBS parallel jobs$([ $FULL = 1 ] && echo ', full')"
[ "$VV" = "5.050" ] || echo "NOTE: the course requires Verilator 5.050 (this is $VV)"
PY=python3; command -v $PY >/dev/null || PY=python
HAVE_PY=0
$PY -c "import numpy, pandas, PIL" 2>/dev/null && HAVE_PY=1
[ $HAVE_PY = 1 ] || echo "NOTE: python3 with numpy, pandas and Pillow not found: golden-model and photo benches will FAIL"

# ------------------------------------------------------------------ sources
RTL="rtl/integration/major_project_top.sv rtl/integration/camera_stream_adapter.sv
 rtl/integration/vision_result_cdc.sv rtl/integration/sonar_result_cdc.sv
 rtl/image_processing/led_colour_mask.sv rtl/image_processing/red_mask.sv
 rtl/image_processing/colour_run_filter.sv rtl/image_processing/tri_led_detect.sv
 rtl/perception/led_centroid.sv
 rtl/navigation/camera_frame_watchdog.sv rtl/navigation/steering_controller.sv rtl/navigation/docking_fsm.sv
 rtl/actuation/sonar_freshness.sv rtl/actuation/sonar_filter.sv rtl/actuation/ir_safety_latch.sv
 rtl/actuation/motion_interface.sv
 vendor/camera/rtl/video_pll.sv vendor/camera/rtl/clock_divider.sv vendor/camera/rtl/ov7670_config.sv
 vendor/camera/rtl/ov7670_capture.sv vendor/camera/rtl/i2c_master.sv
 vendor/ultrasonic/rtl/sonar_pll.sv vendor/ultrasonic/rtl/sonar_range.sv
 vendor/ir/rtl/nec_decoder.sv vendor/chassis/rtl/chassis_drive.sv vendor/chassis/rtl/uart_tx.sv"
RTL=$(echo $RTL)            # one line: the job lines below must not contain newlines
CAM=vendor/camera/tb/ov7670_model.sv
UGV=vendor/chassis/tb/ugv02_model.sv
SONAR=vendor/ultrasonic/hcsr04_model.sv
IRM=vendor/ir/tb/ir_remote_model.sv
# The adapter bench with the deliberately broken adapter swapped in.
RTL_BROKEN_ADAPTER=$(echo $RTL | sed 's#rtl/integration/camera_stream_adapter.sv#sim/broken/camera_stream_adapter_broken.sv#')

# ------------------------------------------------------------------ jobs
# One line per job:  kind|name|top|files (kind: sv = Verilator bench, sv_broken =
# must FAIL -> CAUGHT, py = command whose output has the PASS/CAUGHT line)
JOBLIST=$B/jobs.txt
: > $JOBLIST
sv()  { echo "sv|$1|$1|$2" >> $JOBLIST; }
for t in camera_frame_watchdog_tb camera_stream_adapter_tb docking_fsm_tb ir_safety_latch_tb \
         led_centroid_tb led_colour_mask_tb motion_interface_tb red_mask_tb sonar_filter_tb \
         sonar_freshness_tb sonar_result_cdc_tb steering_controller_tb tri_led_detect_tb \
         vision_result_cdc_tb; do
  sv $t "$RTL sim/unit/$t.sv"
done
sv red_mask_v1_equiv_tb "$RTL sim/unit/red_mask_v1_ref.sv sim/unit/red_mask_v1_equiv_tb.sv"
for t in major_project_top_tb red_vision_pipeline_tb top_r9_bypass_tb; do
  sv $t "$RTL sim/integration/$t.sv"
done
sv camera_capture_vision_tb "$RTL $CAM sim/integration/camera_capture_vision_tb.sv"
sv motion_uart_tb           "$RTL $UGV sim/integration/motion_uart_tb.sv"
sv top_uart_safety_tb       "$RTL $UGV sim/integration/top_uart_safety_tb.sv"
sv top_uart_latency_tb      "$RTL $UGV sim/integration/top_uart_latency_tb.sv"
sv sonar_pipeline_tb        "$RTL $SONAR sim/integration/sonar_pipeline_tb.sv"
sv sonar_filter_pipeline_tb "$RTL $SONAR sim/integration/sonar_filter_pipeline_tb.sv"
sv tri_led_photo_tb         "$RTL sim/integration/tri_led_photo_tb.sv"
echo "sv_broken|camera_stream_adapter_broken|camera_stream_adapter_tb|$RTL_BROKEN_ADAPTER sim/unit/camera_stream_adapter_tb.sv" >> $JOBLIST
# Golden model (R7) and the system-level simulation from the camera pins
echo "py|golden_model|golden model|$PY golden/check_golden.py $([ $FULL = 1 ] || echo --quick)" >> $JOBLIST
echo "py|golden_broken|led_colour_mask_broken|$PY golden/check_golden.py --broken" >> $JOBLIST
echo "py|system_capture|top_capture_tb|$PY golden/check_golden.py --system" >> $JOBLIST
# The JTAG photo (golden/framegrab): the Python checks, the whole flow on a simulated board (vision_grab_top pins
# to pins, two grabs, grab.py, validate_grab.py with the replay), and a broken mask the grab replay must catch
echo "py|framegrab_selftest|framegrab_selftest|$PY golden/framegrab/framegrab_sim.py --selftest" >> $JOBLIST
echo "py|framegrab_sim|framegrab_sim|$PY golden/framegrab/framegrab_sim.py" >> $JOBLIST
echo "py|framegrab_broken|led_colour_mask_broken (grab replay)|$PY golden/framegrab/framegrab_sim.py --broken" >> $JOBLIST
# Beacon visual odometry (golden/vo): RTL == golden model (math fuzz, drive scenes through the adapter and
# both clocks), simulated accuracy against ground truth, the calibration fit, the bench build pins to pins
# with two JTAG photos, and a broken geometry module that must be CAUGHT
echo "py|vo_check|vo_check|$PY golden/vo/vo_check.py $([ $FULL = 1 ] || echo --quick)" >> $JOBLIST
echo "py|vo_calibrate|vo_calibrate selftest|$PY golden/vo/vo_calibrate.py --selftest" >> $JOBLIST
[ $FULL = 1 ] && echo "py|vo_grab_sim|vo_grab_sim|$PY golden/vo/vo_grab_sim.py" >> $JOBLIST   # about a minute
echo "py|vo_broken|beacon_geometry_broken|$PY golden/vo/vo_check.py --broken" >> $JOBLIST
if [ $FULL = 1 ]; then
  sv top_led_photo_tb "$RTL $CAM $SONAR $IRM $UGV sim/integration/top_led_photo_tb.sv"
fi

# ------------------------------------------------------------------ runner
run_job() {   # one job line -> build/run_all/<name>.result = "<word>|<seconds>|<message>"
  IFS='|' read -r kind name top files <<< "$1"
  local t0=$(date +%s) log=$B/$name.log out word msg
  if [ "$kind" = py ]; then
    $files > "$log" 2>&1; local rc=$?
    if   grep -q "^CAUGHT: " "$log"; then word=CAUGHT; msg=$(grep -m1 "^CAUGHT: " "$log" | cut -c9-)
    elif [ $rc -eq 0 ] && grep -q "^PASS: $top" "$log"; then word=PASS; msg=$(grep -m1 "^PASS: $top" "$log" | cut -c7-)
    else word=FAIL; msg="$name (see $log): $(grep -m1 -E '^(FAIL|Traceback)' "$log" | cut -c1-160)"; fi
  else
    mkdir -p $B/$name
    if ! bash golden/verilator_build.sh "$B/$name" "$top" -Wno-fatal -Wno-CASEINCOMPLETE -O1 \
          $files > "$B/$name.compile.log" 2>&1; then
      word=FAIL; msg="$name compile (see $B/$name.compile.log)"
    else
      ( cd "$ROOT" && "./$B/$name/V$top" ) > "$log" 2>&1; local rc=$?
      local passed=0
      [ $rc -eq 0 ] && grep -q "^PASS: $top" "$log" && passed=1
      if [ "$kind" = sv_broken ]; then
        if [ $passed = 0 ]; then word=CAUGHT; msg="$name: $(grep -m1 -oE 'FAIL[^"]*' "$log" | head -c 140)"
        else word=FAIL; msg="$name was NOT caught by $top"; fi
      elif [ $passed = 1 ]; then word=PASS; msg=$(grep -m1 "^PASS: $top" "$log" | cut -c7-)
      else word=FAIL; msg="$name (exit $rc, see $log): $(grep -m1 -E 'FAIL|Error' "$log" | cut -c1-140)"; fi
    fi
  fi
  echo "$word|$(( $(date +%s) - t0 ))|$msg" > "$B/$name.result"
  printf '%-6s %s  [%ss]\n' "$word:" "$msg" "$(( $(date +%s) - t0 ))"
}
export -f run_job
export B ROOT
export VL_RUNTIME=$ROOT/$B/vl_runtime      # one Verilator runtime build shared by every bench

# lab-photo frames for tri_led_photo_tb (generated, not committed)
if [ ! -f sim/data/photo_leds.hex ] && [ $HAVE_PY = 1 ]; then
  $PY sim/tools/make_led_frames.py > $B/make_led_frames.log 2>&1 || echo "NOTE: frame generation failed (see $B/make_led_frames.log)"
fi

# longest jobs (python, photo, camera-model benches) first so the critical path starts early
LONG='^(py\||sv\|(top_led_photo_tb|sonar_filter_pipeline_tb|sonar_pipeline_tb|camera_capture_vision_tb|motion_uart_tb|tri_led_photo_tb)\|)'
{ grep -E "$LONG" $JOBLIST; grep -vE "$LONG" $JOBLIST; } > $B/jobs.sorted
[ "$(wc -l < $B/jobs.sorted)" = "$(wc -l < $JOBLIST)" ] || { echo "FAIL: run_all_tests job list"; exit 1; }
tr '\n' '\0' < $B/jobs.sorted | xargs -0 -n 1 -P "$JOBS" bash -c 'run_job "$1"' _

# ------------------------------------------------------------------ summary
pass=0; fail=0; caught=0
while IFS='|' read -r kind name rest; do
  w=$(cut -d'|' -f1 "$B/$name.result" 2>/dev/null || echo FAIL)
  case $w in PASS) pass=$((pass+1));; CAUGHT) caught=$((caught+1));; *) fail=$((fail+1)); echo "  failed: $name";; esac
done < $JOBLIST
expected_caught=$(grep -cE '^(sv_broken|py\|golden_broken|py\|framegrab_broken|py\|vo_broken)' $JOBLIST)
T=$(( $(date +%s) - T0 ))
echo "------------------------------------------------------------"
echo "$pass passed, $fail failed, $caught of $expected_caught broken modules caught, ${T}s"
if [ $fail -eq 0 ] && [ $caught -eq $expected_caught ]; then
  echo "PASS: run_all_tests ($((pass + caught)) benches, ${T}s)"
  exit 0
fi
echo "FAIL: run_all_tests"
exit 1
