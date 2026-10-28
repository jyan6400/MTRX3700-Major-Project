#!/usr/bin/env bash
# Build (once) and run the RTL replay bench on one or more stimulus files.
#
#   golden/run_rtl_replay.sh [-s W H] <in.stim> <out.rec> [<in.stim> <out.rec> ...]
#
#   -s W H   use a bench built for W x H frames (small-frame fuzz); default 640 x 480
#
# Needs Verilator 5.050. Run from anywhere; paths are relative to the caller.
# Bench binaries are cached in build/golden/replay_<W>x<H>/ and rebuilt when
# any RTL or bench source is newer than the binary.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
W=640; H=480
if [ "${1:-}" = "-s" ]; then W=$2; H=$3; shift 3; fi
[ $# -ge 2 ] && [ $(( $# % 2 )) -eq 0 ] || { sed -n '2,10p' "$0"; exit 2; }

SRC=(
  "$ROOT/rtl/integration/camera_stream_adapter.sv"
  "$ROOT/rtl/image_processing/led_colour_mask.sv"
  "$ROOT/rtl/image_processing/red_mask.sv"
  "$ROOT/rtl/image_processing/colour_run_filter.sv"
  "$ROOT/rtl/perception/led_centroid.sv"
  "$ROOT/rtl/image_processing/tri_led_detect.sv"
  "$ROOT/golden/rtl/vision_tap.sv"
  "$ROOT/golden/sim/vision_replay_tb.sv"
)
# Optional override for deliberately-broken-module checks: REPLAY_SUBST="red_mask.sv=sim/broken/red_mask_broken.sv"
if [ -n "${REPLAY_SUBST:-}" ]; then
  from=${REPLAY_SUBST%%=*}; to=${REPLAY_SUBST#*=}
  for i in "${!SRC[@]}"; do [[ "${SRC[$i]}" == */"$from" ]] && SRC[$i]="$ROOT/$to"; done
fi
TAG="${REPLAY_TAG:-}"
B="$ROOT/build/golden/replay_${W}x${H}${TAG}"
BIN="$B/Vvision_replay_tb"

mkdir -p "$B"
# Several jobs of run_all_tests.sh may want the same bench at once: build under a lock.
if command -v flock >/dev/null 2>&1; then exec 9>"$B.lock"; flock 9; fi
need=0
[ -x "$BIN" ] || need=1
if [ $need -eq 0 ]; then for f in "${SRC[@]}"; do [ "$f" -nt "$BIN" ] && need=1; done; fi
if [ $need -eq 1 ]; then
  "$ROOT/golden/verilator_build.sh" "$B" vision_replay_tb -Wno-fatal -Wno-CASEINCOMPLETE -O2 \
      -GW=$W -GH=$H "${SRC[@]}" > "$B/compile.log" 2>&1 \
    || { echo "FAIL: vision_replay_tb compile (see $B/compile.log)"; tail -20 "$B/compile.log"; exit 1; }
fi
if command -v flock >/dev/null 2>&1; then flock -u 9; fi

while [ $# -gt 0 ]; do
  stim=$1; out=$2; shift 2
  mkdir -p "$(dirname "$out")"
  "$BIN" +STIM="$stim" +OUT="$out" ${REPLAY_TAP:+"+TAP=$REPLAY_TAP"} > "$out.log" 2>&1 \
    || { echo "FAIL: replay of $stim (see $out.log)"; tail -5 "$out.log"; exit 1; }
done
