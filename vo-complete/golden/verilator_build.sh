#!/usr/bin/env bash
# Build a Verilator --binary bench, reusing one prebuilt copy of the Verilator
# runtime (verilated.o, verilated_threads.o, verilated_timing.o) instead of
# compiling it again for every bench: about 2 s per bench instead of 11.
#
#   golden/verilator_build.sh <mdir> <top> [verilator options and sources...]
#
# The runtime is built once into $VL_RUNTIME (default build/vl_runtime) from the
# first bench's own makefile, so it always has the flags this Verilator uses.
set -euo pipefail
M=$1; TOP=$2; shift 2
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RT="${VL_RUNTIME:-$ROOT/build/vl_runtime}"
mkdir -p "$M" "$RT"

verilator --cc --exe --main --timing --top-module "$TOP" --Mdir "$M" "$@"

OBJS="verilated.o verilated_threads.o verilated_timing.o"
lock() { if command -v flock >/dev/null 2>&1; then exec 8>"$RT/.lock"; flock 8; fi; }
unlock() { if command -v flock >/dev/null 2>&1; then flock -u 8; fi; }
lock
if [ ! -f "$RT/verilated_timing.o" ]; then
  make -s -C "$M" -f "V$TOP.mk" $OBJS > /dev/null
  cp -p $(for o in $OBJS; do echo "$M/$o"; done) "$RT/"
fi
unlock
for o in $OBJS; do cp "$RT/$o" "$M/$o"; done
touch $(for o in $OBJS; do echo "$M/$o"; done)
make -s -C "$M" -f "V$TOP.mk" > /dev/null
