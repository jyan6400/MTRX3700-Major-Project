#!/bin/sh
# Compile the Group 26 JTAG photo project from a Linux shell, with Quartus Prime Lite 18.1's quartus/bin on the PATH:
#     cd golden/framegrab/quartus && sh build.sh
# On Windows: open vision_grab.qpf in Quartus and Start Compilation, or in a Command Prompt in this folder
#     C:\intelFPGA_lite\18.1\quartus\bin64\quartus_sh.exe --flow compile vision_grab
# Quartus generates the VGA system (vga_sink.qsys) itself. At the end this prints the resources and the worst slack.
set -e
command -v quartus_sh > /dev/null 2>&1 || { echo "quartus_sh is not on the PATH (Quartus 18.1's quartus/bin)"; exit 1; }
[ -f vga_sink.qsys ] || qsys-script --script=make_vga_sink_qsys.tcl
quartus_sh --flow compile vision_grab
echo
echo "== Resources (output_files/vision_grab.fit.summary)"
grep -E "Logic utilization|Total registers|Total block memory bits|Total RAM Blocks|Total DSP Blocks" output_files/vision_grab.fit.summary || true
echo "== Timing"
worst=$(awk '/^Slack/ { print $3 }' output_files/vision_grab.sta.summary | sort -g | head -n 1)
case "$worst" in
  "") echo "no timing results: is vision_grab.sdc in the project?" ;;
  -*) echo "TIMING FAILED: worst slack $worst ns. Do not trust this .sof (output_files/vision_grab.sta.rpt)" ;;
  *)  echo "timing met: worst slack $worst ns" ;;
esac
[ -f output_files/vision_grab.sof ] && echo "output_files/vision_grab.sof: program it, then python golden/framegrab/tools/take_photo.py"
