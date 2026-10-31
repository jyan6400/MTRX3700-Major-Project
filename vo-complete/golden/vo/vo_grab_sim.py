#!/usr/bin/env python3
"""vo_grab_sim.py -- Group 26 (Wilson, M2): the VO bench board, simulated, photo to verdict.

    python golden/vo/vo_grab_sim.py              pose: 800 mm from the LED line, 120 mm right, turned 4 deg left

1. the three LEDs drawn on the lab photo from the pose (vo_scene.py), at the full 640 x 480;
2. golden/framegrab/sim/vo_grab_top_tb.sv: vo_grab_top pins to pins with the repo's camera model; two snapshots
   (KEY1 tracking green, the PC's request tracking red); the HEX digits at each;
3. memory -> dump.bin -> grab.py, plus truth.json (the pose);
4. vo_validate_grab.py: KEPT and GEOM exact, the tape check strict (the simulated pose is exact), and the HEX
   digits equal the tracked range.
Last line PASS / FAIL: vo_grab_sim.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
GOLDEN = HERE.parent
ROOT = GOLDEN.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(GOLDEN / "framegrab"))
sys.path.insert(0, str(GOLDEN / "framegrab" / "vendor" / "tools"))

import grab as vgrab                 # noqa: E402
import framegrab_sim as fs           # noqa: E402
import vo_scene as vs                # noqa: E402
import vo_validate_grab as vv        # noqa: E402

BUILD = ROOT / "build" / "vo" / "grab"
SOURCES = [s for s in fs.TOP_SOURCES if "vision_grab" not in s] + [
    "rtl/vo/vo_divider.sv", "rtl/vo/beacon_detect.sv", "rtl/vo/vo_mailbox_cdc.sv", "rtl/vo/beacon_geometry.sv",
    "rtl/vo/beacon_tracker.sv", "rtl/vo/vo_standoff.sv", "rtl/vo/beacon_vo.sv", "rtl/vo/vo_hex_display.sv",
    "rtl/image_processing/colour_run_filter.sv",
    "golden/framegrab/rtl/vo_grab_results.sv", "golden/framegrab/rtl/vo_grab_top.sv",
    "golden/framegrab/sim/vo_grab_top_tb.sv"]


def build():
    out = BUILD / "top"
    exe = out / "Vvo_grab_top_tb"
    srcs = [str(ROOT / s) for s in SOURCES]
    with fs.vg.build_lock():
        stamp = out / "sources.txt"
        if exe.exists() and stamp.exists() and stamp.read_text() == "\n".join(srcs) and \
                exe.stat().st_mtime > max(Path(s).stat().st_mtime for s in srcs + [str(ROOT / "rtl/vo/vo_calibration.svh")]):
            return exe
        out.mkdir(parents=True, exist_ok=True)
        cmd = ["bash", str(GOLDEN / "verilator_build.sh"), str(out), "vo_grab_top_tb", "-Wno-fatal", "-Wno-lint",
               "-Wno-style", "-O1", f"-I{ROOT / 'rtl/vo'}", "-DVGA_PPM_EVERY=100000", "-DVGA_MAX_PPM=0",
               "-DVGA_H_RES=640", "-DVGA_V_RES=480"] + srcs
        r = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
        if r.returncode != 0 or not exe.exists():
            raise RuntimeError("vo_grab_top_tb build failed:\n" + (r.stdout + r.stderr)[-3000:])
        stamp.write_text("\n".join(srcs))
    return exe


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--X", type=float, default=120.0)
    ap.add_argument("--Y", type=float, default=-800.0)
    ap.add_argument("--psi", type=float, default=4.0)
    a = ap.parse_args(argv)
    t0 = time.time()
    cal, g = vv.load_calibration()
    rig = vs.Rig(F_PX=g.F_Q4 / 16, CX=g.CX_Q4 / 16, CY=g.CY_Q4 / 16, H_LED=vs.Rig().H_CAM + g.DH_MM,
                 PITCH_DEG=float(__import__("numpy").degrees(__import__("numpy").arcsin(g.SIN_Q14 / 16384))))
    out = BUILD / "run"
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    discs, truth = vs.frame_discs(rig, a.X, a.Y, a.psi, (1, 1, 1))
    f640 = vs.render(vs.background(), discs)
    fs.write_scene(f640, out / "scene_in.hex")
    try:
        exe = build()
    except RuntimeError as e:
        print(f"FAIL: vo_grab_sim: {e}")
        return 1
    r = subprocess.run([str(exe), "+out=."], capture_output=True, text=True, cwd=out)
    (out / "sim.log").write_text(r.stdout + r.stderr)
    for line in (r.stdout + r.stderr).splitlines():
        if line.startswith(("[tb", "vo_grab_top", "PASS", "FAIL")):
            print("  " + line)
    if "PASS: vo_grab_top_tb" not in r.stdout:
        print(f"FAIL: vo_grab_sim: the bench did not pass ({out / 'sim.log'})")
        return 1
    fails = 0
    for k in (1, 2):
        gdir = out / f"grab_{k}"
        gdir.mkdir()
        fs.words_to_dump(out / f"snap_{k}.words", gdir / "dump.bin")
        with contextlib.redirect_stdout(io.StringIO()):
            vgrab.main(["--file", str(gdir / "dump.bin"), "--out", str(gdir)])
        json.dump({"pose": {"X": a.X, "Y": a.Y, "psi_deg": a.psi}}, open(gdir / "truth.json", "w"))
        rep, summ = vv.validate(gdir, (cal, g), tape_strict=True)
        fails += rep.fails
        hexs = (out / f"hex_{k}.txt").read_text().rstrip("\n")
        t = summ["decoded"]["track"] if summ else None
        want = (" rGb"[0] if False else "rGb"[summ["sel"]]) + " " + (f"{t['range_mm']:04d}" if t and t["track_valid"] else "----")
        ok = hexs == want
        print(f"{'PASS' if ok else 'FAIL'}: grab_{k}: HEX shows '{hexs}', the tracked range is '{want}'")
        fails += not ok
    dt = time.time() - t0
    print(("PASS" if fails == 0 else "FAIL") + f": vo_grab_sim (vo_grab_top pins to pins, 2 photos, KEPT and GEOM exact, "
          f"tape within {vv.TOL_MM} mm, HEX right; {dt:.0f} s)")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
