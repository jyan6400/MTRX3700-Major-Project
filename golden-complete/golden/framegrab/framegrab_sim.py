#!/usr/bin/env python3
"""framegrab_sim.py -- Group 26 (Wilson, M2): the whole photo-and-validate flow, on a simulated board.

    python golden/framegrab/framegrab_sim.py                  the simulated board, then validate both grabs
    python golden/framegrab/framegrab_sim.py --scene photo_leds
    python golden/framegrab/framegrab_sim.py --selftest       the Python checks only (seconds, no Verilator)
    python golden/framegrab/framegrab_sim.py --broken         a broken led_colour_mask in the replay: must be CAUGHT

What it does, step by step, the same as on the bench with a real board:
  1. a scene for the repo's camera model: a lab photo with LEDs on it (golden/stimulus.py PHOTO_SCENES), at the
     full 640 x 480, so the simulated camera sends a real full-resolution frame, odd pixels and all;
  2. sim/vision_grab_top_tb.sv: vision_grab_top pins to pins (ov7670_config over the SCCB, the robot's chain,
     frame_snap), two snapshots: KEY1 with SW1 down, then the PC's request (what grab_jtag.tcl does) with SW1 up;
  3. each snapshot's memory -> dump.bin (the bytes grab_jtag.tcl writes) -> vendor/tools/grab.py (frame.png,
     frame.hex, scene.hex, results.json), exactly as on a real grab;
  4. validate_grab.py on each: integrity, KEPT exact, FULL approximate, the replay exact, and (simulation only)
     FULL exact against the frame the camera really sent.
Output in build/framegrab/sim/<scene>/grab_1 and grab_2. Last line: PASS / FAIL (or CAUGHT with --broken).
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

import numpy as np

HERE = Path(__file__).resolve().parent
GOLDEN = HERE.parent
ROOT = GOLDEN.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(GOLDEN))
sys.path.insert(0, str(HERE / "vendor" / "tools"))

import grab as vgrab              # noqa: E402  (FrameGrabJtag's tools/grab.py, unchanged)
import validate_grab as vg        # noqa: E402

BUILD = ROOT / "build" / "framegrab"

TOP_SOURCES = [
    "vendor/camera/rtl/camera_pkg.sv", "rtl/integration/camera_stream_adapter.sv",
    "rtl/image_processing/led_colour_mask.sv", "rtl/image_processing/red_mask.sv", "rtl/perception/led_centroid.sv",
    "vendor/camera/rtl/video_pll.sv", "vendor/camera/rtl/clock_divider.sv", "vendor/camera/rtl/ov7670_config.sv",
    "vendor/camera/rtl/ov7670_capture.sv", "vendor/camera/rtl/i2c_master.sv", "vendor/camera/rtl/frame_buffer.sv",
    "vendor/camera/rtl/camera_display.sv",
    "golden/framegrab/vendor/rtl/frame_snap.sv", "golden/framegrab/vendor/rtl/snapshot_ram.sv",
    "golden/framegrab/vendor/rtl/snapshot_probes.sv", "golden/framegrab/vendor/rtl/ov7670_hold.sv",
    "golden/framegrab/rtl/vision_grab_results.sv", "golden/framegrab/rtl/vision_grab_top.sv",
    "vendor/camera/tb/ov7670_model.sv", "vendor/camera/tb/vga_monitor_model.sv", "vendor/camera/tb/vga_sink_sim.sv",
    "golden/framegrab/sim/vision_grab_top_tb.sv",
]


def scene_frame(name):
    import stimulus
    return stimulus.photo_frame(stimulus.PHOTO_SCENES[name])


def write_scene(frame640, path):
    """A 656 x 488 scene that makes the camera model send frame640 exactly (grab.py's mapping, at full size)."""
    sc = vgrab.scene_from_frame(frame640, 0x37, 160, 11)
    with open(path, "w") as f:
        for y in range(sc.shape[0]):
            f.write("\n".join(f"{r:02x}{g:02x}{b:02x}" for r, g, b in sc[y].tolist()) + "\n")


def build_top():
    with vg.build_lock():
        return _build_top()


def _build_top():
    out = BUILD / "top"
    exe = out / "Vvision_grab_top_tb"
    srcs = [str(ROOT / s) for s in TOP_SOURCES]
    stamp = out / "sources.txt"
    newest = max(p.stat().st_mtime for p in map(Path, srcs))
    if exe.exists() and stamp.exists() and stamp.read_text() == "\n".join(srcs) and exe.stat().st_mtime > newest:
        return exe
    out.mkdir(parents=True, exist_ok=True)
    cmd = ["bash", str(GOLDEN / "verilator_build.sh"), str(out), "vision_grab_top_tb", "-Wno-fatal", "-Wno-lint",
           "-Wno-style", "-O1", "-DVGA_PPM_EVERY=100000", "-DVGA_MAX_PPM=0", "-DVGA_H_RES=640", "-DVGA_V_RES=480"] + srcs
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
    if r.returncode != 0 or not exe.exists():
        raise RuntimeError("vision_grab_top_tb build failed:\n" + (r.stdout + r.stderr)[-3000:])
    stamp.write_text("\n".join(srcs))
    return exe


def words_to_dump(words_file, dump):
    """snap_<k>.words (the memory GRAB, address order) -> dump.bin as grab_jtag.tcl writes it (high byte first)."""
    words = [int(s, 16) for s in Path(words_file).read_text().split()]
    Path(dump).write_bytes(b"".join(w.to_bytes(2, "big") for w in words))


def simulate_board(scene, out):
    """Steps 1 to 3. Returns the grab folders."""
    out.mkdir(parents=True, exist_ok=True)
    f640 = scene_frame(scene)
    write_scene(f640, out / "scene_in.hex")
    exe = build_top()
    t0 = time.time()
    r = subprocess.run([str(exe), "+out=."], capture_output=True, text=True, cwd=out)
    (out / "sim.log").write_text(r.stdout + r.stderr)
    for line in (r.stdout + r.stderr).splitlines():
        if line.startswith(("[tb", "vision_grab_top", "PASS", "FAIL")):
            print("  " + line)
    if "PASS: vision_grab_top_tb" not in r.stdout:
        raise RuntimeError(f"vision_grab_top_tb did not pass (see {out / 'sim.log'})")
    print(f"  simulated board: {time.time() - t0:.0f} s")
    # the frame the camera model sent (scene_from_frame at full size is exact): for the 'truth' check
    with open(out / "truth.hex", "w") as f:
        f.write("\n".join(f"{p:04x}" for p in f640.ravel().tolist()) + "\n")
    grabs = []
    for k in (1, 2):
        g = out / f"grab_{k}"
        if g.exists():
            shutil.rmtree(g)
        g.mkdir()
        words_to_dump(out / f"snap_{k}.words", g / "dump.bin")
        rc = vgrab.main(["--file", str(g / "dump.bin"), "--out", str(g)])
        if rc != 0:
            raise RuntimeError(f"grab.py rejected snapshot {k}")
        kept = vg.read_hex16(g / "frame.hex").reshape(240, 320)
        same = int((kept == f640[::2, ::2]).sum())
        print(f"{'PASS' if same == kept.size else 'FAIL'}: grab_{k}: {same} of {kept.size} photo pixels are the scene's "
              f"(the simulated camera sent the truth frame)")
        if same != kept.size:
            raise RuntimeError(f"snapshot {k}: the photo is not the scene the camera model was given")
        json.dump({"frame": "../truth.hex", "scene": scene}, open(g / "truth.json", "w"))
        grabs.append(g)
    return grabs


# ---------------------------------------------------------------- the Python-only self-test

def model_grab(g, frame, f640, sw1, tamper=None):
    """A grab folder made in Python: the photo `frame` (320 x 240) and the words the golden model gives for it
    (FULL from f640, the full frame), laid out as frame_snap lays out its memory, then decoded by grab.py."""
    import binascii
    import model
    r_full = model.frame_result(f640, bool(sw1))
    r_kept, su, sv, _ = vg.model_kept(frame, sw1)
    words = [(2 << 16) | 1,
             (0x26 << 24) | (1 << 16) | (1 << 10) | (sw1 << 9) | (1 << 8) | (1 << 3) | (1 << 2)]
    words += vg.encode_result(r_full) + vg.encode_result(r_kept)
    words += [su, sv, 320 * 240, (5 << 16) | 0, 0, 799680]
    if tamper:
        tamper(words)
    snap = [0x4752, 0x4142, 1, 320, 240, 1, 18]
    for w in words:
        snap += [w >> 16, w & 0xFFFF]
    snap += frame.ravel().tolist()
    data = b"".join(int(w).to_bytes(2, "big") for w in snap)
    data += binascii.crc_hqx(data, 0xFFFF).to_bytes(2, "big")
    g = Path(g)
    if g.exists():
        shutil.rmtree(g)
    g.mkdir(parents=True)
    (g / "dump.bin").write_bytes(data)
    with contextlib.redirect_stdout(io.StringIO()):
        assert vgrab.main(["--file", str(g / "dump.bin"), "--out", str(g)]) == 0
    return g


def near_threshold_photo():
    """photo_leds' photo with a 64 x 48 patch of pixels within +-2 of every colour threshold (top left): a grab
    whose red_mask decisions sit on every edge, so a mask bug cannot hide."""
    import stimulus
    f640 = scene_frame("photo_leds")
    f640[:96, :128] = stimulus.near_threshold_frame()[:96, :128]
    return f640


def selftest():
    """validate_grab on grabs made in Python from the model: a right one must pass, wrong ones must fail."""
    import model
    fails = 0
    tmp = BUILD / "selftest"
    if tmp.exists():
        shutil.rmtree(tmp)
    tmp.mkdir(parents=True)
    f640 = scene_frame("photo_leds")
    frame = f640[::2, ::2].copy()

    def make(name, sw1, tamper=None):
        return model_grab(tmp / name, frame, f640, sw1, tamper)

    cases = [
        ("right_v2", 0, None, True),
        ("right_v1", 1, None, True),
        ("kept_x_off_by_one", 0, lambda w: w.__setitem__(7, w[7] + (1 << 16)), False),
        ("kept_count", 1, lambda w: w.__setitem__(8, w[8] + 1), False),
        ("kept_sum_u", 0, lambda w: w.__setitem__(12, w[12] ^ 4), False),
        ("stale_words", 0, lambda w: w.__setitem__(0, (3 << 16) | 1), False),
        ("rejected_frame", 0, lambda w: w.__setitem__(1, w[1] & ~(1 << 8)), False),
        ("short_frame", 0, lambda w: w.__setitem__(14, 76799), False),
        ("not_our_words", 0, lambda w: w.__setitem__(1, 0), False),
    ]
    for name, sw1, tamper, should_pass in cases:
        g = make(name, sw1, tamper)
        rep = vg.validate(g, replay=False, plot=False, quiet_name=f"selftest {name}", echo=False)
        why = "; ".join(f"{c}: {t}" for w, c, t in rep.lines if w == "FAIL")[:160]
        ok = (rep.fails == 0) == should_pass
        print(f"{'PASS' if ok else 'FAIL'}: selftest {name}: validate_grab {'passed' if rep.fails == 0 else 'failed'} "
              f"({rep.fails} failed checks{': ' + why if why else ''}), expected {'pass' if should_pass else 'fail'}")
        fails += not ok
    # the words round-trip
    r = dict(valid=1, x=639, y=479, count=123456, conf=200, flags=0b1010, bx0=1, bx1=1023, by0=0, by1=511)
    back = vg._result(*vg.encode_result(r))
    rt = all(back[k] == r[k] for k in r)
    print(f"{'PASS' if rt else 'FAIL'}: selftest words round-trip")
    fails += not rt
    return fails


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default="photo_two_red", help="a golden/stimulus.py PHOTO_SCENES name (default photo_two_red)")
    ap.add_argument("--selftest", action="store_true", help="only the Python checks")
    ap.add_argument("--broken", action="store_true", help="replay with golden/broken/led_colour_mask_broken.sv: must be CAUGHT")
    a = ap.parse_args(argv)
    t0 = time.time()

    if a.selftest:
        n = selftest()
        print(("PASS" if n == 0 else "FAIL") + f": framegrab_selftest ({n} failed, {time.time() - t0:.0f} s)")
        return 0 if n == 0 else 1

    out = BUILD / "sim" / a.scene
    try:
        if a.broken:
            # a grab of a photo with near-threshold colours, its words from the golden model (which the simulated
            # board shows the RTL matches): the real replay must agree with them, a broken mask must not
            f640 = near_threshold_photo()
            g = model_grab(BUILD / "broken" / "near_threshold", f640[::2, ::2].copy(), f640, 0)
            rep = vg.validate(g, replay=True, exe=vg.build_replay(), plot=False, quiet_name="near_threshold (real RTL)")
            if rep.fails:
                print("FAIL: the replay of the near-threshold grab failed with the real RTL")
                return 1
            exe = vg.build_replay({"rtl/image_processing/led_colour_mask.sv": "golden/broken/led_colour_mask_broken.sv"}, "_broken")
            rep = vg.validate(g, replay=True, exe=exe, plot=False, quiet_name="near_threshold (broken mask in the replay)")
            if rep.fails:
                why = next(t for w, c, t in rep.lines if w == "FAIL" and "replay" in c)
                print(f"CAUGHT: led_colour_mask_broken: the grab replay disagrees with the board's words: {why[:150]}")
                return 0
            print("FAIL: led_colour_mask_broken was NOT caught by the grab replay")
            return 1
        print(f"== simulated board, scene {a.scene}")
        grabs = simulate_board(a.scene, out)
        exe = vg.build_replay()
        fails = warns = 0
        for g in grabs:
            print(f"== validate {g}")
            rep = vg.validate(g, replay=True, exe=exe)
            fails += rep.fails
            warns += rep.warns
    except RuntimeError as e:
        print(f"FAIL: framegrab_sim: {e}")
        return 1
    dt = time.time() - t0
    if fails == 0:
        print(f"PASS: framegrab_sim ({a.scene}: 2 grabs from the simulated board, golden model exact on KEPT, "
              f"replay exact, {warns} warning(s), {dt:.0f} s)")
        return 0
    print(f"FAIL: framegrab_sim ({fails} failed checks)")
    return 1


if __name__ == "__main__":
    sys.exit(main())
