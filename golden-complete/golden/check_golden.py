"""Golden-model regression (R7), headless. Prints PASS/FAIL/CAUGHT lines like the
other benches; exit status 0 only if everything passed.

    python golden/check_golden.py              all checks
    python golden/check_golden.py --quick      fewer fuzz seeds and photo scenes (run_all_tests.sh)
    python golden/check_golden.py --no-rtl     model-only checks (no Verilator, e.g. on Windows)
    python golden/check_golden.py --broken     replay with golden/broken/led_colour_mask_broken.sv:
                                               must print CAUGHT
    python golden/check_golden.py --system     system-level: simulate the whole robot from the
                                               camera pins with vision_tap (top_capture_tb),
                                               capture it, and require board == model == RTL

Checks
  params      the model's parameters equal the RTL defaults, and major_project_top
              instantiates the chain without overriding them
  v1_equiv    golden red_mask == independent V1 red_mask on all 65,536 codes
  scenarios   golden model == RTL replay, every record and field, on generated
              streams: every centroid gate, all colour codes, lab-photo frames
              at camera timing, adapter faults, random fuzz, both SW1 modes
  parsers     SignalTap export readers agree (taplog / VCD vector / VCD bits / CSV)
  capture_faults  vision_tap + capture.py on a stream with truncated and
              coordinate-error frames: every close classified, replay exact
  captures    board vs golden model vs RTL for every golden/data/captures/*/
"""
from __future__ import annotations

import argparse
import csv
import gzip
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

import numpy as np          # noqa: E402

import model                # noqa: E402

RESULTS = []


def report(name, ok, detail="", word=None):
    word = word or ("PASS" if ok else "FAIL")
    RESULTS.append((name, ok))
    print(f"{word}: golden {name}" + (f" ({detail})" if detail else ""), flush=True)


# ---------------------------------------------------------------- params

def _sv_params(path):
    out = {}
    txt = open(path).read()
    for m in re.finditer(r"parameter\s+(?:logic\s*\[[^\]]*\]\s*|int\s+(?:unsigned\s+)?|bit\s+)?(\w+)\s*=\s*(?:\d+'[dD])?(\d+)", txt):
        out[m.group(1)] = int(m.group(2))
    return out


def check_params():
    problems = []
    lcm = _sv_params(ROOT / "rtl/image_processing/led_colour_mask.sv")
    rm = _sv_params(ROOT / "rtl/image_processing/red_mask.sv")
    cen = _sv_params(ROOT / "rtl/perception/led_centroid.sv")
    flt = _sv_params(ROOT / "rtl/image_processing/colour_run_filter.sv")
    mp, cp = model.MaskParams(), model.CentroidParams()
    for k, v in mp.__dict__.items():
        src = rm if k == "TARGET_COLOUR" else lcm
        if k in src and src[k] != v:
            problems.append(f"{k}: RTL {src[k]} model {v}")
        if k != "TARGET_COLOUR" and k in rm and rm[k] != v:
            problems.append(f"red_mask {k}: RTL {rm[k]} model {v}")
    for k, v in cp.__dict__.items():
        if cen.get(k) != v:
            problems.append(f"led_centroid {k}: RTL {cen.get(k)} model {v}")
    if flt.get("RUN_LEN") != model.ChainParams().RUN_LEN:
        problems.append(f"colour_run_filter RUN_LEN: RTL {flt.get('RUN_LEN')}")
    top = open(ROOT / "rtl/integration/major_project_top.sv").read()
    if not re.search(r"\bred_mask\s+u_red_mask\s*\(", top):
        problems.append("major_project_top overrides red_mask parameters (update model.MaskParams)")
    if not re.search(r"\bled_centroid\s+u_led_centroid\s*\(", top):
        problems.append("major_project_top overrides led_centroid parameters (update model.CentroidParams)")
    m = re.search(r"camera_stream_adapter\s*#\s*\((.*?)\)\s*u_camera_adapter", top, re.S)
    if m and not (re.search(r"\.W\(640\)", m.group(1)) and re.search(r"\.H\(480\)", m.group(1))):
        problems.append("camera_stream_adapter W/H in the top are not 640x480")
    report("params", not problems, "; ".join(problems) or "model parameters == RTL defaults, top uses defaults")


def check_v1():
    codes = np.arange(65536)
    a, b = model.red_mask(codes), model.red_mask_v1(codes)
    report("v1_equiv", bool(np.all(a == b)), f"{int(a.sum())} red codes, {int(np.sum(a != b))} differ from V1")


# ---------------------------------------------------------------- parsers

def check_parsers():
    import capture
    cdir = HERE / "data/captures/sim_photo"
    if not (cdir / "capture.taplog").exists():
        report("parsers", True, "no sim capture to test with", "SKIP")
        return
    words = capture.read_export(cdir / "capture.taplog")
    ok, detail = True, []
    tmp = ROOT / "build/golden/parsers"
    tmp.mkdir(parents=True, exist_ok=True)
    for name in ("export.vcd.gz", "export_bits.vcd.gz"):
        if (cdir / name).exists():
            out = tmp / name[:-3]
            out.write_bytes(gzip.open(cdir / name).read())
            w = capture.read_export(out)
            ok &= (w == words)
            detail.append(f"{name[:-3]} {'==' if w == words else '!='}")
    # CSV, two plausible SignalTap layouts: one hex bus column, and one column per bit
    p1 = tmp / "bus.csv"
    with open(p1, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["Signal Tap export (synthetic)"]); wr.writerow([])
        wr.writerow(["time unit: samples", "u_vision_tap|tap_store", "u_vision_tap|tap_word[55..0]"])
        for i, w in enumerate(words):
            wr.writerow([i, 1, f"{w:014X}"])
    p2 = tmp / "bits.csv"
    with open(p2, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["sample", "tap_store"] + [f"tap_word[{b}]" for b in range(56)])
        for i, w in enumerate(words):
            wr.writerow([i, 1] + [(w >> b) & 1 for b in range(56)])
            if i % 7 == 0:
                wr.writerow([i, 0] + [0] * 56)                     # not stored: dropped
    for p in (p1, p2):
        w = capture.read_export(p)
        ok &= (w == words)
        detail.append(f"{p.name} {'==' if w == words else '!='}")
    report("parsers", ok, f"{len(words)} samples; taplog " + ", ".join(detail))


def check_capture_faults():
    """The capture path on a stream with adapter faults (which the camera model
    never makes): vision_tap inside the RTL replay bench records it, capture.py
    must classify every close, and the replay must reproduce the 'board'."""
    import os
    import capture
    import replay
    import stimulus
    import vlog
    bg = stimulus.blank()
    one = stimulus.draw_disc(bg, 300, 200, 12, core=4)
    two = stimulus.draw_disc(stimulus.draw_disc(bg, 120, 100, 10), 500, 380, 10)
    ev = stimulus.frames_to_events([one, one, one, two, two, one], [0, 0, 0, 0, 1, 0], stimulus.CAMERA)
    sl = stimulus.stream_frame_slice(ev)
    keep = np.ones(len(ev), bool)
    keep[sl[1][0] + 1 + 150000: sl[1][1]] = False        # frame 1 truncated
    keep[sl[2][0] + 1 + 640 * 100 + 7] = False           # frame 2 loses a pixel: coordinate error
    ev = ev[keep]
    work = ROOT / "build/golden/capture_faults"
    vlog.write_stim(work / "faults.stim", ev)
    os.environ["REPLAY_TAP"] = str(work / "faults.taplog")
    try:
        replay.run_rtl([(work / "faults.stim", work / "faults.rec")])
    finally:
        del os.environ["REPLAY_TAP"]
    cap = capture.load(work / "faults.taplog")
    closes = [f.close for f in cap.frames]
    cr = replay.replay_capture(cap, tag="cap_faults")
    ok = closes == ["good", "truncated", "coordinate", "good", "good", "good"] and cr.ok
    failed = [c for c, o, _ in cr.checks if not o]
    report("capture_faults", ok, f"closes {closes}; {len(cr.frames)} frames rebuilt, {len(cr.checks)} checks"
           + (f"; failed: {'; '.join(failed)}" if failed else ""))


# ---------------------------------------------------------------- main

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--no-rtl", action="store_true")
    ap.add_argument("--broken", action="store_true")
    ap.add_argument("--system", action="store_true")
    a = ap.parse_args(argv)
    t0 = time.time()
    import replay

    if a.system:
        import capture_sim
        out = ROOT / "build/golden/system_capture"
        try:
            summ, log = capture_sim.run(out)
        except Exception as e:                       # noqa: BLE001
            report("system capture", False, str(e)[-400:])
            return 1
        cr = replay.replay_capture(out, tag="cap_system")
        failed = [c for c, ok, _ in cr.checks if not ok]
        print(f"{'PASS' if cr.ok else 'FAIL'}: top_capture_tb system replay (real major_project_top from the camera pins, "
              f"{summ['samples']} SignalTap samples, {len(cr.frames)} frames, board == model == RTL on "
              f"{len(cr.checks)} checks{'; failed: ' + '; '.join(failed) if failed else ''}, {time.time() - t0:.0f} s)")
        return 0 if cr.ok else 1

    if a.broken:
        try:
            res = replay.run_scenarios(subst="led_colour_mask.sv=golden/broken/led_colour_mask_broken.sv",
                                       tag="_broken", log=None, minimal=True)
        except RuntimeError as e:
            report("broken led_colour_mask", False, f"replay did not run: {str(e)[-300:]}")
            return 1
        bad = [r for r in res if not r.report.ok]
        if bad:
            r = bad[0]
            P = r.report.types["P"]
            ex = P.first[0] if P.first else None
            detail = f"{len(bad)} of {len(res)} scenarios differ from the golden model"
            if ex:
                rec = r.rtl_rec["P"][ex[0]]
                detail += f"; first in {r.name}: pixel ({rec[1]},{rec[2]}) rgb {rec[3]:04x} {ex[1]} model {ex[2]} RTL {ex[3]}"
            print(f"CAUGHT: led_colour_mask_broken (5-bit green vs 6-bit ceiling): {detail}")
            return 0
        print("FAIL: broken led_colour_mask was NOT caught by the golden model")
        return 1

    check_params()
    check_v1()
    rtl = not a.no_rtl and replay.have_rtl()
    if not rtl:
        print("NOTE: golden: Verilator not found or --no-rtl: model-vs-RTL checks skipped")
    try:
        res = replay.run_scenarios(quick=a.quick, rtl=rtl, log=None)
    except ImportError as e:            # Pillow missing: no photo frames
        report("scenarios", False, f"needs numpy, pandas and Pillow: {e}")
        res = []
    if rtl and res:
        bad = [r for r in res if not r.report.ok]
        n = {k: sum(r.report.types[k].n_a for r in res) for k in "PRT"}
        report("scenarios", not bad,
               f"{len(res)} streams, {n['P']} pixels, {n['R']} centroid + {n['T']} tri-colour results"
               + (f"; MISMATCH in {', '.join(r.name for r in bad[:5])}" if bad else ", all records equal"))
        for r in bad[:2]:
            print(r.report.summary())
    check_parsers()
    if rtl:
        check_capture_faults()
    for cdir in sorted((HERE / "data/captures").glob("*/")):
        if not (cdir / "capture.taplog").exists():
            continue
        cr = replay.replay_capture(cdir, rtl=rtl, tag=f"cap_{cdir.name}")
        failed = [c for c, ok, _ in cr.checks if not ok]
        report(f"capture {cdir.name}", cr.ok,
               f"{len(cr.frames)} frames, {len(cr.checks)} checks" + (f"; failed: {'; '.join(failed)}" if failed else ""))
    ok = all(o for _, o in RESULTS)
    print(f"{'PASS' if ok else 'FAIL'}: golden model ({sum(o for _, o in RESULTS)}/{len(RESULTS)} checks, {time.time() - t0:.0f} s)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
