#!/usr/bin/env python3
"""vo_validate_grab.py -- Group 26 (Wilson, M2): a JTAG photo from vo_grab_top, checked against the golden model
and, when the pose was tape-measured, against the tape.

    python golden/vo/vo_validate_grab.py GRAB_DIR [GRAB_DIR ...]

GRAB_DIR holds what grab_jtag.tcl + grab.py wrote (frame.hex, results.json, dump.bin; validate_grab.py's
ensure_decoded rebuilds them from dump.bin alone) and, optionally, truth.json (vo_take_photo.py writes it):
    {"pose": {"X": mm, "Y": mm, "psi_deg": deg}}   the camera's position: Y = minus the perpendicular distance from
                                                   the LED line, X = sideways from the green LED (right +), psi =
                                                   turned left of square-on (0 when squared up)

THE CHECKS
  integrity  layout 2 words (status 0x26 0x02), the kept frame's (live = result + 1), the adapter passed it,
             both detections reported.
  KEPT       EXACT: the board's second beacon_detect saw only the photo's pixels, so vo_model.detect_frame on
             frame.hex (320 x 240 parameters) gives its valid, centroid, count and flags for each colour.
  GEOM       EXACT: vo_model.geometry on the board's own FULL centroids (words 2..7), with the calibration in
             golden/vo/calibration.json (the one rtl/vo/vo_calibration.svh was built with), gives the board's
             forward and lateral of each LED, the pair check and the heading.
  FULL       APPROXIMATE (WARN): FULL centroid about twice KEPT's, count about four times.
  track      the tracker's range agrees with this frame's geometry (WARN otherwise: it filters over frames).
  tape       with truth.json: each LED's forward and lateral against the pose. FAIL above TOL_MM once the camera
             is calibrated (calibration.json "source" not the default guess), WARN before.
Writes vo_validation.json and vo_validation.png into the folder. Exit 0 when nothing FAILed.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
GOLDEN = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(GOLDEN))
sys.path.insert(0, str(GOLDEN / "framegrab"))
sys.path.insert(0, str(GOLDEN / "framegrab" / "vendor" / "tools"))

import vo_model as vm        # noqa: E402
import vo_scene as vs        # noqa: E402
import validate_grab as fg   # noqa: E402  (load_grab, ensure_decoded, Report)

LAYOUT = 2
N_WORDS = 21
KEPT_PARAMS = vm.DetectParams(MIN_PIXELS=2, MAX_PIXELS=1500, W=320, H=240)
TOL_MM = 80                  # the datasheet's range threshold
CAL_FILE = HERE / "calibration.json"


def load_calibration(path=CAL_FILE):
    c = json.load(open(path))
    q = c["q"]
    g = vm.GeomParams(F_Q4=q["F_Q4"], CX_Q4=q["CX_Q4"], CY_Q4=q["CY_Q4"], SIN_Q14=q["SIN_Q14"],
                      COS_Q14=q["COS_Q14"], DH_MM=q["DH_MM"])
    return c, g


def _s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def decode_words(words):
    w = [int(v) & 0xFFFFFFFF for v in words]
    if len(w) != N_WORDS:
        raise ValueError(f"{len(w)} result words; vo_grab_results.sv gives {N_WORDS}")

    def det(a, b):
        return dict(x_q4=a >> 18, y_q4=(a >> 4) & 0x3FFF, valid=a & 1, flags=b >> 27, count=b & 0x7FFFF)
    s = w[1]
    d = dict(live_no=w[0] >> 16, result_no=w[0] & 0xFFFF, magic=s >> 24, layout=(s >> 16) & 0xFF,
             sel=(s >> 6) & 3, frame_good=(s >> 5) & 1, both_done=(s >> 4) & 1,
             full=[det(w[2 + 2 * k], w[3 + 2 * k]) for k in range(3)],
             kept=[det(w[8 + 2 * k], w[9 + 2 * k]) for k in range(3)],
             leds=[dict(forward=w[14 + k] >> 16, lateral=_s16(w[14 + k])) for k in range(3)],
             words=w)
    p = w[17]
    for k in range(3):
        d["leds"][k]["valid"] = (p >> (16 + k)) & 1
    d["pair"] = dict(pair_valid=(p >> 19) & 1, pair_ok=(p >> 20) & 1, heading_q12=_s16(p))
    d["track"] = dict(range_mm=_s16(w[18] >> 16), vel_mm_s=_s16(w[18]), lateral_mm=_s16(w[19] >> 16),
                      disp_mm=_s16(w[19]), track_valid=(w[20] >> 4) & 1, settled=(w[20] >> 3) & 1, coast=w[20] & 7)
    return d


def det_text(d):
    if not d["valid"]:
        return f"none (flags {d['flags']:#x}, {d['count']} px)"
    return f"({d['x_q4'] / 16:.2f}, {d['y_q4'] / 16:.2f}) px, {d['count']} px"


def truth_leds(truth, rig):
    p = truth["pose"]
    return [dict(zip(("forward", "lateral"), vs.robot_frame(rig, p["X"], p["Y"], p.get("psi_deg", 0.0), k)[:2]))
            for k in range(3)]


def validate(grab_dir, cal=None, echo=True, plot=True, tape_strict=None):
    grab_dir = Path(grab_dir)
    rep = fg.Report(grab_dir.name, echo)
    c, g = cal or load_calibration()
    try:
        info, frame = fg.load_grab(grab_dir)
        dec = decode_words(info["results"])
    except (OSError, ValueError, KeyError) as e:
        rep.add("FAIL", "load", str(e))
        return rep, None
    rep.add("INFO", "grab", f"tracking {vm.COLOURS[dec['sel']]}; calibration: {c.get('source')}")

    # integrity
    rep.ok(dec["magic"] == 0x26 and dec["layout"] == LAYOUT, "integrity",
           f"status 0x{dec['words'][1]:08x}" + ("" if dec["layout"] == LAYOUT else " -- not vo_grab_top's words"))
    fresh = (dec["live_no"] - dec["result_no"]) & 0xFFFF
    rep.ok(fresh == 1, "integrity", f"the words are the kept frame's (frame {dec['result_no']}, live {dec['live_no']})")
    rep.ok(dec["frame_good"] == 1 and dec["both_done"] == 1, "integrity",
           f"frame passed the adapter ({dec['frame_good']}), both detections reported ({dec['both_done']})")

    # KEPT exact
    want_k = vm.detect_frame(frame, True, KEPT_PARAMS)
    bad = [f"{vm.COLOURS[k]} {f} board {dec['kept'][k][f]} model {want_k[k][f]}"
           for k in range(3) for f in ("valid", "x_q4", "y_q4", "count", "flags") if dec["kept"][k][f] != want_k[k][f]]
    rep.ok(not bad, "KEPT (exact)", "equal: " + "; ".join(f"{vm.COLOURS[k]} {det_text(dec['kept'][k])}" for k in range(3))
           if not bad else "differ: " + "; ".join(bad[:4]))

    # GEOM exact, from the board's own FULL centroids
    leds_m, pair_m = vm.geometry(dec["full"], g)
    bad = [f"{vm.COLOURS[k]} {f} board {dec['leds'][k][f]} model {leds_m[k][f]}"
           for k in range(3) for f in ("valid", "forward", "lateral") if dec["leds"][k][f] != leds_m[k][f]]
    bad += [f"{f} board {dec['pair'][f]} model {pair_m[f]}" for f in ("pair_valid", "pair_ok", "heading_q12")
            if dec["pair"][f] != pair_m[f]]
    rep.ok(not bad, "GEOM (exact)", "equal: " + "; ".join(
        f"{vm.COLOURS[k]} {leds_m[k]['forward']} mm ahead, {leds_m[k]['lateral']:+d} mm" for k in range(3) if leds_m[k]["valid"])
        + (f"; heading {np.degrees(np.arctan(pair_m['heading_q12'] / 4096)):+.1f} deg" if pair_m["pair_valid"] else "")
        if not bad else "differ: " + "; ".join(bad[:4]))

    # FULL vs KEPT
    probs = []
    for k in range(3):
        f, kk = dec["full"][k], dec["kept"][k]
        if f["valid"] != kk["valid"]:
            probs.append(f"{vm.COLOURS[k]} found at full resolution {f['valid']}, in the photo {kk['valid']}")
        elif f["valid"]:
            dx, dy = f["x_q4"] / 16 - 2 * kk["x_q4"] / 16, f["y_q4"] / 16 - 2 * kk["y_q4"] / 16
            if abs(dx) > 2.5 or abs(dy) > 2.5:
                probs.append(f"{vm.COLOURS[k]} full-resolution centroid {dx:+.1f},{dy:+.1f} px from twice the photo's")
    rep.add("WARN" if probs else "PASS", "FULL (approximate)",
            "; ".join(probs) if probs else "every LED's full-resolution centroid within 2.5 px of twice the photo's")

    # tracker against this frame
    t = dec["track"]
    sel_led = dec["leds"][dec["sel"]]
    if t["track_valid"] and sel_led["valid"]:
        dr = t["range_mm"] - sel_led["forward"]
        rep.add("PASS" if abs(dr) <= 100 else "WARN", "track",
                f"range {t['range_mm']} mm (this frame's geometry {sel_led['forward']}), velocity {t['vel_mm_s']} mm/s, "
                f"lateral {t['lateral_mm']} mm, moved {t['disp_mm']} mm since lock-on")
    else:
        rep.add("INFO", "track", f"no track of {vm.COLOURS[dec['sel']]} (track_valid {t['track_valid']})")

    # tape
    acc = None
    tf = grab_dir / "truth.json"
    if tf.exists():
        truth = json.load(open(tf))
        rig = vs.Rig(H_LED=vs.Rig().H_CAM + g.DH_MM)
        tl = truth_leds(truth, rig)
        errs = [(k, dec["leds"][k]["forward"] - tl[k]["forward"], dec["leds"][k]["lateral"] - tl[k]["lateral"])
                for k in range(3) if dec["leds"][k]["valid"]]
        acc = dict(pose=truth["pose"], errors=[dict(colour=vm.COLOURS[k], forward_err=fe, lateral_err=le,
                                                    forward_true=tl[k]["forward"]) for k, fe, le in errs])
        if errs:
            worst = max(abs(fe) for _, fe, _ in errs)
            calibrated = ("default" not in str(c.get("source", "default"))) if tape_strict is None else tape_strict
            word = "PASS" if worst <= TOL_MM else ("FAIL" if calibrated else "WARN")
            rep.add(word, "tape", "; ".join(f"{vm.COLOURS[k]} forward {fe:+.0f} mm, lateral {le:+.0f} mm" for k, fe, le in errs)
                    + f" (threshold {TOL_MM} mm" + ("" if calibrated else "; camera not calibrated yet: vo_calibrate.py") + ")")
        else:
            rep.add("WARN", "tape", "no LED measured in this frame")

    summary = dict(grab=grab_dir.name, sel=dec["sel"], decoded={k: v for k, v in dec.items() if k != "words"},
                   model_kept=want_k, model_geom=dict(leds=leds_m, pair=pair_m), tape=acc,
                   checks=[dict(result=w, check=c_, text=t_) for w, c_, t_ in rep.lines], fails=rep.fails, warns=rep.warns)
    json.dump(summary, open(grab_dir / "vo_validation.json", "w"), indent=2, default=int)
    if plot:
        try:
            plot_grab(grab_dir, frame, dec, want_k, rep)
        except Exception as e:
            rep.add("INFO", "plot", f"vo_validation.png not written ({e})")
    return rep, summary


def plot_grab(grab_dir, frame, dec, want_k, rep):
    from matplotlib.figure import Figure
    import grab as vgrab
    fig = Figure(figsize=(7.5, 5.6))
    ax = fig.subplots()
    ax.imshow(vgrab.rgb565_to_rgb888(frame), interpolation="nearest")
    cols = ("red", "lime", "deepskyblue")
    for k in range(3):
        f, kk = dec["full"][k], dec["kept"][k]
        if f["valid"]:
            ax.plot(f["x_q4"] / 32, f["y_q4"] / 32, "o", mfc="none", color=cols[k], ms=14, mew=1.5)
        if kk["valid"]:
            ax.plot(kk["x_q4"] / 16, kk["y_q4"] / 16, "+", color="white", ms=12, mew=1.5)
        if want_k[k]["valid"]:
            ax.plot(want_k[k]["x_q4"] / 16, want_k[k]["y_q4"] / 16, "x", color="black", ms=8, mew=1)
        if dec["leds"][k]["valid"]:
            ax.text(dec["full"][k]["x_q4"] / 32 + 8, dec["full"][k]["y_q4"] / 32 - 8,
                    f"{dec['leds'][k]['forward']} mm\n{dec['leds'][k]['lateral']:+d}", color="white", fontsize=8,
                    bbox=dict(facecolor="black", alpha=0.5, pad=1))
    t = dec["track"]
    verdict = "FAIL" if rep.fails else ("PASS (with warnings)" if rep.warns else "PASS")
    ax.set_title(f"{grab_dir.name}: {verdict}. Tracking {vm.COLOURS[dec['sel']]}: "
                 + (f"{t['range_mm']} mm, {t['vel_mm_s']} mm/s" if t["track_valid"] else "no track"), fontsize=9)
    ax.set_xticks([]); ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(grab_dir / "vo_validation.png", dpi=90)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("grabs", nargs="+")
    ap.add_argument("--calibration", type=Path, default=CAL_FILE)
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args(argv)
    cal = load_calibration(a.calibration)
    fails = warns = 0
    for d in a.grabs:
        rep, _ = validate(d, cal, plot=not a.no_plot)
        fails += rep.fails
        warns += rep.warns
    print(f"{'PASS' if fails == 0 else 'FAIL'}: vo_validate_grab: {len(a.grabs)} grab(s), {fails} failed, {warns} warning(s)")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
