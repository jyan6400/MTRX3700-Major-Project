#!/usr/bin/env python3
"""vo_check.py -- Group 26 (Wilson, M2): the beacon visual odometry RTL against its golden model.

    python golden/vo/vo_check.py            the math fuzz (two calibrations) and every drive scene
    python golden/vo/vo_check.py --quick    fewer vectors, shorter drives (run_all_tests.sh)
    python golden/vo/vo_check.py --broken   beacon_geometry with a pitch sign bug: must be CAUGHT
    python golden/vo/vo_check.py --model-only   the accuracy figures from the model alone (no Verilator)

Checks, each a PASS / FAIL line:
  math      sim/vo/beacon_math_tb.sv: beacon_geometry -> beacon_tracker -> vo_standoff on random and
            trajectory-like detections (both calibrations): every output of every frame equals
            vo_model.py.
  scene     sim/vo/beacon_vo_scene_tb.sv: drives rendered by vo_scene.py through camera_stream_adapter
            and beacon_vo (both clocks, the CDC): every detection (D) and every VO update (V) equals
            vo_model.py, frame for frame.
  accuracy  the same drives against the ground truth (the pose, not the pixels): the predicted
            performance for the datasheet. Simulation is ideal discs: the hardware error budget
            (lens, calibration, blooming, motion blur) comes on top (vo_calibrate.py, the notebook).
Last line: PASS: vo_check / FAIL / CAUGHT.
"""
from __future__ import annotations

import argparse
import json
import os
import random
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

import vo_model as vm      # noqa: E402
import vo_scene as vs      # noqa: E402

BUILD = ROOT / "build" / "vo"
VO_RTL = ["rtl/vo/vo_divider.sv", "rtl/vo/beacon_detect.sv", "rtl/vo/vo_mailbox_cdc.sv",
          "rtl/vo/beacon_geometry.sv", "rtl/vo/beacon_tracker.sv", "rtl/vo/vo_standoff.sv", "rtl/vo/beacon_vo.sv"]
DEPS = ["rtl/image_processing/led_colour_mask.sv", "rtl/image_processing/colour_run_filter.sv",
        "rtl/integration/camera_stream_adapter.sv"]
BROKEN = {"rtl/vo/beacon_geometry.sv": "sim/vo/broken/beacon_geometry_broken.sv"}

# accuracy thresholds the simulation must meet (ideal discs; hardware adds its own error on top)
SIM_RANGE_ERR_MM = 15        # |forward error| of the selected LED, frames inside 0.3 to 1.6 m
SIM_VEL_ERR_MM_S = 120       # |velocity error| once settled, steady motion


def report(ok, name, text, word=None):
    w = word or ("PASS" if ok else "FAIL")
    print(f"{w}: vo {name}: {text}", flush=True)
    return ok


def build(top, extra=(), tag="", subst=None, gparams=None):
    out = BUILD / f"{top}{tag}"
    exe = out / f"V{top}"
    srcs = [str(ROOT / (subst or {}).get(s, s)) for s in DEPS + VO_RTL] + [str(ROOT / "sim/vo" / f"{top}.sv")]
    sig = "\n".join(srcs + [str(gparams)])
    stamp = out / "sources.txt"
    if exe.exists() and stamp.exists() and stamp.read_text() == sig and \
            exe.stat().st_mtime > max(os.path.getmtime(s) for s in srcs):
        return exe
    out.mkdir(parents=True, exist_ok=True)
    g = [f"-G{k}={v}" for k, v in (gparams or {}).items()]
    cmd = ["bash", str(GOLDEN / "verilator_build.sh"), str(out), top, "-Wno-fatal", "-Wno-lint", "-Wno-style",
           "-O1"] + g + list(extra) + srcs
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
    if r.returncode != 0 or not exe.exists():
        raise RuntimeError(f"{top} build failed:\n" + (r.stdout + r.stderr)[-2500:])
    stamp.write_text(sig)
    return exe


def run(exe, args, cwd):
    r = subprocess.run([str(exe)] + args, capture_output=True, text=True, cwd=cwd)
    log = r.stdout + r.stderr
    ok = any(l.startswith("PASS:") for l in log.splitlines())
    return ok, log


V_FIELDS = ["idx", "lv0", "f0", "l0", "lv1", "f1", "l1", "lv2", "f2", "l2", "pair_valid", "pair_ok", "heading",
            "tv", "settled", "range", "vel", "lat", "disp", "coast", "reached", "lost", "sel"]
D_FIELDS = ["idx", "good"] + [f"{n}{k}" for k in range(3) for n in ("v", "x", "y", "n", "fl", "bx0", "bx1", "by0", "by1")]


def read_rec(path, kind, fields):
    rows = []
    for line in open(path):
        s = line.split()
        if s and s[0] == kind:
            rows.append(dict(zip(fields, map(int, s[1:]))))
    return rows


def model_v(leds, pair, trk, so, sel):
    d = dict(pair_valid=pair["pair_valid"], pair_ok=pair["pair_ok"], heading=pair["heading_q12"],
             tv=trk["track_valid"], settled=trk["settled"], range=trk["range_mm"], vel=trk["vel_mm_s"],
             lat=trk["lateral_mm"], disp=trk["disp_mm"], coast=trk["coast"], reached=so["reached"],
             lost=so["lost"], sel=sel)
    for k in range(3):
        d[f"lv{k}"], d[f"f{k}"], d[f"l{k}"] = leds[k]["valid"], leds[k]["forward"], leds[k]["lateral"]
    return d


def first_diff(got, want, keys):
    for k in keys:
        if int(got[k]) != int(want[k]):
            return k
    return None


# ---------------------------------------------------------------- math fuzz

CALIBRATIONS = {
    "default": vm.GeomParams(),
    "level_long": vm.GeomParams(F_Q4=7000, CX_Q4=5000, CY_Q4=3700, SIN_Q14=0, COS_Q14=16384, DH_MM=200),
}


def math_vectors(n, seed, g):
    rnd = random.Random(seed)
    rig = vs.Rig(H_LED=300.0 + g.DH_MM, PITCH_DEG=float(np.degrees(np.arcsin(g.SIN_Q14 / 16384))),
                 F_PX=g.F_Q4 / 16, CX=g.CX_Q4 / 16, CY=g.CY_Q4 / 16)
    vec = []
    sel = 1
    y = -400.0
    vel = 0.0
    x_off = 0.0
    for i in range(n):
        if rnd.random() < 0.01:
            sel = rnd.randrange(3)
        if rnd.random() < 0.05:
            vel = rnd.uniform(-30, 30)
        y = min(-250.0, max(-3500.0, y - vel))
        if rnd.random() < 0.02:
            x_off = rnd.uniform(-600, 600)
        valid, xs, ys = 0, [0, 0, 0], [0, 0, 0]
        mode = rnd.random()
        for k in range(3):
            if mode < 0.65:                                   # a consistent scene, with noise
                f, l, u = vs.robot_frame(rig, x_off, y, rnd.uniform(-3, 3), k)
                pr = vs.project(rig, f, l, u)
                if pr is None:
                    continue
                px = pr[0] * 16 + rnd.gauss(0, 6)
                py = pr[1] * 16 + rnd.gauss(0, 6)
                if rnd.random() < 0.03:
                    py += rnd.choice([-1, 1]) * rnd.uniform(200, 2000)   # an outlier
                if 0 <= px < 16384 and 0 <= py < 16384 and rnd.random() < 0.9:
                    valid |= 1 << k
                    xs[k], ys[k] = int(px), int(py)
            else:                                             # anything: corners, below the horizon
                if rnd.random() < 0.6:
                    valid |= 1 << k
                    xs[k] = rnd.choice([0, 16383, rnd.randrange(16384), rnd.randrange(10240)])
                    ys[k] = rnd.choice([0, 16383, rnd.randrange(16384), rnd.randrange(7680)])
        en = int(rnd.random() < 0.7)
        appr = int(rnd.random() < 0.3)
        tgt = rnd.choice([0, 500, 800, 1000, 1500, 4000, 65535])
        vec.append((valid, xs, ys, sel, en, appr, tgt))
    return vec


def check_math(n, subst=None, tag=""):
    ok_all = True
    for cname, g in CALIBRATIONS.items():
        gp = dict(F_Q4=g.F_Q4, CX_Q4=g.CX_Q4, CY_Q4=g.CY_Q4, SIN_Q14=g.SIN_Q14, COS_Q14=g.COS_Q14, DH_MM=g.DH_MM)
        try:
            exe = build("beacon_math_tb", tag=f"_{cname}{tag}", subst=subst, gparams=gp)
        except RuntimeError as e:
            ok_all = report(False, f"math {cname}", str(e))
            continue
        d = BUILD / f"math_{cname}{tag}"
        d.mkdir(parents=True, exist_ok=True)
        vec = math_vectors(n, 26 + len(cname), g)
        with open(d / "vec.txt", "w") as f:
            for (v, xs, ys, s, e, a, t) in vec:
                f.write(f"S {v} {xs[0]} {ys[0]} {xs[1]} {ys[1]} {xs[2]} {ys[2]} {s} {e} {a} {t}\n")
        ok, log = run(exe, ["+VEC=vec.txt", "+OUT=out.rec"], d)
        if not ok:
            ok_all = report(False, f"math {cname}", log[-400:])
            continue
        got = read_rec(d / "out.rec", "V", V_FIELDS)
        trk, so = vm.Tracker(), vm.Standoff()
        bad, first = 0, None
        stats = dict(leds=0, pairs=0, tracks=0, reached=0, rejected=0, pair_ok=0, coast=0, drops=0, restarts=0)
        prev_tv = 0
        for i, (v, xs, ys, s, e, a, t) in enumerate(vec):
            dets = [dict(valid=(v >> k) & 1, x_q4=xs[k], y_q4=ys[k]) for k in range(3)]
            leds, pair = vm.geometry(dets, g)
            tk = trk.update(leds, s)
            sr = so.update(e, a, t, tk)
            want = model_v(leds, pair, tk, sr, s)
            stats["leds"] += sum(l["valid"] for l in leds)
            stats["pairs"] += pair["pair_valid"]
            stats["tracks"] += tk["track_valid"]
            stats["reached"] += sr["reached"]
            stats["rejected"] += sum(((v >> k) & 1) and not leds[k]["valid"] for k in range(3))
            stats["pair_ok"] += pair["pair_ok"]
            stats["coast"] += tk["coast"] > 0
            stats["drops"] += prev_tv and not tk["track_valid"]
            stats["restarts"] += tk["track_valid"] and trk.age == 1 and prev_tv
            prev_tv = tk["track_valid"]
            k = first_diff(got[i], want, V_FIELDS[1:]) if i < len(got) else "missing"
            if k:
                bad += 1
                if first is None:
                    first = f"frame {i} field {k}: RTL {got[i].get(k) if i < len(got) else '-'} model {want.get(k)}"
        ok_all &= report(bad == 0 and len(got) == len(vec), f"math {cname}",
                         f"{len(vec)} frames: {stats['leds']} LEDs placed, {stats['rejected']} rejected by the geometry, "
                         f"{stats['pairs']} pairs ({stats['pair_ok']} consistent), {stats['tracks']} tracked, {stats['coast']} "
                         f"coasting, {stats['drops']} drops, {stats['restarts']} restarts, {stats['reached']} at stand-off: " +
                         ("every output equals the model" if bad == 0 else f"{bad} frames differ; first {first}"))
    return ok_all


# ---------------------------------------------------------------- drive scenes

def scene_model(sc, frames, rig):
    """The model, frame by frame, with the inputs each result really sees (a cut-short frame's
    result arrives at the next frame's start, with that frame's sel and stand-off inputs)."""
    g = rig.geom_params()
    trk, so = vm.Tracker(), vm.Standoff()
    out = []
    n = len(frames)
    for i, f in enumerate(frames):
        j = i + 1 if (not sc.good[i] and i + 1 < n) else i
        sel = sc.sel[j]
        en, ap, tg = sc.standoff[j] if sc.standoff else (0, 0, 0)
        dets = vm.detect_frame(f, bool(sc.good[i]))
        leds, pair = vm.geometry(dets, g)
        tk = trk.update(leds, sel)
        sr = so.update(en, ap, tg, tk)
        out.append(dict(dets=dets, leds=leds, pair=pair, trk=tk, so=sr, sel=sel))
    return out


def accuracy(sc, fr, res):
    """Errors of the selected LED's forward and lateral, and of the velocity, against the truth."""
    fe, le, ve = [], [], []
    for i, r in enumerate(res):
        k = r["sel"]
        truth = fr[i][1][k]
        if r["leds"][k]["valid"] and 300 <= truth["forward"] <= 1600:
            fe.append(r["leds"][k]["forward"] - truth["forward"])
            le.append(r["leds"][k]["lateral"] - truth["lateral"])
        if r["trk"]["settled"] and 0 < i < len(res) - 1 and sc.sel[i - 1] == sc.sel[i] == sc.sel[i + 1]:
            tv = (fr[i + 1][1][k]["forward"] - fr[i - 1][1][k]["forward"]) / 2 * 50
            if all(sc.good[i - 2:i + 1]) and all(sc.leds_on[j][k] for j in range(max(0, i - 4), i + 1)):
                ve.append(r["trk"]["vel_mm_s"] - tv)
    a = lambda x: np.abs(np.array(x)) if x else np.array([0.0])
    return dict(n_range=len(fe), range_max=float(a(fe).max()), range_mean=float(np.mean(fe)) if fe else 0.0,
                lat_max=float(a(le).max()), n_vel=len(ve), vel_max=float(a(ve).max()),
                vel_rms=float(np.sqrt(np.mean(np.square(ve)))) if ve else 0.0)


def check_scenes(quick, subst=None, tag="", model_only=False):
    rig = vs.Rig()
    bg = vs.background()
    d = BUILD / f"scenes{tag}"
    d.mkdir(parents=True, exist_ok=True)
    vs.write_background(d / "background.hex", bg)
    exe = None
    if not model_only:
        try:
            exe = build("beacon_vo_scene_tb", tag=tag, subst=subst)
        except RuntimeError as e:
            return report(False, "scene", str(e)), {}
    ok_all = True
    summary = {}
    for sc in vs.scenarios(quick):
        fr = vs.write_scene(d / f"{sc.name}.txt", sc, rig)
        frames = [vs.render(bg, discs) for discs, _ in fr]
        res = scene_model(sc, frames, rig)
        acc = accuracy(sc, fr, res)
        summary[sc.name] = acc
        if exe is not None:
            ok, log = run(exe, [f"+BG=background.hex", f"+SCENE={sc.name}.txt", f"+OUT={sc.name}.rec"], d)
            if not ok:
                ok_all = report(False, f"scene {sc.name}", log[-500:])
                continue
            D = read_rec(d / f"{sc.name}.rec", "D", D_FIELDS)
            V = read_rec(d / f"{sc.name}.rec", "V", V_FIELDS)
            bad, first = 0, None
            for i, r in enumerate(res):
                wd = dict(good=int(sc.good[i]))
                for k in range(3):
                    b = r["dets"][k]
                    wd.update({f"v{k}": b["valid"], f"x{k}": b["x_q4"], f"y{k}": b["y_q4"], f"n{k}": b["count"],
                               f"fl{k}": b["flags"], f"bx0{k}": b["bx0"], f"bx1{k}": b["bx1"], f"by0{k}": b["by0"],
                               f"by1{k}": b["by1"]})
                wv = model_v(r["leds"], r["pair"], r["trk"], r["so"], r["sel"])
                kd = first_diff(D[i], wd, D_FIELDS[1:]) if i < len(D) else "missing D"
                kv = first_diff(V[i], wv, V_FIELDS[1:]) if i < len(V) else "missing V"
                if kd or kv:
                    bad += 1
                    if first is None:
                        first = (f"frame {i}: " + (f"detect {kd} RTL {D[i].get(kd) if i < len(D) else '-'} model {wd.get(kd)}" if kd
                                 else f"VO {kv} RTL {V[i].get(kv) if i < len(V) else '-'} model {wv.get(kv)}"))
            reached_at = next((i for i, r in enumerate(res) if r["so"]["reached"]), None)
            ok_all &= report(bad == 0 and len(D) == len(V) == len(res), f"scene {sc.name}",
                             f"{len(res)} frames through the adapter, detect, CDC, geometry, tracker, stand-off: " +
                             ("every detection and update equals the model" if bad == 0 else f"{bad} differ; first {first}") +
                             ("" if reached_at is None else f"; stand-off reached at frame {reached_at}"))
        ok_acc = acc["range_max"] <= SIM_RANGE_ERR_MM and (acc["n_vel"] == 0 or acc["vel_max"] <= SIM_VEL_ERR_MM_S)
        ok_all &= report(ok_acc, f"accuracy {sc.name}",
                         f"selected LED range error max {acc['range_max']:.1f} mm (mean {acc['range_mean']:+.1f}) over "
                         f"{acc['n_range']} frames in 0.3-1.6 m, lateral max {acc['lat_max']:.1f} mm; velocity error "
                         f"max {acc['vel_max']:.0f} mm/s, rms {acc['vel_rms']:.0f} over {acc['n_vel']} settled frames "
                         f"(limits {SIM_RANGE_ERR_MM} mm, {SIM_VEL_ERR_MM_S} mm/s)")
    return ok_all, summary


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--broken", action="store_true")
    ap.add_argument("--model-only", action="store_true")
    a = ap.parse_args(argv)
    t0 = time.time()
    if a.broken:
        ok = check_math(800, subst=BROKEN, tag="_broken")
        if not ok:
            print("CAUGHT: beacon_geometry_broken (pitch sign): the math fuzz differs from the golden model")
            return 0
        print("FAIL: beacon_geometry_broken was NOT caught")
        return 1
    ok = True
    if not a.model_only:
        ok &= check_math(1500 if a.quick else 6000)
    ok_s, summary = check_scenes(a.quick, model_only=a.model_only)
    ok &= ok_s
    BUILD.mkdir(parents=True, exist_ok=True)
    json.dump(summary, open(BUILD / "accuracy.json", "w"), indent=2)
    dt = time.time() - t0
    if ok:
        print(f"PASS: vo_check (RTL == golden model on every frame, simulated accuracy within limits, {dt:.0f} s)")
        return 0
    print(f"FAIL: vo_check ({dt:.0f} s)")
    return 1


if __name__ == "__main__":
    sys.exit(main())
