#!/usr/bin/env python3
"""vo_calibrate.py -- Group 26 (Wilson, M2): the camera's focal length, tilt and LED height for the beacon VO,
from photos taken at tape-measured spots.

    python golden/vo/vo_calibrate.py golden/framegrab/grabs/cal_*          fit and print, change nothing
    python golden/vo/vo_calibrate.py golden/framegrab/grabs/cal_* --write  also rewrite calibration.json and
                                                                          rtl/vo/vo_calibration.svh (then recompile)
    python golden/vo/vo_calibrate.py --selftest                            recover a known camera from simulated photos

TAKING THE PHOTOS (vo_grab_top on the board, all three LEDs on, SW9 up so the exposure stays put):
    python golden/vo/vo_take_photo.py cal_600  --Y -600            camera lens 600 mm from the LED line, centred on
    python golden/vo/vo_take_photo.py cal_900  --Y -900  --X 150   green, squared up; X = sideways (right +)
    ...  four to six spots between 400 mm and 1.5 m, some off-centre. Measure from the camera LENS to the wall the
    LEDs are on, square to it. Each photo's truth.json holds the pose.

WHAT IS FITTED: f (pixels), cx (pixels), the pitch (degrees, up +) and DH (LED height above the lens, mm), by
least squares on the reprojection error of every LED in every photo (the board's own full-resolution centroids,
words 2..7). cy is held at the image centre: it trades almost one for one with the pitch. --fix-dh holds DH at
a value you measured. The fit reports the reprojection error in pixels and each photo's range error before and
after: those are the numbers for the report's simulation-to-hardware section.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(HERE))

import vo_model as vm        # noqa: E402
import vo_scene as vs        # noqa: E402

SVH = ROOT / "rtl" / "vo" / "vo_calibration.svh"
CAL = HERE / "calibration.json"


def rig_of(p, cy):
    f, cx, pitch, dh = p
    return vs.Rig(F_PX=f, CX=cx, CY=cy, PITCH_DEG=pitch, H_LED=vs.Rig().H_CAM + dh)


def predict(p, cy, obs):
    rig = rig_of(p, cy)
    out = []
    for (pose, k, u, v) in obs:
        fwd, lat, up = vs.robot_frame(rig, pose["X"], pose["Y"], pose.get("psi_deg", 0.0), k)
        pr = vs.project(rig, fwd, lat, up)
        out += [pr[0], pr[1]] if pr else [1e4, 1e4]
    return np.array(out)


def fit(obs, p0, cy=239.5, fix_dh=None, iters=50):
    """Gauss-Newton with a numeric Jacobian (numpy only, so it runs on any laptop)."""
    meas = np.array([c for (_, _, u, v) in obs for c in (u, v)])
    free = [0, 1, 2] + ([] if fix_dh is not None else [3])
    p = np.array(p0, dtype=float)
    if fix_dh is not None:
        p[3] = fix_dh
    step = np.array([0.5, 0.2, 0.01, 0.2])
    for _ in range(iters):
        r = predict(p, cy, obs) - meas
        J = np.zeros((len(r), len(free)))
        for j, i in enumerate(free):
            q = p.copy()
            q[i] += step[i]
            J[:, j] = (predict(q, cy, obs) - meas - r) / step[i]
        dp, *_ = np.linalg.lstsq(J, -r, rcond=None)
        p[free] += dp
        if np.max(np.abs(dp) / step[free]) < 1e-3:
            break
    r = predict(p, cy, obs) - meas
    return p, float(np.sqrt(np.mean(r ** 2)))


def q_params(p, cy):
    f, cx, pitch, dh = p
    s, c = vm.pitch_q14(pitch)
    return dict(F_Q4=int(round(f * 16)), CX_Q4=int(round(cx * 16)), CY_Q4=int(round(cy * 16)), SIN_Q14=s, COS_Q14=c,
                DH_MM=int(round(dh)))


def range_errors(p, cy, grabs):
    """Each photo: the geometry's forward error for every LED found, with these parameters (integer RTL maths)."""
    g = vm.GeomParams(**q_params(p, cy))
    out = []
    for (name, pose, dets) in grabs:
        rig = rig_of(p, cy)
        errs = []
        for k in range(3):
            if dets[k]["valid"]:
                led = vm.led_geometry(dets[k], g)
                if led["valid"]:
                    errs.append(led["forward"] - vs.robot_frame(rig, pose["X"], pose["Y"], pose.get("psi_deg", 0.0), k)[0])
        out.append((name, -pose["Y"], errs))
    return out


def load_grabs(dirs):
    sys.path.insert(0, str(HERE.parent / "framegrab"))
    sys.path.insert(0, str(HERE.parent / "framegrab" / "vendor" / "tools"))
    import validate_grab as fg
    import vo_validate_grab as vv
    grabs = []
    for d in dirs:
        d = Path(d)
        if not (d / "truth.json").exists():
            print(f"skip {d}: no truth.json (take it with vo_take_photo.py --Y ...)")
            continue
        info, _ = fg.load_grab(d)
        dec = vv.decode_words(info["results"])
        grabs.append((d.name, json.load(open(d / "truth.json"))["pose"], dec["full"]))
    return grabs


def write_out(p, cy, rms, n, source):
    q = q_params(p, cy)
    cal = dict(source=source, f_px=round(p[0], 2), cx=round(p[1], 2), cy=cy, pitch_deg=round(p[2], 3),
               dh_mm=int(round(p[3])), rms_px=round(rms, 3), photos=n, q=q)
    json.dump(cal, open(CAL, "w"), indent=2)
    SVH.write_text(f"""// Group 26 - beacon VO calibration (written by golden/vo/vo_calibrate.py; edit there, not here).
// Source: {source}: f {p[0]:.1f} px, cx {p[1]:.1f}, cy {cy}, pitch {p[2]:.2f} deg up, LEDs {p[3]:.0f} mm above
// the lens; reprojection error {rms:.2f} px rms over {n} photos.
`ifndef VO_CALIBRATION_SVH
`define VO_CALIBRATION_SVH
`define VO_F_Q4    {q['F_Q4']}
`define VO_CX_Q4   {q['CX_Q4']}
`define VO_CY_Q4   {q['CY_Q4']}
`define VO_SIN_Q14 {q['SIN_Q14']}
`define VO_COS_Q14 {q['COS_Q14']}
`define VO_DH_MM   {q['DH_MM']}
`endif
""")
    print(f"wrote {CAL} and {SVH}: recompile vo_grab (and the robot) for the new numbers")


def report(grabs, p0, p, cy, rms0, rms):
    print(f"fit over {len(grabs)} photos: f {p[0]:.1f} px (was {p0[0]:.1f}), cx {p[1]:.1f} (was {p0[1]:.1f}), "
          f"pitch {p[2]:.2f} deg (was {p0[2]:.2f}), DH {p[3]:.1f} mm (was {p0[3]:.1f}); "
          f"reprojection rms {rms:.2f} px (was {rms0:.2f})")
    before, after = range_errors(p0, cy, grabs), range_errors(p, cy, grabs)
    print(f"{'photo':<22}{'distance':>10}  {'range error before':>22}  {'after':>22}")
    worst = 0.0
    for (n, d, eb), (_, _, ea) in zip(before, after):
        fmt = lambda e: " ".join(f"{x:+.0f}" for x in e) or "-"
        print(f"{n:<22}{d:>8.0f} mm  {fmt(eb):>22}  {fmt(ea):>22}")
        worst = max([worst] + [abs(x) for x in ea])
    print(f"worst range error after calibration: {worst:.0f} mm")
    return worst


def selftest():
    """A camera unlike the default guess, photos at nine spots with 0.15 px centroid noise: the fit must find it."""
    true = np.array([611.0, 323.0, 9.5, 152.0])
    cy = 239.5
    rng = np.random.default_rng(26)
    poses = [dict(X=x, Y=y, psi_deg=0.0) for y in (-450, -700, -1000, -1400) for x in (0, 200)] + [dict(X=-150, Y=-850)]
    obs, grabs = [], []
    rig = rig_of(true, cy)
    for i, pose in enumerate(poses):
        dets = []
        for k in range(3):
            pr = vs.project(rig, *vs.robot_frame(rig, pose["X"], pose["Y"], 0.0, k))
            u, v = pr[0] + rng.normal(0, 0.15), pr[1] + rng.normal(0, 0.15)
            ok = 0 <= u < 640 and 0 <= v < 480
            if ok:
                obs.append((pose, k, u, v))
            dets.append(dict(valid=int(ok), x_q4=int(u * 16) if ok else 0, y_q4=int(v * 16) if ok else 0))
        grabs.append((f"sim_{i}", pose, dets))
    c = json.load(open(CAL))
    p0 = np.array([c["f_px"], c["cx"], c["pitch_deg"], c["dh_mm"]], dtype=float)
    rms0 = float(np.sqrt(np.mean((predict(p0, cy, obs) - np.array([q for o in obs for q in o[2:]])) ** 2)))
    p, rms = fit(obs, p0, cy)
    worst = report(grabs, p0, p, cy, rms0, rms)
    ok = abs(p[0] - true[0]) < 3 and abs(p[1] - true[1]) < 1.5 and abs(p[2] - true[2]) < 0.2 and abs(p[3] - true[3]) < 2 \
        and worst <= 15
    print(("PASS" if ok else "FAIL") + f": vo_calibrate selftest (true f {true[0]}, cx {true[1]}, pitch {true[2]}, DH {true[3]}: "
          f"found {p[0]:.1f}, {p[1]:.1f}, {p[2]:.2f}, {p[3]:.1f}; worst range error {worst:.0f} mm)")
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("grabs", nargs="*")
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--fix-dh", type=float, default=None, help="hold DH (mm) at a measured value")
    ap.add_argument("--cy", type=float, default=239.5)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args(argv)
    if a.selftest:
        return selftest()
    grabs = load_grabs(a.grabs)
    obs = [(pose, k, d[k]["x_q4"] / 16, d[k]["y_q4"] / 16) for (_, pose, d) in grabs for k in range(3) if d[k]["valid"]]
    if len(grabs) < 2 or len(obs) < 6:
        print(f"FAIL: vo_calibrate: {len(grabs)} photos with truth.json and {len(obs)} LEDs found; need 2+ photos and 6+ LEDs")
        return 1
    c = json.load(open(CAL))
    p0 = np.array([c["f_px"], c["cx"], c["pitch_deg"], c["dh_mm"]], dtype=float)
    meas = np.array([q for o in obs for q in o[2:]])
    rms0 = float(np.sqrt(np.mean((predict(p0, a.cy, obs) - meas) ** 2)))
    p, rms = fit(obs, p0, a.cy, a.fix_dh)
    report(grabs, p0, p, a.cy, rms0, rms)
    if a.write:
        write_out(p, a.cy, rms, len(grabs), f"measured: {len(grabs)} photos ({', '.join(n for n, _, _ in grabs)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
