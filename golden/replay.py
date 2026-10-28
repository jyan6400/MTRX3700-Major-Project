"""Replays: golden model vs RTL on generated stimulus, and board vs model vs RTL
on a capture. Used by check_golden.py, run_all_tests.sh and the notebook."""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

import capture as capmod   # noqa: E402
import compare             # noqa: E402
import model               # noqa: E402
import stimulus            # noqa: E402
import vlog                # noqa: E402

BUILD = ROOT / "build/golden"


def have_rtl():
    """True when the RTL replay can run here (bash + Verilator)."""
    return shutil.which("bash") is not None and shutil.which("verilator") is not None


def run_rtl(stims_recs, small=None, subst=None, tag=""):
    """Run golden/run_rtl_replay.sh on [(stim, rec), ...]."""
    args = ["bash", str(HERE / "run_rtl_replay.sh")]
    if small:
        args += ["-s", str(small[0]), str(small[1])]
    for s, r in stims_recs:
        args += [str(s), str(r)]
    env = dict(os.environ)
    if subst:
        env["REPLAY_SUBST"] = subst
        env["REPLAY_TAG"] = tag or "_subst"
    r = subprocess.run(args, capture_output=True, text=True, env=env)
    if r.returncode != 0:
        raise RuntimeError(r.stdout + r.stderr)


# ---------------------------------------------------------------- scenarios


@dataclass
class ScenarioResult:
    name: str
    params: model.ChainParams
    events: np.ndarray
    model: model.ChainResult
    model_rec: vlog.Records
    rtl_rec: vlog.Records | None
    report: compare.Report | None


def scenario_set(quick=False, minimal=False):
    """(name, events, ChainParams, small) for every generated scenario.
    minimal: only the two frames that hold every colour code (fast mask check)."""
    small = model.ChainParams(adapter=model.AdapterParams(W=32, H=24))
    full = model.ChainParams()
    out = []
    F = stimulus.synthetic_frames()
    if minimal:
        return [("mask_all_codes", stimulus.frames_to_events(
            [F["all_codes"], F["near_threshold"]], [0, 1], stimulus.COMPACT), full, None)]
    names = list(F)
    if quick:   # each frame once, SW1 alternating, plus the fill-gate frame in both modes
        seq = names + ["two_discs"]
        sw = [i % 2 for i in range(len(names))] + [1 - names.index("two_discs") % 2]
    else:       # every frame in both SW1 modes
        seq, sw = names * 2, [0] * len(names) + [1] * len(names)
    out.append(("synthetic_gates", stimulus.frames_to_events(
        [F[n] for n in seq], sw, stimulus.COMPACT, lead_pixels=37), full, None))
    try:
        photos = ["photo_leds", "photo_two_red"] if quick else list(stimulus.PHOTO_SCENES)
        imgs = [stimulus.photo_frame(stimulus.PHOTO_SCENES[n]) for n in photos]
        out.append(("photo_camera_timing", stimulus.frames_to_events(
            imgs * 2, [0] * len(imgs) + [1] * len(imgs), stimulus.CAMERA), full, None))
    except ImportError:
        print("NOTE: golden: Pillow not installed, lab-photo scenarios skipped")
    for name, ev in stimulus.adapter_edge_cases():
        out.append((f"adapter_{name}", ev, small, (32, 24)))
    for s in range(8 if quick else 40):
        out.append((f"fuzz_{s}", stimulus.fuzz_stream(s), small, (32, 24)))
    return out


def run_scenarios(quick=False, rtl=True, subst=None, tag="", log=print, minimal=False):
    work = BUILD / ("scen" + tag)
    sets = scenario_set(quick, minimal)
    jobs = {}
    for name, ev, p, small in sets:
        stim = work / f"{name}.stim"
        vlog.write_stim(stim, ev, comment=name)
        jobs.setdefault(small, []).append((stim, work / f"{name}.rtl.rec"))
    if rtl:
        for small, lst in jobs.items():
            run_rtl(lst, small, subst, tag)
    results = []
    for name, ev, p, small in sets:
        m = model.run_chain(ev, p)
        mrec = vlog.chain_records(m)
        rrec = vlog.read_records(work / f"{name}.rtl.rec") if rtl else None
        rep = compare.compare(mrec, rrec) if rtl else None
        results.append(ScenarioResult(name, p, ev, m, mrec, rrec, rep))
        if log and rep is not None and not rep.ok:
            log(rep.summary())
    return results


# ---------------------------------------------------------------- capture


@dataclass
class CaptureReplay:
    cap: capmod.Capture
    frames: list                       # replayed BoardFrames
    events: np.ndarray | None
    model: model.ChainResult | None
    rtl_rec: vlog.Records | None
    checks: list = field(default_factory=list)      # (check, ok, detail)

    @property
    def ok(self):
        return all(ok for _, ok, _ in self.checks)

    def table(self):
        import pandas as pd
        return pd.DataFrame([dict(check=c, result="PASS" if ok else "FAIL", detail=d) for c, ok, d in self.checks])

    def frame_table(self):
        """Per replayed frame: board vs model vs RTL results, and the model's
        internals the board does not export."""
        import pandas as pd
        rows = []
        rtlR = self.rtl_rec["R"] if self.rtl_rec is not None else None
        for i, f in enumerate(self.frames):
            b = f.result
            m = self.model.results[i]
            row = dict(frame=f.index, SW1=b["sw1"],
                       board=f"{'valid' if b['valid'] else 'none'} ({b['x']},{b['y']}) n={b['count']}",
                       model=f"{'valid' if m['valid'] else 'none'} ({m['x']},{m['y']}) n={m['count']}")
            if rtlR is not None and i < len(rtlR):
                r = rtlR[i]
                row["RTL"] = f"{'valid' if r[1] else 'none'} ({r[2]},{r[3]}) n={r[4]}"
            row.update(confidence=m["conf"], flags=f"{m['flags']:04b}",
                       reason=reject_reason(m), bbox=f"x {m['bx0']}-{m['bx1']}, y {m['by0']}-{m['by1']}",
                       latency=b["latency"])
            rows.append(row)
        return pd.DataFrame(rows)


def reject_reason(r, p: model.CentroidParams = model.CentroidParams()):
    f = r["flags"]
    why = []
    if f & 1:
        why.append(f"too few pixels (<{p.MIN_PIXELS})")
    if f & 2:
        why.append(f"too many pixels (>{p.MAX_PIXELS})")
    if f & 4:
        why.append(f"aspect > {p.MAX_ASPECT}:1")
    if f & 8:
        why.append(f"fill {r['conf']}/255 < {p.MIN_CONFIDENCE}")
    if not why:
        return "accepted (all gates pass)"
    return ("accepted by SW1 bypass despite: " if r["valid"] else "rejected: ") + ", ".join(why)


def replay_capture(cap_or_path, rtl=True, params: model.ChainParams = model.ChainParams(), tag="cap"):
    cap = cap_or_path if isinstance(cap_or_path, capmod.Capture) else capmod.load(cap_or_path)
    ev, frames = cap.replay_events()
    checks = []
    add = lambda c, ok, d="": checks.append((c, bool(ok), d))

    add("capture integrity (seq toggles, every result assigned)",
        cap.seq_errors == 0 and cap.unassigned_results == 0,
        f"{len(cap.words)} samples, {cap.seq_errors} seq errors, {cap.unassigned_results} unassigned results")
    sup, n_missing = capmod.loose_is_superset(params.mask)
    add("vision_tap loose filter covers every model-red code", sup,
        "unrecorded pixels cannot be red" if sup else f"{n_missing} red codes not covered: raise LOOSE_* in vision_tap")
    complete = [f for f in cap.frames if f.complete]
    add("complete frames in the capture", len(complete) > 0, f"{len(complete)} complete of {len(cap.frames)}")
    rejected = [f for f in complete if not f.good]
    add("frames the board adapter rejected gave no target",
        all(f.result["valid"] == 0 and f.result["count"] == 0 for f in rejected),
        f"{len(rejected)} rejected frames")
    if ev is None:
        return CaptureReplay(cap, [], None, None, None, checks)

    m = model.run_chain(ev, params)
    n_px = params.adapter.W * params.adapter.H
    rrec = None
    if rtl:
        work = BUILD / tag
        stim = work / "replay.stim"
        vlog.write_stim(stim, ev)
        run_rtl([(stim, work / "replay.rtl.rec")])
        rrec = vlog.read_records(work / "replay.rtl.rec")
        rep = compare.compare(vlog.chain_records(m), rrec)
        add("RTL replay == golden model (every record, every field)", rep.ok,
            "; ".join(f"{k}:{t.n_a}" for k, t in rep.types.items()) if rep.ok else rep.summary())

    # per-pixel: board red_mask verdict on every recorded pixel
    n_rec = n_bad = n_bad_rtl = 0
    bad_examples = []
    for i, f in enumerate(frames):
        p = f.pixels
        acc = p["accepted"].astype(bool)
        idx = i * n_px + p["y"][acc] * params.adapter.W + p["x"][acc]
        mod_bit = m.mask[idx].astype(int)
        brd = p["is_red"][acc]
        n_rec += int(acc.sum())
        bad = np.flatnonzero(mod_bit != brd)
        n_bad += len(bad)
        for j in bad[:3]:
            bad_examples.append(f"frame {f.index} ({p['x'][acc][j]},{p['y'][acc][j]}) rgb {p['rgb'][acc][j]:04x}: "
                                f"board {brd[j]} model {mod_bit[j]}")
        if rrec is not None:
            n_bad_rtl += int(np.sum(rrec["P"][idx, 4] != brd))
    add("board red_mask == golden model on every recorded pixel", n_bad == 0,
        f"{n_rec} pixels ({sum(int(f.pixels['band'].sum()) for f in frames)} in dense rows), {n_bad} differ"
        + ("; " + "; ".join(bad_examples) if bad_examples else ""))
    if rrec is not None:
        add("board red_mask == RTL replay on every recorded pixel", n_bad_rtl == 0, f"{n_bad_rtl} differ")

    # per-frame results
    fields = ("valid", "x", "y", "count")
    diffs, diffs_rtl = [], []
    for i, f in enumerate(frames):
        b, mm = f.result, m.results[i]
        if any(b[k] != mm[k] for k in fields):
            diffs.append(f"frame {f.index}: board {[b[k] for k in fields]} model {[mm[k] for k in fields]}")
        if rrec is not None:
            r = rrec["R"][i]
            if [b["valid"], b["x"], b["y"], b["count"]] != [int(r[1]), int(r[2]), int(r[3]), int(r[4])]:
                diffs_rtl.append(f"frame {f.index}")
    add("board frame result == golden model (valid, x, y, red_count)", not diffs,
        f"{len(frames)} frames" + ("; " + "; ".join(diffs) if diffs else ""))
    if rrec is not None:
        add("board frame result == RTL replay", not diffs_rtl, f"{len(frames)} frames" +
            ("; differ: " + ", ".join(diffs_rtl) if diffs_rtl else ""))

    # timing
    lat = sorted({f.result["latency"] for f in complete})
    add(f"board centroid latency == {model.CENTROID_LATENCY} PCLKs after the frame close",
        lat == [model.CENTROID_LATENCY], f"measured {lat}")
    periods = [f.period for f in cap.frames[1:]]
    if periods:
        add("board frame period steady", max(periods) - min(periods) <= 4,
            f"{min(periods)}..{max(periods)} PCLKs = {40e6 / np.mean(periods):.2f} fps at 40 MHz")
    cad = []
    for f in frames:
        p = f.pixels
        d = p["band"].astype(bool) & p["accepted"].astype(bool)
        xs, cy, ys = p["x"][d], p["cyc"][d], p["y"][d]
        same_row = (np.diff(ys) == 0) & (np.diff(xs) == 1)
        cad += list(((np.diff(cy) % 2048))[same_row])
    if cad:
        u = sorted(set(int(c) for c in cad))
        add("board pixel cadence inside a row = 2 PCLKs", u == [2], f"deltas seen {u[:6]}")
    return CaptureReplay(cap, frames, ev, m, rrec, checks)
