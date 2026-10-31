#!/usr/bin/env python3
"""validate_grab.py -- Group 26 (Wilson, M2): check one JTAG photo from the robot against the golden model.

    python golden/framegrab/validate_grab.py GRAB_DIR [GRAB_DIR ...] [--replay | --no-replay] [--fps 50]

GRAB_DIR is a folder tcl/grab_jtag.tcl + tools/grab.py wrote (tools/take_photo.py does both): frame.hex (the
320 x 240 RGB565 picture the board kept), results.json (the 18 result words vision_grab_results.sv put with
it), scene.hex (the picture for the camera model).

THE CHECKS (one PASS / FAIL / WARN line each, then a summary line):

  integrity   the words are layout 1 (status 0x26 0x01), they belong to the kept frame (live frame number =
              result frame number + 1), both centroids reported, the adapter passed the frame (frame_good) and
              let through all 76,800 kept pixels.
  KEPT        EXACT. The board's second led_centroid only saw the pixels in frame.hex, so the golden model on
              frame.hex (model.frame_result: red_mask -> led_centroid, SW1 from the status word) must give the
              same valid, x, y, count, confidence, flags and bounding box, and the same red-pixel coordinate
              sums. This is the real camera, the real FPGA and the real RTL against model.py.
  FULL        APPROXIMATE (WARN, never FAIL). The robot's own led_centroid saw all 640 x 480 pixels; the photo has
              a quarter of them. Compared with the golden model on the photo pixel-doubled, and with KEPT
              (FULL count about 4 x KEPT, FULL x about 2 x KEPT u). Large differences mean the LED blob is small
              or ragged at the scale of one pixel; the replay checks FULL exactly.
  replay      EXACT (with --replay, default when Verilator is on the PATH). scene.hex through the repo's camera
              model, ov7670_capture and the robot's RTL (sim/vision_grab_replay_tb.sv): every camera pixel must
              be frame.hex pixel-doubled, the replay's KEPT words must equal the board's, and the replay's FULL
              words must equal the golden model on the pixel-doubled frame.
  truth       EXACT, simulated board only (framegrab_sim.py leaves truth.json): the board's FULL words equal the
              golden model on the full-resolution frame the simulated camera really sent.

Writes GRAB_DIR/validation.json and GRAB_DIR/validation.png (the photo, the red mask and the targets).
Exit status 0 when nothing FAILed.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent            # golden/framegrab
GOLDEN = HERE.parent
ROOT = GOLDEN.parent
sys.path.insert(0, str(GOLDEN))
sys.path.insert(0, str(HERE / "vendor" / "tools"))

import model                                      # noqa: E402

LAYOUT = 1
N_WORDS = 18
W, H = 320, 240
KEPT_PIXELS = W * H
RESULT_KEYS = ("valid", "x", "y", "count", "conf", "flags", "bx0", "bx1", "by0", "by1")
FLAG_NAMES = ("too_few", "too_many", "aspect", "low_fill")
BUILD = ROOT / "build" / "framegrab"

# Tolerances for FULL (approximate) against the model on the pixel-doubled photo
TOL_XY = 3                  # pixels, at 640 x 480
TOL_COUNT_REL = 0.35        # relative
TOL_COUNT_ABS = 24          # pixels


# ---------------------------------------------------------------- the words

def _result(w_xy, w_count, w_vf, w_bx, w_by):
    return dict(valid=(w_vf >> 31) & 1, x=(w_xy >> 16) & 0x3FF, y=w_xy & 0x3FF, count=w_count,
                conf=w_vf & 0xFF, flags=(w_vf >> 24) & 0xF,
                bx0=(w_bx >> 16) & 0x3FF, bx1=w_bx & 0x3FF, by0=(w_by >> 16) & 0x3FF, by1=w_by & 0x3FF)


def decode_words(words):
    """The 18 words of vision_grab_results.sv (layout 1) as a dict."""
    w = [int(v) & 0xFFFFFFFF for v in words]
    if len(w) != N_WORDS:
        raise ValueError(f"{len(w)} result words; vision_grab_results.sv gives {N_WORDS}")
    s = w[1]
    return dict(
        live_no=w[0] >> 16, result_no=w[0] & 0xFFFF,
        magic=s >> 24, layout=(s >> 16) & 0xFF,
        last_error=(s >> 11) & 3, stream_ok=(s >> 10) & 1, bypass=(s >> 9) & 1, frame_good=(s >> 8) & 1,
        kept_done=(s >> 3) & 1, full_done=(s >> 2) & 1,
        full=_result(w[2], w[3], w[4], w[5], w[6]),
        kept=_result(w[7], w[8], w[9], w[10], w[11]),
        sum_u=w[12], sum_v=w[13], n_kept=w[14],
        good_frames=w[15] >> 16, bad_frames=w[15] & 0xFFFF, dropped=w[16] & 0xFFFF, period=w[17] & 0xFFFFFF,
        words=w)


def encode_result(r):
    """A result dict -> its five words (2..6 or 7..11), for benches and tests."""
    return [((r["x"] & 0x3FF) << 16) | (r["y"] & 0x3FF), r["count"] & 0xFFFFFFFF,
            (r["valid"] << 31) | ((r["flags"] & 0xF) << 24) | (r["conf"] & 0xFF),
            ((r["bx0"] & 0x3FF) << 16) | (r["bx1"] & 0x3FF), ((r["by0"] & 0x3FF) << 16) | (r["by1"] & 0x3FF)]


def flag_text(f):
    return "+".join(n for k, n in enumerate(FLAG_NAMES) if f >> k & 1) or "none"


def result_text(r, scale=""):
    pos = f"({r['x']},{r['y']})" if r["valid"] else "-"
    return (f"valid={r['valid']} {scale}{pos} count={r['count']} conf={r['conf']} flags={flag_text(r['flags'])} "
            f"box x {r['bx0']}..{r['bx1']} y {r['by0']}..{r['by1']}")


# ---------------------------------------------------------------- the grab and the model

def read_hex16(path, n=None):
    vals = [int(s, 16) for s in open(path).read().split()]
    a = np.array(vals, dtype=np.uint16)
    return a if n is None else a[:n]


def ensure_decoded(d):
    """A grab kept as dump.bin alone (the files worth committing): run grab.py again for frame.hex, scene.hex and
    results.json."""
    d = Path(d)
    if all((d / f).exists() for f in ("frame.hex", "scene.hex", "results.json")) or not (d / "dump.bin").exists():
        return
    import contextlib
    import io
    import grab as vgrab
    hstart, vstart = 160, 11
    if (d / "results.json").exists():
        sc = json.load(open(d / "results.json")).get("scene", {})
        hstart, vstart = int(sc.get("hstart", 160)), int(sc.get("vstart", 11))
    with contextlib.redirect_stdout(io.StringIO()) as buf:
        rc = vgrab.main(["--file", str(d / "dump.bin"), "--out", str(d), "--hstart", str(hstart), "--vstart", str(vstart)])
    if rc != 0:
        raise ValueError(f"{d}/dump.bin: grab.py: {buf.getvalue().strip()}")


def load_grab(d):
    d = Path(d)
    ensure_decoded(d)
    info = json.load(open(d / "results.json"))
    if (info.get("width"), info.get("height"), info.get("pixel_format")) != (W, H, "RGB565"):
        raise ValueError(f"{d}: a {info.get('width')} x {info.get('height')} {info.get('pixel_format')} grab; "
                         f"validation needs SNAP_MODE 0's 320 x 240 RGB565 (vision_grab_top.sv)")
    frame = read_hex16(d / "frame.hex")
    if frame.size != W * H:
        raise ValueError(f"{d}/frame.hex has {frame.size} pixels, not {W * H}")
    return info, frame.reshape(H, W)


def doubled(frame):
    """The 640 x 480 frame the camera model sends for this photo (each stored pixel on a 2 x 2 block)."""
    return np.repeat(np.repeat(frame, 2, axis=0), 2, axis=1)


def model_kept(frame, bypass, p=model.ChainParams()):
    """The golden model's KEPT words for a 320 x 240 photo: led_centroid's result + the red coordinate sums."""
    r = model.frame_result(frame, bool(bypass), p)
    m = model.frame_mask(frame, p.mask)
    ys, xs = np.nonzero(m)
    return r, int(xs.sum()) & 0xFFFFFFFF, int(ys.sum()) & 0xFFFFFFFF, m


def model_full(frame640, bypass, p=model.ChainParams()):
    return model.frame_result(frame640, bool(bypass), p)


# ---------------------------------------------------------------- the report

class Report:
    def __init__(self, name, echo=True):
        self.name = name
        self.echo = echo
        self.lines = []          # (word, check, text)

    def add(self, word, check, text):
        self.lines.append((word, check, text))
        if self.echo:
            print(f"{word}: {self.name}: {check}: {text}", flush=True)

    def ok(self, cond, check, text, fail_word="FAIL"):
        self.add("PASS" if cond else fail_word, check, text)
        return cond

    @property
    def fails(self):
        return sum(1 for w, _, _ in self.lines if w == "FAIL")

    @property
    def warns(self):
        return sum(1 for w, _, _ in self.lines if w == "WARN")


def compare_result(rep, check, got, want, keys=RESULT_KEYS):
    diff = [k for k in keys if int(got[k]) != int(want[k])]
    if not diff:
        rep.add("PASS", check, "equal: " + result_text(got))
        return True
    rep.add("FAIL", check, "differ in " + ", ".join(f"{k} board {got[k]} model {want[k]}" for k in diff)
            + f"; board {result_text(got)}; model {result_text(want)}")
    return False


def full_consistency(rep, full, want, kept):
    """FULL against the model on the pixel-doubled photo and against KEPT: approximate (WARN)."""
    problems = []
    if full["valid"] != want["valid"]:
        problems.append(f"valid board {full['valid']} model {want['valid']} (flags board {flag_text(full['flags'])}, "
                        f"model {flag_text(want['flags'])})")
    elif full["valid"]:
        dx, dy = int(full["x"]) - int(want["x"]), int(full["y"]) - int(want["y"])
        if abs(dx) > TOL_XY or abs(dy) > TOL_XY:
            problems.append(f"target off by ({dx},{dy}) px (tolerance {TOL_XY})")
    dc = int(full["count"]) - int(want["count"])
    if abs(dc) > max(TOL_COUNT_ABS, TOL_COUNT_REL * max(1, want["count"])):
        problems.append(f"count board {full['count']} model {want['count']}")
    text = (f"board {result_text(full)}; model on the doubled photo {result_text(want)}; "
            f"KEPT x 4 = {4 * kept['count']}" + (f", KEPT x 2 = ({2 * kept['x']},{2 * kept['y']})" if kept["valid"] else ""))
    if problems:
        rep.add("WARN", "FULL (approximate)", "; ".join(problems) + ". " + text
                + ". Expected when the blob is a few pixels across: the photo holds a quarter of the pixels; the replay checks FULL exactly")
    else:
        rep.add("PASS", "FULL (approximate)", f"within {TOL_XY} px / {int(TOL_COUNT_REL * 100)} %: " + text)


# ---------------------------------------------------------------- the replay

REPLAY_SOURCES = [
    "vendor/camera/rtl/camera_pkg.sv", "rtl/integration/camera_stream_adapter.sv",
    "rtl/image_processing/led_colour_mask.sv", "rtl/image_processing/red_mask.sv", "rtl/perception/led_centroid.sv",
    "vendor/camera/rtl/ov7670_capture.sv", "golden/framegrab/rtl/vision_grab_results.sv",
    "vendor/camera/tb/ov7670_model.sv", "golden/framegrab/sim/vision_grab_replay_tb.sv",
]


def have_verilator():
    return shutil.which("verilator") is not None and shutil.which("bash") is not None


class build_lock:
    """One build at a time in build/framegrab (run_all_tests.sh runs the framegrab jobs in parallel)."""
    def __enter__(self):
        BUILD.mkdir(parents=True, exist_ok=True)
        self.f = open(BUILD / ".lock", "w")
        try:
            import fcntl
            fcntl.flock(self.f, fcntl.LOCK_EX)
        except ImportError:          # Windows: no Verilator builds there anyway
            pass
        return self

    def __exit__(self, *exc):
        self.f.close()


def build_replay(subst=None, tag=""):
    """Build the replay bench once (build/framegrab/replay<tag>). subst: {repo path: replacement path}."""
    with build_lock():
        return _build_replay(subst, tag)


def _build_replay(subst=None, tag=""):
    out = BUILD / f"replay{tag}"
    exe = out / "Vvision_grab_replay_tb"
    srcs = [str(ROOT / (subst or {}).get(s, s)) for s in REPLAY_SOURCES]
    stamp = out / "sources.txt"
    newest = max(os.path.getmtime(s) for s in srcs)
    if exe.exists() and stamp.exists() and stamp.read_text() == "\n".join(srcs) and os.path.getmtime(exe) > newest:
        return exe
    out.mkdir(parents=True, exist_ok=True)
    cmd = ["bash", str(GOLDEN / "verilator_build.sh"), str(out), "vision_grab_replay_tb",
           "-Wno-fatal", "-Wno-lint", "-Wno-style", "-O1"] + srcs
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
    if r.returncode != 0 or not exe.exists():
        raise RuntimeError("replay build failed:\n" + (r.stdout + r.stderr)[-3000:])
    stamp.write_text("\n".join(srcs))
    return exe


def run_replay(grab_dir, sw1, fps=50, hstart=160, vstart=11, exe=None, out_name="replay_words.hex"):
    """Run the replay bench in grab_dir; returns (pass_line_ok, words or None, log text)."""
    exe = exe or build_replay()
    grab_dir = Path(grab_dir)
    out = grab_dir / out_name
    if out.exists():
        out.unlink()
    r = subprocess.run([str(exe), f"+sw1={int(sw1)}", f"+fps={fps}", f"+hstart={hstart}", f"+vstart={vstart}",
                        f"+out={out_name}"], capture_output=True, text=True, cwd=grab_dir)
    log = r.stdout + r.stderr
    (grab_dir / "replay.log").write_text(log)
    words = [int(s, 16) for s in out.read_text().split()] if out.exists() else None
    return ("PASS: vision_grab_replay_tb" in log), words, log


# ---------------------------------------------------------------- one grab

def validate(grab_dir, replay=None, fps=50, exe=None, plot=True, quiet_name=None, echo=True):
    """Run every check on one grab folder; returns the Report."""
    grab_dir = Path(grab_dir)
    rep = Report(quiet_name or grab_dir.name, echo)
    try:
        info, frame = load_grab(grab_dir)
        dec = decode_words(info["results"])
    except (OSError, ValueError, KeyError) as e:
        rep.add("FAIL", "load", str(e))
        return rep
    sw1 = dec["bypass"]
    rep.add("INFO", "grab", f"{info.get('source', '?')}, CRC {info.get('crc')}, SW1={sw1} "
            f"({'V1 bypass' if sw1 else 'V2 gates'}), adapter good/bad frames {dec['good_frames']}/{dec['bad_frames']}, "
            f"dropped pixels {dec['dropped']}, frame period {dec['period']} PCLKs")

    # integrity
    rep.ok(dec["magic"] == 0x26 and dec["layout"] == LAYOUT, "integrity",
           f"status word 0x{dec['words'][1]:08x}: Group 26 words, layout {dec['layout']}"
           + ("" if dec["magic"] == 0x26 else " -- not vision_grab_top's words (another design, or no frame reported yet)"))
    fresh = (dec["live_no"] - dec["result_no"]) & 0xFFFF
    rep.ok(fresh == 1, "integrity", f"the words are the kept frame's (frame {dec['result_no']}, live {dec['live_no']})"
           if fresh == 1 else f"the words are {fresh} frames old (frame {dec['result_no']}, live {dec['live_no']}): the kept frame got no result")
    rep.ok(dec["full_done"] and dec["kept_done"], "integrity",
           f"both centroids reported together (full_done={dec['full_done']}, kept_done={dec['kept_done']})")
    rep.ok(dec["frame_good"] == 1, "integrity", "the adapter passed the frame (frame_good)" if dec["frame_good"]
           else f"the adapter REJECTED the frame (last_error {dec['last_error']}): its results are an empty close; grab again")
    rep.ok(dec["n_kept"] == KEPT_PIXELS, "integrity", f"{dec['n_kept']} kept pixels through the adapter (expected {KEPT_PIXELS})")

    # KEPT exact
    want_k, su, sv, mask = model_kept(frame, sw1)
    compare_result(rep, "KEPT (exact)", dec["kept"], want_k)
    rep.ok(dec["sum_u"] == su and dec["sum_v"] == sv, "KEPT sums (exact)",
           f"red coordinate sums board ({dec['sum_u']},{dec['sum_v']}) model ({su},{sv}) over {int(mask.sum())} red pixels")

    # FULL approximate
    f640 = doubled(frame)
    want_f = model_full(f640, sw1)
    full_consistency(rep, dec["full"], want_f, dec["kept"])

    # truth (simulated board)
    truth = None
    if (grab_dir / "truth.json").exists():
        truth = json.load(open(grab_dir / "truth.json"))
        tf = read_hex16(grab_dir / truth["frame"]).reshape(480, 640)
        compare_result(rep, "FULL vs the camera's real frame (exact, simulated board)", dec["full"], model_full(tf, sw1))

    # replay
    if replay is None:
        replay = have_verilator()
        if not replay:
            rep.add("INFO", "replay", "skipped: no Verilator on the PATH (run on Ed, or with --replay where it is)")
    rep_words = None
    if replay:
        scene = info.get("scene", {})
        try:
            ok, rep_words, log = run_replay(grab_dir, sw1, fps, int(scene.get("hstart", 160)), int(scene.get("vstart", 11)), exe)
        except RuntimeError as e:
            rep.add("FAIL", "replay", str(e))
            ok, log = False, ""
        if ok:
            rep.add("PASS", "replay pixels", next(l for l in log.splitlines() if l.startswith("replay: window")))
        elif log:
            rep.add("FAIL", "replay pixels", " | ".join(l for l in log.splitlines() if "FAIL" in l)[:600] or log[-600:])
        if rep_words is not None:
            rd = decode_words(rep_words)
            same = [i for i in range(7, 15) if rep_words[i] != dec["words"][i]]
            rep.ok(not same, "replay KEPT == board (exact)",
                   "words 7..14 equal: " + result_text(rd["kept"]) if not same else
                   "words differ: " + ", ".join(f"{i}: replay 0x{rep_words[i]:08x} board 0x{dec['words'][i]:08x}" for i in same))
            compare_result(rep, "replay FULL == model on the doubled photo (exact)", rd["full"], want_f)

    if plot:
        try:
            plot_grab(grab_dir, frame, mask, dec, want_k, want_f, rep)
        except Exception as e:      # plotting is a convenience: never a failure
            rep.add("INFO", "plot", f"validation.png not written ({e})")

    try:
        shown = str(grab_dir.resolve().relative_to(ROOT))
    except ValueError:
        shown = str(grab_dir)
    summary = dict(grab=shown, sw1=sw1, decoded={k: v for k, v in dec.items() if k != "words"},
                   words=[f"0x{v:08x}" for v in dec["words"]], model_kept=want_k, model_full_doubled=want_f,
                   replay_words=None if rep_words is None else [f"0x{v:08x}" for v in rep_words],
                   checks=[dict(result=w, check=c, text=t) for w, c, t in rep.lines],
                   fails=rep.fails, warns=rep.warns)
    with open(grab_dir / "validation.json", "w") as f:
        json.dump(summary, f, indent=2, default=int)
    return rep


def plot_grab(grab_dir, frame, mask, dec, want_k, want_f, rep):
    from matplotlib.figure import Figure          # no pyplot: leaves a notebook's backend alone
    from matplotlib.patches import Rectangle
    import grab as vgrab
    rgb = vgrab.rgb565_to_rgb888(frame)
    fig = Figure(figsize=(12, 4.8))
    ax = fig.subplots(1, 2)
    ax[0].imshow(rgb, interpolation="nearest")
    ax[0].set_title(f"{grab_dir.name}: the photo (320 x 240), SW1={dec['bypass']}")
    k, f = dec["kept"], dec["full"]
    if k["count"]:
        ax[0].add_patch(Rectangle((k["bx0"] - 0.5, k["by0"] - 0.5), k["bx1"] - k["bx0"] + 1, k["by1"] - k["by0"] + 1,
                                  fill=False, ec="yellow", lw=1))
    if k["valid"]:
        ax[0].plot(k["x"], k["y"], "+", color="yellow", ms=16, mew=2, label="board KEPT")
    if want_k["valid"]:
        ax[0].plot(want_k["x"], want_k["y"], "x", color="cyan", ms=11, mew=1.5, label="model KEPT")
    if f["valid"]:
        ax[0].plot(f["x"] / 2, f["y"] / 2, "o", mfc="none", color="lime", ms=14, mew=1.5, label="board FULL / 2 (robot)")
    if ax[0].get_legend_handles_labels()[0]:
        ax[0].legend(loc="lower right", fontsize=8)
    ax[1].imshow(mask, cmap="gray", interpolation="nearest")
    ax[1].set_title(f"red_mask on the photo: {int(mask.sum())} px; board KEPT {k['count']}, FULL {f['count']}")
    verdict = "FAIL" if rep.fails else ("PASS (with warnings)" if rep.warns else "PASS")
    fig.suptitle(f"golden-model validation: {verdict}. KEPT {result_text(k)}", fontsize=9)
    for a in ax:
        a.set_xticks([]); a.set_yticks([])
    fig.tight_layout()
    fig.savefig(grab_dir / "validation.png", dpi=90)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("grabs", nargs="+", help="grab folders (frame.hex, results.json, scene.hex)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--replay", dest="replay", action="store_true", default=None, help="run the RTL replay (needs Verilator)")
    g.add_argument("--no-replay", dest="replay", action="store_false", help="skip the RTL replay")
    ap.add_argument("--fps", type=int, default=50, choices=(25, 50), help="the camera rate the grab was taken at (default 50)")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args(argv)
    exe = None
    if a.replay or (a.replay is None and have_verilator()):
        try:
            exe = build_replay()
        except RuntimeError as e:
            print(f"FAIL: validate_grab: {e}")
            return 1
    fails = warns = 0
    for d in a.grabs:
        rep = validate(d, a.replay, a.fps, exe, plot=not a.no_plot)
        fails += rep.fails
        warns += rep.warns
    word = "PASS" if fails == 0 else "FAIL"
    print(f"{word}: validate_grab: {len(a.grabs)} grab(s), {fails} failed check(s), {warns} warning(s)")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
