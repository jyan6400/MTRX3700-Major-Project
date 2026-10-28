"""Camera-stream stimulus for the golden model and the RTL replay bench.

Everything returns an EVENT_DTYPE array (see model.py): one event per PCLK
cycle on which ov7670_capture drives frame_start and/or valid, stamped with
its cycle. Idle cycles between events are implicit.

Timing presets
--------------
CAMERA  what the OV7670 does with the course register list: a pixel every
        2 PCLKs, 1568 PCLKs per line, 17 blank lines after VSYNC, 510 lines
        per frame -> frame period 799,680 PCLKs = 50.0 fps at 40 MHz.
COMPACT short blanking for fast regression; still >= 64 idle cycles between
        a frame's last pixel and the next frame_start, so results never
        overlap the next frame unless a scenario wants them to.
"""
from __future__ import annotations

import importlib.util
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from model import EVENT_DTYPE, MaskParams, colour_classes

ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class Timing:
    pix: int = 2               # PCLKs per pixel
    line: int = 1568           # PCLKs per line (>= pix * W)
    top_blank_lines: int = 17  # frame_start -> first pixel
    frame_lines: int = 510     # frame_start -> next frame_start, in lines


CAMERA = Timing()
COMPACT = Timing(pix=2, line=0, top_blank_lines=0, frame_lines=0)   # line=0: packed rows


def rgb565(r8, g8, b8):
    r8, g8, b8 = (np.asarray(v, dtype=np.int64) for v in (r8, g8, b8))
    return ((np.clip(r8, 0, 255) >> 3) << 11) | ((np.clip(g8, 0, 255) >> 2) << 5) | (np.clip(b8, 0, 255) >> 3)


def to_rgb888(frame565):
    f = np.asarray(frame565, dtype=np.int64)
    r = ((f >> 11) & 31) * 255 // 31
    g = ((f >> 5) & 63) * 255 // 63
    b = (f & 31) * 255 // 31
    return np.stack([r, g, b], -1).astype(np.uint8)


# ---------------------------------------------------------------- streams


def frames_to_events(frames, sw1=None, timing: Timing = CAMERA, start_cycle: int = 100,
                     lead_pixels: int = 0, tail_gap: int = 2000):
    """Complete raster frames -> camera-stream events.

    frames: list of (H, W) uint16 RGB565 images; sw1: per-frame SW1 (0/1).
    lead_pixels: pixels streamed before the first frame_start (the adapter
    must drop them: a capture that starts mid-frame)."""
    out = []
    c = start_cycle
    H, W = frames[0].shape
    sw1 = list(sw1) if sw1 is not None else [0] * len(frames)
    line = timing.line if timing.line else timing.pix * W + 8
    if lead_pixels:
        e = np.zeros(lead_pixels, dtype=EVENT_DTYPE)
        k = np.arange(lead_pixels)
        e["cycle"] = c + timing.pix * k
        e["valid"] = 1
        e["x"] = (W - lead_pixels + k) % W
        e["y"] = H - 1
        e["rgb"] = 0xF800
        e["sw1"] = sw1[0]
        out.append(e)
        c = int(e["cycle"][-1]) + 64
    for f, s in zip(frames, sw1):
        f = np.asarray(f, dtype=np.uint16)
        fs = np.zeros(1, dtype=EVENT_DTYPE)
        fs["cycle"] = c; fs["fs"] = 1; fs["sw1"] = s
        first = c + max(timing.top_blank_lines * line, 4)
        yy, xx = np.mgrid[0:H, 0:W]
        e = np.zeros(H * W, dtype=EVENT_DTYPE)
        e["cycle"] = (first + yy * line + xx * timing.pix).reshape(-1)
        e["valid"] = 1
        e["rgb"] = f.reshape(-1)
        e["x"] = xx.reshape(-1); e["y"] = yy.reshape(-1); e["sw1"] = s
        out += [fs, e]
        if timing.frame_lines:
            c = c + timing.frame_lines * line
        else:
            c = int(e["cycle"][-1]) + 64
    ev = np.concatenate(out)
    end = np.zeros(1, dtype=EVENT_DTYPE)          # a final idle marker so results flush
    end["cycle"] = int(ev["cycle"][-1]) + tail_gap; end["sw1"] = ev["sw1"][-1]
    return np.concatenate([ev, end])


def stream_frame_slice(ev):
    """Index ranges of each frame (from one fs to the next) in an event array."""
    fsi = list(np.flatnonzero(ev["fs"])) + [len(ev)]
    return [(fsi[i], fsi[i + 1]) for i in range(len(fsi) - 1)]


# ---------------------------------------------------------------- frames


def blank(W=640, H=480, rgb=(30, 40, 30)):
    return np.full((H, W), int(rgb565(*rgb)), dtype=np.uint16)


def draw_disc(f, cx, cy, rad, colour=(250, 30, 25), core=0, core_rgb=(255, 255, 255)):
    H, W = f.shape
    yy, xx = np.mgrid[0:H, 0:W]
    d2 = (xx - cx) ** 2 + (yy - cy) ** 2
    f = f.copy()
    f[d2 <= rad * rad] = int(rgb565(*colour))
    if core:
        f[d2 <= core * core] = int(rgb565(*core_rgb))
    return f


def draw_rect(f, x0, y0, x1, y1, colour=(250, 30, 25)):
    f = f.copy()
    f[y0:y1, x0:x1] = int(rgb565(*colour))
    return f


def speckle(f, n, colour=(250, 30, 25), seed=1):
    rng = np.random.default_rng(seed)
    f = f.copy()
    H, W = f.shape
    f[rng.integers(0, H, n), rng.integers(0, W, n)] = int(rgb565(*colour))
    return f


def all_codes_frame(W=640, H=480):
    """Every RGB565 code once (first 65536 pixels), the rest black."""
    f = np.zeros(H * W, dtype=np.uint16)
    f[:65536] = np.arange(65536, dtype=np.uint16)
    return f.reshape(H, W)


def near_threshold_frame(W=640, H=480, seed=7, p: MaskParams = MaskParams()):
    """Random pixels concentrated within +-2 of every red/green/blue threshold."""
    rng = np.random.default_rng(seed)
    n = W * H
    r = rng.integers(0, 32, n); g6 = rng.integers(0, 64, n); b = rng.integers(0, 32, n)
    pick = rng.integers(0, 6, n)
    r = np.where(pick == 0, p.RED_MIN + rng.integers(-2, 3, n), r)
    g6 = np.where(pick == 1, p.RED_GREEN6_MAX + rng.integers(-2, 3, n), g6)
    b = np.where(pick == 2, p.RED_BLUE_MAX + rng.integers(-2, 3, n), b)
    g6 = np.where(pick == 3, 2 * (r - p.RED_MARGIN) + rng.integers(-3, 4, n), g6)
    b = np.where(pick == 4, r - p.RED_MARGIN + rng.integers(-2, 3, n), b)
    g6 = np.where(pick == 5, p.BLUE_GREEN6_MAX + rng.integers(-2, 3, n), g6)
    r, g6, b = np.clip(r, 0, 31), np.clip(g6, 0, 63), np.clip(b, 0, 31)
    return ((r << 11) | (g6 << 5) | b).astype(np.uint16).reshape(H, W)


# ---------------------------------------------------------------- lab photo


def _make_led_frames_module():
    spec = importlib.util.spec_from_file_location("make_led_frames", ROOT / "sim/tools/make_led_frames.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def photo_frame(leds, noise_seed=26):
    """The lab photo (sim/data/lab_photo.jpg) at 640x480 with LEDs drawn on it.

    leds: list of (colour, cx, cy, radius), colour in red/green/blue. Uses
    sim/tools/make_led_frames.py so the frames match the existing benches."""
    import os
    mod = _make_led_frames_module()
    here = os.getcwd()
    try:
        os.chdir(ROOT)
        base = mod.load_base()
        img = base.copy()
        for led in leds:
            img = mod.draw_led(img, *led)
        return mod.to565(img).astype(np.uint16)
    finally:
        os.chdir(here)


PHOTO_SCENES = {
    "photo_only": [],
    "photo_leds": [("red", 430, 205, 9), ("green", 165, 300, 8), ("blue", 205, 190, 10)],
    "photo_red_left": [("red", 150, 330, 9), ("blue", 205, 190, 10)],
    "photo_green_blue": [("green", 165, 300, 8), ("blue", 205, 190, 10)],
    # two red LEDs far apart: V2 fill gate rejects, SW1 bypass steers to the midpoint
    "photo_two_red": [("red", 430, 205, 9), ("red", 120, 330, 9)],
}


# ---------------------------------------------------------------- scenarios


def synthetic_frames(W=640, H=480):
    """Named frames that exercise every centroid gate and image edge."""
    bg = blank(W, H)
    F = {}
    F["empty"] = bg
    F["disc"] = draw_disc(bg, 300, 200, 12)
    F["ring_core"] = draw_disc(bg, 450, 120, 14, core=5)                   # saturated white core
    F["two_discs"] = draw_disc(draw_disc(bg, 100, 100, 10), 540, 400, 10)  # fill gate
    F["stripe"] = draw_rect(bg, 50, 300, 600, 306)                         # aspect gate
    F["huge"] = draw_rect(bg, 0, 0, 640, 100)                              # MAX_PIXELS gate
    F["tiny"] = draw_rect(bg, 10, 10, 14, 14)                              # MIN_PIXELS gate (16 px)
    F["min_edge"] = draw_rect(bg, 20, 20, 25, 24)                          # exactly 20 px
    F["corners"] = draw_rect(draw_rect(bg, 0, 0, 5, 5), 635, 475, 640, 480)
    F["border_disc"] = draw_disc(bg, 0, 479, 15)
    F["speckled_disc"] = speckle(draw_disc(bg, 320, 240, 12), 300)
    F["green_blue"] = draw_disc(draw_disc(bg, 200, 200, 10, (30, 250, 60)), 400, 300, 10, (30, 60, 250))
    F["all_codes"] = all_codes_frame(W, H)
    F["near_threshold"] = near_threshold_frame(W, H)
    return F


def adapter_edge_cases(W=32, H=24, seed=3):
    """Hand-written stream faults on small frames (bench built -GW=32 -GH=24).

    Returns (name, events) pairs. Each starts from reset, like the bench."""
    rng = np.random.default_rng(seed)

    def frame(colour_seed):
        r = np.random.default_rng(colour_seed)
        f = blank(W, H)
        f = draw_disc(f, int(r.integers(4, W - 4)), int(r.integers(4, H - 4)), 4)
        return f

    def raster(f, c0, sw=0, pix=2, line_gap=6):
        yy, xx = np.mgrid[0:H, 0:W]
        e = np.zeros(H * W, dtype=EVENT_DTYPE)
        e["cycle"] = (c0 + yy * (pix * W + line_gap) + xx * pix).reshape(-1)
        e["valid"] = 1; e["rgb"] = f.reshape(-1); e["x"] = xx.reshape(-1); e["y"] = yy.reshape(-1)
        e["sw1"] = sw
        return e

    def fs(c, sw=0, valid=0):
        e = np.zeros(1, dtype=EVENT_DTYPE)
        e["cycle"] = c; e["fs"] = 1; e["valid"] = valid; e["sw1"] = sw
        return e

    def tail(ev, gap=200):
        end = np.zeros(1, dtype=EVENT_DTYPE)
        end["cycle"] = int(ev["cycle"].max()) + gap
        return np.concatenate([ev, end])

    def seq(*parts):
        ev = np.concatenate(parts)
        ev = ev[np.argsort(ev["cycle"], kind="stable")]
        return tail(ev)

    cases = []
    good = [frame(i) for i in range(6)]
    gap = 2 * W * H + 8 * H + 200

    # 1. pixels before the first frame_start, then two good frames
    pre = raster(good[0], 10)[-50:].copy(); pre["cycle"] -= pre["cycle"][0] - 10
    c = int(pre["cycle"][-1]) + 100
    cases.append(("lock_mid_frame", seq(pre, fs(c), raster(good[1], c + 10), fs(c + gap), raster(good[2], c + gap + 10))))

    # 2. truncated frame: next frame_start arrives early
    a = raster(good[0], 20)[: W * H // 2]
    c2 = int(a["cycle"][-1]) + 50
    cases.append(("truncated", seq(fs(10), a, fs(c2), raster(good[1], c2 + 10), fs(c2 + gap))))

    # 3. coordinate errors: skipped pixel, repeated pixel, x >= W, missing row
    for name, mut in [("skip_pixel", "skip"), ("repeat_pixel", "repeat"), ("x_out_of_range", "xbig"),
                      ("missing_row", "row")]:
        e = raster(good[3], 20)
        k = W * 5 + 7
        if mut == "skip":
            e = np.delete(e, k)
        elif mut == "repeat":
            e = np.insert(e, k, e[k]); e["cycle"][k + 1:] += 2
        elif mut == "xbig":
            e["x"][k] = W + 3
        elif mut == "row":
            e = np.delete(e, np.arange(W * 6, W * 7))
        c3 = int(e["cycle"][-1]) + 100
        cases.append((name, seq(fs(10), e, fs(c3), raster(good[4], c3 + 10), fs(c3 + gap))))

    # 4. extra pixels after a complete frame, and a pixel on the frame_start cycle
    e = raster(good[1], 20)
    extra = e[:5].copy(); extra["cycle"] = int(e["cycle"][-1]) + 2 + 2 * np.arange(5)
    c4 = int(extra["cycle"][-1]) + 100
    cases.append(("extra_pixels_and_fs_valid", seq(fs(10), e, extra, fs(c4, valid=1), raster(good[2], c4 + 10), fs(c4 + gap))))

    # 5. closes closer than the centroid's 34-cycle busy window
    e = raster(good[2], 20)
    c5 = int(e["cycle"][-1])
    cases.append(("close_while_busy", seq(fs(10), e, fs(c5 + 3), fs(c5 + 30), fs(c5 + 34), fs(c5 + 70),
                                          raster(good[5], c5 + 80), fs(c5 + 80 + gap))))

    # 6. SW1 bypass: the same two-blob frame with SW1 off then on
    two = draw_disc(draw_disc(blank(W, H), 5, 5, 3), W - 6, H - 6, 3)
    e0 = raster(two, 20, sw=0)
    c6 = int(e0["cycle"][-1]) + 100
    e1 = raster(two, c6 + 10, sw=1)
    cases.append(("bypass_toggle", seq(fs(10, 0), e0, fs(c6, 1), e1, fs(c6 + gap, 1))))

    # 7. no frame_start at all
    cases.append(("never_synced", seq(raster(good[0], 10)[:200])))
    return cases


def fuzz_stream(seed, W=32, H=24, n_frames=12):
    """Random mix of good and faulty frames, random colours near the thresholds,
    random SW1, random blanking (including closes inside the busy window)."""
    rng = np.random.default_rng(seed)
    parts = []
    c = int(rng.integers(0, 30))
    sw = int(rng.integers(0, 2))
    if rng.random() < 0.5:                                   # start mid-frame
        k = int(rng.integers(1, W * H))
        e = np.zeros(k, dtype=EVENT_DTYPE)
        e["cycle"] = c + 2 * np.arange(k); e["valid"] = 1
        e["x"] = (W * H - k + np.arange(k)) % W; e["y"] = (W * H - k + np.arange(k)) // W
        e["rgb"] = rng.integers(0, 65536, k); e["sw1"] = sw
        parts.append(e); c = int(e["cycle"][-1]) + int(rng.integers(1, 60))
    for _ in range(n_frames):
        if rng.random() < 0.3:
            sw ^= 1
        f = near_threshold_frame(W, H, seed=int(rng.integers(1 << 30)))
        if rng.random() < 0.6:                               # a target blob of random size
            col = [(250, 30, 25), (30, 250, 60), (30, 60, 250)][int(rng.integers(0, 3))]
            f = draw_disc(f, int(rng.integers(0, W)), int(rng.integers(0, H)), int(rng.integers(1, 9)), col)
        fsv = np.zeros(1, dtype=EVENT_DTYPE)
        fsv["cycle"] = c; fsv["fs"] = 1; fsv["sw1"] = sw; fsv["valid"] = int(rng.random() < 0.05)
        parts.append(fsv)
        c += int(rng.integers(1, 20))
        yy, xx = np.mgrid[0:H, 0:W]
        e = np.zeros(H * W, dtype=EVENT_DTYPE)
        step = 1 + int(rng.random() < 0.5)                    # 1 or 2 PCLKs per pixel
        e["cycle"] = (c + (yy * (step * W + int(rng.integers(0, 10))) + xx * step)).reshape(-1)
        e["valid"] = 1; e["rgb"] = f.reshape(-1); e["x"] = xx.reshape(-1); e["y"] = yy.reshape(-1); e["sw1"] = sw
        fault = rng.random()
        if fault < 0.15:                                      # truncate
            e = e[: int(rng.integers(1, len(e)))]
        elif fault < 0.25:                                    # skip
            e = np.delete(e, int(rng.integers(0, len(e))))
        elif fault < 0.32:                                    # repeat
            k = int(rng.integers(0, len(e) - 1)); e = np.insert(e, k, e[k]); e["cycle"][k + 1:] += step
        elif fault < 0.38:                                    # bad coordinate
            k = int(rng.integers(0, len(e))); e["x"][k] = int(rng.integers(0, 1024))
        elif fault < 0.42:                                    # extra pixels
            x = e[: int(rng.integers(1, 20))].copy(); x["cycle"] = int(e["cycle"][-1]) + step * (1 + np.arange(len(x)))
            e = np.concatenate([e, x])
        parts.append(e)
        c = int(e["cycle"][-1]) + int(rng.choice([1, 5, 20, 33, 34, 35, 36, 60, 400]))
    ev = np.concatenate(parts)
    ev = ev[np.argsort(ev["cycle"], kind="stable")]
    # make cycles strictly increasing (a fault may collide two events)
    keep = np.concatenate([[True], np.diff(ev["cycle"]) > 0])
    ev = ev[keep]
    end = np.zeros(1, dtype=EVENT_DTYPE); end["cycle"] = int(ev["cycle"][-1]) + 300; end["sw1"] = sw
    return np.concatenate([ev, end])
