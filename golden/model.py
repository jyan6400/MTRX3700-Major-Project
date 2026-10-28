"""Bit-exact golden model of the Group 26 vision chain.

    ov7670_capture outputs (the "camera stream")
        -> camera_stream_adapter   frame lock, raster check, one close per frame
        -> red_mask                (led_colour_mask, TARGET_COLOUR = red)
        -> led_centroid            accumulate, gate, divide; SW1 = gate_bypass
    plus, in parallel (Wilson's V3 candidate, not yet in major_project_top):
        -> led_colour_mask (R, G, B) -> colour_run_filter -> 3 x led_centroid

The model is written from the behaviour each module's header and ports
specify, not by copying the RTL register by register:

  * the adapter is a frame-level state machine over input *events*
    (frame_start pulses and valid pixels, each stamped with its PCLK cycle),
    vectorised per frame with numpy;
  * the masks are pure functions of the RGB565 word, vectorised;
  * the centroid is the closed-form frame result (sums, bounding box, floor
    divisions, gates) plus the two timing rules the RTL imposes: the result
    appears CENTROID_LATENCY cycles after the frame close, and a close that
    arrives while the previous result is still being computed produces no
    result.

Cycle numbering (shared with sim/vision_replay_tb.sv and the board capture):
an input event at cycle c is sampled by the adapter on PCLK edge c; anything
registered on that edge is reported "at cycle c". So the adapter's outputs
for input c are at cycle c, the mask (combinational on them) too, and the
centroid's frame_done for a close at cycle e is at e + CENTROID_LATENCY.

Everything here is plain numpy; no simulator is needed to run the model.
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Optional

import numpy as np

# --------------------------------------------------------------------------
# Parameters (defaults = the RTL defaults on main; check_golden.py parses the
# RTL and fails if they drift)
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class MaskParams:
    """led_colour_mask parameters. MIN/MARGIN in 5-bit space (green >> 1);
    the *_GREEN6_MAX ceilings use the full 6-bit green, as V1 did."""
    RED_MIN: int = 20
    RED_GREEN6_MAX: int = 24
    RED_BLUE_MAX: int = 12
    RED_MARGIN: int = 8
    GREEN_MIN: int = 18
    GREEN_RED_MAX: int = 14
    GREEN_BLUE_MAX: int = 20
    GREEN_MARGIN: int = 6
    BLUE_MIN: int = 18
    BLUE_RED_MAX: int = 14
    BLUE_GREEN6_MAX: int = 45
    BLUE_MARGIN: int = 6
    TARGET_COLOUR: int = 0          # red_mask: 0 red, 1 green, 2 blue


@dataclass(frozen=True)
class CentroidParams:
    MIN_PIXELS: int = 20
    MAX_PIXELS: int = 40000
    MAX_ASPECT: int = 4
    MIN_CONFIDENCE: int = 64


@dataclass(frozen=True)
class AdapterParams:
    W: int = 640
    H: int = 480
    PERIOD_W: int = 24


@dataclass(frozen=True)
class ChainParams:
    mask: MaskParams = field(default_factory=MaskParams)
    centroid: CentroidParams = field(default_factory=CentroidParams)
    adapter: AdapterParams = field(default_factory=AdapterParams)
    RUN_LEN: int = 3                # colour_run_filter (tri-colour path)


# led_centroid: frame_end registered by the adapter on edge e is seen by the
# centroid on e+1 (IDLE -> BOX), BOX e+2, GATE e+3 (load divider), 32 divide
# steps e+4 .. e+35, frame_done registered on e+35.
CENTROID_LATENCY = 35
# A close whose adapter edge f satisfies 1 <= f - e <= BUSY_WINDOW after an
# accepted close e is ignored (the centroid is not IDLE when it sees it).
BUSY_WINDOW = 34

# Record type for the replay stimulus / camera stream.
EVENT_DTYPE = np.dtype([("cycle", np.int64), ("fs", np.uint8), ("valid", np.uint8),
                        ("rgb", np.uint16), ("x", np.uint16), ("y", np.uint16),
                        ("sw1", np.uint8)])


def make_events(n: int) -> np.ndarray:
    return np.zeros(n, dtype=EVENT_DTYPE)


# --------------------------------------------------------------------------
# Pixel classification: led_colour_mask / red_mask
# --------------------------------------------------------------------------


def split565(rgb):
    """RGB565 word(s) -> (R5, G6, G5, B5) as int arrays."""
    rgb = np.asarray(rgb, dtype=np.int64)
    r = (rgb >> 11) & 0x1F
    g6 = (rgb >> 5) & 0x3F
    b = rgb & 0x1F
    return r, g6, g6 >> 1, b


def colour_classes(rgb, p: MaskParams = MaskParams()):
    """led_colour_mask: (is_red, is_green, is_blue) boolean arrays.

    A colour is accepted when its channel reaches MIN, the other two stay at or
    below their ceilings, and it beats both of them by at least MARGIN. With
    every MARGIN >= 1 at most one colour can be true per pixel."""
    r, g6, g, b = split565(rgb)
    red = ((r >= p.RED_MIN) & (g6 <= p.RED_GREEN6_MAX) & (b <= p.RED_BLUE_MAX)
           & (r >= g + p.RED_MARGIN) & (r >= b + p.RED_MARGIN))
    green = ((g >= p.GREEN_MIN) & (r <= p.GREEN_RED_MAX) & (b <= p.GREEN_BLUE_MAX)
             & (g >= r + p.GREEN_MARGIN) & (g >= b + p.GREEN_MARGIN))
    blue = ((b >= p.BLUE_MIN) & (r <= p.BLUE_RED_MAX) & (g6 <= p.BLUE_GREEN6_MAX)
            & (b >= r + p.BLUE_MARGIN) & (b >= g + p.BLUE_MARGIN))
    return red, green, blue


def red_mask(rgb, p: MaskParams = MaskParams()):
    """red_mask.is_red: the TARGET_COLOUR classification."""
    cls = colour_classes(rgb, p)
    return cls[p.TARGET_COLOUR if p.TARGET_COLOUR in (1, 2) else 0]


def red_mask_v1(rgb):
    """The V1 red_mask, kept as an independent reference: V2 must equal it."""
    r, g6, g, b = split565(rgb)
    return ((r >= 20) & (g6 <= 24) & (b <= 12) & (r >= g + 8) & (r >= b + 8))


# --------------------------------------------------------------------------
# colour_run_filter (horizontal run-length speckle filter)
# --------------------------------------------------------------------------


def run_filter(x, bits, run_len: int = 3):
    """Filtered bits and filtered mask_x for a sequence of accepted pixels.

    x:    (N,) pixel_x of each accepted pixel, in stream order since reset
    bits: (N,) or (N, k) boolean raw classifications
    A pixel is kept when it and the previous run_len-1 pixels of the same row
    segment are all set. A segment starts at each x == 0 pixel (and at reset);
    its first run_len-1 pixels can never pass. mask_x = x - (run_len-1)//2,
    clamped at 0, cancels the left-edge trim bias."""
    x = np.asarray(x, dtype=np.int64)
    bits = np.asarray(bits, dtype=bool)
    n = len(x)
    comp = (run_len - 1) // 2
    fx = np.where(x >= comp, x - comp, 0)
    if run_len <= 1 or n == 0:
        return bits.copy(), fx
    idx = np.arange(n)
    seg_start = np.where(x == 0, idx, 0)
    seg_start = np.maximum.accumulate(seg_start)         # most recent x==0 (or 0)
    ok = (idx - seg_start) >= (run_len - 1)
    out = bits.copy()
    for k in range(1, run_len):
        prev = np.zeros_like(bits)
        prev[k:] = bits[:-k]
        out &= prev
    if out.ndim == 2:
        out &= ok[:, None]
    else:
        out &= ok
    return out, fx


# --------------------------------------------------------------------------
# camera_stream_adapter
# --------------------------------------------------------------------------


@dataclass
class AdapterOut:
    # accepted pixels (pixel_valid outputs)
    p_cycle: np.ndarray
    p_x: np.ndarray
    p_y: np.ndarray
    p_rgb: np.ndarray
    # frame_start outputs (cycles), frame_end outputs (cycle, good)
    s_cycle: list
    e_cycle: list
    e_good: list
    # diagnostics after each S/E edge and at the end:
    # (cycle, stream_ok, last_error, good_frames, bad_frames, dropped, frame_period)
    diag: list
    final: tuple
    closes: list            # (cycle, kind) kind in good/truncated/coordinate


def camera_stream_adapter(ev: np.ndarray, p: AdapterParams = AdapterParams()) -> AdapterOut:
    """Frame-integrity adapter. ev: EVENT_DTYPE array sorted by cycle.

    Behaviour (from the module header):
      lock      pixels are dropped until the first frame_start after reset;
      check     each pixel must be at the next raster position (exp_x, exp_y);
      close     every frame exactly once: good on its last pixel (W-1, H-1)
                with frame_good; a coordinate error is an EMPTY CLOSE on the
                offending pixel (frame_start = frame_end = 1, pixel dropped);
                a frame_start before the last pixel closes the open frame in
                the same cycle (TRUNCATED);
      drop      pixels outside an open frame, and a pixel coinciding with
                frame_start, are dropped and counted (saturating 16-bit);
      diag      good/bad frame counters (16-bit wrap), last_error
                {TRUNCATED, COORDINATE}, stream_ok, frame_period (cycles
                between frame_starts, saturating at 2^PERIOD_W - 1).
    Outputs are registered: they appear at the input's cycle (see module doc)."""
    W, H = p.W, p.H
    NPIX = W * H
    PMAX = (1 << p.PERIOD_W) - 1

    cyc = ev["cycle"].astype(np.int64)
    fs = ev["fs"].astype(bool)
    val = ev["valid"].astype(bool)
    ex = ev["x"].astype(np.int64) & 0x3FF
    ey = ev["y"].astype(np.int64) & 0x1FF

    acc_idx = []                 # indices of accepted events
    s_cycle, e_cycle, e_good, diag, closes = [], [], [], [], []

    synced = False
    open_frame = False
    pos = 0                      # expected raster index exp_y*W + exp_x
    good = bad = 0
    dropped = 0
    last_error = 0
    stream_ok = 0
    frame_period = 0
    last_fs_cycle = None

    def snap(c):
        diag.append((int(c), stream_ok, last_error, good, bad, dropped, frame_period))

    fs_idx = np.flatnonzero(fs)
    # blocks of non-fs valid events between frame_starts
    bounds = [0] + list(fs_idx) + [len(ev)]
    # process: block before first fs, then for each fs: the fs event, then block
    def block(lo, hi):
        nonlocal open_frame, pos, good, bad, dropped, last_error, stream_ok
        sel = np.arange(lo, hi)
        sel = sel[val[lo:hi]]                       # pixels only (fs excluded by construction)
        n = len(sel)
        if n == 0:
            return
        if not open_frame:
            dropped = min(dropped + n, 0xFFFF)
            return
        k = np.arange(n)
        expect = pos + k
        okc = (expect < NPIX) & (ex[sel] == (expect % W)) & (ey[sel] == (expect // W))
        bad_at = np.flatnonzero(~okc)
        first_bad = int(bad_at[0]) if len(bad_at) else n
        close_at = NPIX - 1 - pos                    # index within block of the last pixel
        if close_at < first_bad and close_at < n:
            # good close
            acc_idx.extend(sel[:close_at + 1].tolist())
            c = cyc[sel[close_at]]
            e_cycle.append(int(c)); e_good.append(1)
            good = (good + 1) & 0xFFFF
            last_error = 0; stream_ok = 1; open_frame = False
            snap(c)
            closes.append((int(c), "good"))
            rest = n - close_at - 1
            dropped = min(dropped + rest, 0xFFFF)
            pos = 0
        elif first_bad < n:
            acc_idx.extend(sel[:first_bad].tolist())
            c = cyc[sel[first_bad]]
            s_cycle.append(int(c)); e_cycle.append(int(c)); e_good.append(0)
            bad = (bad + 1) & 0xFFFF
            last_error = 1; stream_ok = 0; open_frame = False
            dropped = min(dropped + 1, 0xFFFF)
            snap(c)
            closes.append((int(c), "coordinate"))
            rest = n - first_bad - 1
            dropped = min(dropped + rest, 0xFFFF)
        else:
            acc_idx.extend(sel.tolist())
            pos += n

    block(bounds[0], bounds[1] if len(fs_idx) else len(ev))
    for j, i in enumerate(fs_idx):
        c = int(cyc[i])
        if synced:
            frame_period = min(c - last_fs_cycle, PMAX)
        last_fs_cycle = c
        s_cycle.append(c)
        if open_frame:
            e_cycle.append(c); e_good.append(0)
            bad = (bad + 1) & 0xFFFF
            last_error = 2; stream_ok = 0
            closes.append((c, "truncated"))
        if val[i]:
            dropped = min(dropped + 1, 0xFFFF)
        synced = True
        open_frame = True
        pos = 0
        snap(c)
        block(i + 1, bounds[j + 2])

    acc = np.array(acc_idx, dtype=np.int64)
    last_cycle = int(cyc[-1]) if len(ev) else 0
    return AdapterOut(
        p_cycle=cyc[acc], p_x=ex[acc], p_y=ey[acc], p_rgb=ev["rgb"][acc].astype(np.int64),
        s_cycle=s_cycle, e_cycle=e_cycle, e_good=e_good,
        diag=diag, final=(last_cycle, stream_ok, last_error, good, bad, dropped, frame_period),
        closes=closes)


# --------------------------------------------------------------------------
# led_centroid
# --------------------------------------------------------------------------

RESULT_FIELDS = ("cycle", "valid", "x", "y", "count", "conf", "flags",
                 "bx0", "bx1", "by0", "by1")


def centroid_result(xs, ys, bypass: bool, p: CentroidParams = CentroidParams()):
    """led_centroid's result for one frame's target pixels (x, y arrays).

    count, bounding box, box area; gates
        flags[0] too few   count < MIN_PIXELS (or 0)
        flags[1] too many  count > MAX_PIXELS
        flags[2] aspect    box_w > MAX_ASPECT*box_h or box_h > MAX_ASPECT*box_w
        low_fill           confidence < MIN_CONFIDENCE,
                           confidence = min(255, floor(256*count / area))
    accept = !flags[0] in bypass (V1), else no flag and no low_fill.
    target = floor(sum/count), reported only when accepted."""
    xs = np.asarray(xs, dtype=np.int64)
    ys = np.asarray(ys, dtype=np.int64)
    n = int(len(xs))
    if n:
        x0, x1, y0, y1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
        bw, bh = x1 - x0 + 1, y1 - y0 + 1
        sx, sy = int(xs.sum()) & 0xFFFFFFFF, int(ys.sum()) & 0xFFFFFFFF
    else:
        x0 = x1 = y0 = y1 = 0
        bw = bh = 0
        sx = sy = 0
    area = bw * bh
    f0 = (n < p.MIN_PIXELS) or n == 0
    f1 = n > p.MAX_PIXELS
    w_limit = (bh * (p.MAX_ASPECT & 0x1F)) & 0xFFFF
    h_limit = (bw * (p.MAX_ASPECT & 0x1F)) & 0xFFFF
    f2 = (bw > w_limit) or (bh > h_limit)
    den = n if n else 1
    dvd_c = 0xFFFFFF00 if n > 0xFFFFFF else ((n & 0xFFFFFF) << 8)
    conf = min(255, dvd_c // (area if area else 1))
    low_fill = conf < p.MIN_CONFIDENCE
    accept = (not f0) if bypass else (not (f0 or f1 or f2) and not low_fill)
    qx, qy = sx // den, sy // den
    flags = (int(low_fill) << 3) | (int(f2) << 2) | (int(f1) << 1) | int(f0)
    return dict(valid=int(accept),
                x=(qx & 0x3FF) if accept else 0,
                y=(qy & 0x3FF) if accept else 0,
                count=n, conf=int(conf), flags=flags,
                bx0=x0, bx1=x1, by0=y0, by1=y1)


def led_centroid(s_cycle, e_cycle, p_cycle, p_x, p_y, p_bit, bypass_at,
                 p: CentroidParams = CentroidParams()):
    """Frame results from an adapter output stream.

    Accumulation restarts after every frame_end and is cleared by frame_start
    (the frame_start cycle's own pixel is counted). A frame_end produces a
    result CENTROID_LATENCY cycles later unless it falls inside the previous
    accepted close's BUSY_WINDOW. bypass_at(cycle) -> SW1 at that cycle."""
    p_cycle = np.asarray(p_cycle, dtype=np.int64)
    sel = np.asarray(p_bit, dtype=bool)
    tc, tx, ty = p_cycle[sel], np.asarray(p_x)[sel], np.asarray(p_y)[sel]
    s = np.asarray(sorted(set(s_cycle)), dtype=np.int64)
    out = []
    last_e = None           # previous frame_end edge (accumulator reset)
    busy_from = None        # accepted close edge
    for e in sorted(set(e_cycle)):
        # boundary: after previous frame_end, or at (including) last frame_start <= e
        lo_excl = last_e if last_e is not None else -1
        s_le = s[s <= e]
        if len(s_le) and s_le[-1] > lo_excl:
            lo, incl = int(s_le[-1]), True
        else:
            lo, incl = lo_excl, False
        a = np.searchsorted(tc, lo, side="left" if incl else "right")
        b = np.searchsorted(tc, e, side="right")
        last_e = e
        if busy_from is not None and 1 <= e - busy_from <= BUSY_WINDOW:
            continue        # centroid busy: close ignored, no result
        busy_from = e
        rc = e + CENTROID_LATENCY
        r = centroid_result(tx[a:b], ty[a:b], bool(bypass_at(rc)), p)
        r["cycle"] = rc
        out.append(r)
    return out


# --------------------------------------------------------------------------
# The whole chain
# --------------------------------------------------------------------------


@dataclass
class ChainResult:
    adapter: AdapterOut
    mask: np.ndarray            # red_mask bit per accepted pixel
    lcm: np.ndarray             # (N, 3) led_colour_mask bits
    filt: np.ndarray            # (N, 3) colour_run_filter bits
    fx: np.ndarray              # filtered mask_x
    results: list               # red_mask -> led_centroid (in major_project_top)
    tri: list                   # [(colour, result dict)] from tri_led_detect

    def records(self):
        """All observable outputs as text records (see golden/vlog.py)."""
        from vlog import chain_records
        return chain_records(self)


def bypass_lookup(ev: np.ndarray):
    """SW1 seen by led_centroid at a cycle: the last event's sw1 at or before it."""
    cyc = ev["cycle"].astype(np.int64)
    sw = ev["sw1"].astype(np.int64)

    def at(c):
        i = np.searchsorted(cyc, c, side="right") - 1
        return int(sw[i]) if i >= 0 else 0
    return at


def run_chain(ev: np.ndarray, p: ChainParams = ChainParams(), tri: bool = True) -> ChainResult:
    """Run the golden model of the vision chain on a camera-stream event list."""
    ad = camera_stream_adapter(ev, p.adapter)
    lcm = np.stack(colour_classes(ad.p_rgb, p.mask), axis=1) if len(ad.p_rgb) else np.zeros((0, 3), bool)
    mask = red_mask(ad.p_rgb, p.mask) if len(ad.p_rgb) else np.zeros(0, bool)
    bypass = bypass_lookup(ev)
    results = led_centroid(ad.s_cycle, ad.e_cycle, ad.p_cycle, ad.p_x, ad.p_y, mask, bypass, p.centroid)
    filt, fx = run_filter(ad.p_x, lcm, p.RUN_LEN) if len(ad.p_x) else (np.zeros((0, 3), bool), np.zeros(0, np.int64))
    tri_out = []
    if tri:
        per = [led_centroid(ad.s_cycle, ad.e_cycle, ad.p_cycle, fx, ad.p_y, filt[:, k], bypass, p.centroid)
               for k in range(3)]
        for rs in zip(*per):
            for k, r in enumerate(rs):
                tri_out.append((k, r))
    return ChainResult(ad, mask, lcm, filt, fx, results, tri_out)


# --------------------------------------------------------------------------
# Frame-level convenience (pictures in, result out) for the notebook
# --------------------------------------------------------------------------


def frame_mask(frame_rgb565: np.ndarray, p: MaskParams = MaskParams()) -> np.ndarray:
    """(H, W) RGB565 image -> (H, W) red_mask image."""
    return red_mask(frame_rgb565.reshape(-1), p).reshape(frame_rgb565.shape)


def frame_result(frame_rgb565: np.ndarray, bypass: bool = False, p: ChainParams = ChainParams()):
    """Centroid result for one complete, good frame (the common case)."""
    m = frame_mask(frame_rgb565, p.mask)
    ys, xs = np.nonzero(m)
    return centroid_result(xs, ys, bypass, p.centroid)
