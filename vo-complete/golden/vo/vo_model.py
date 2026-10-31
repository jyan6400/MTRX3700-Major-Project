"""Golden model of the beacon visual odometry (Group 26, Wilson, M2).

Bit-exact specification of rtl/vo/:

    beacon_detect     (camera PCLK) led_colour_mask -> colour_run_filter -> per-colour blob:
                      count, coordinate sums, bounding box; after frame_end one shared divider
                      gives the centroid in 1/16 pixel (Q4); gates as led_centroid's
    beacon_geometry   (CLOCK_50) pixel -> robot frame: forward and lateral mm of each LED from
                      its elevation above the camera (known height difference DH_MM) and the
                      camera pitch; with two LEDs, their spacing check and the robot's heading
    beacon_tracker    alpha-beta filter on the selected bay's LED: range, velocity, lateral,
                      displacement since the track started; coasts through short dropouts
    vo_standoff       V4: the robot is at the set distance from the LED (reverse or approach)

Every function here does what the RTL does, integer for integer; golden/vo/vo_check.py runs
the RTL benches and compares them with this file.

GEOMETRY (all exact pinhole maths, no trig at run time):
    camera at height H_CAM, LEDs at H_LED, DH = H_LED - H_CAM > 0
    pitch alpha (camera tilted UP by alpha), focal length f (pixels), principal point (cx, cy)
    for an LED seen at pixel (x, y):    U = x - cx,   V = cy - y          (image y grows down)
        up' = V cos(alpha) + f sin(alpha)       = f * up      / z_cam
        fw' = f cos(alpha) - V sin(alpha)       = f * forward / z_cam
        forward = DH * fw' / up'                lateral = DH * U / up'      (mm, right positive)
    In fixed point: x, y, cx, cy, f in Q4 pixels; sin, cos in Q14.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import model  # noqa: E402  (golden/model.py: led_colour_mask and colour_run_filter)

COLOURS = ("red", "green", "blue")


# ---------------------------------------------------------------- parameters

@dataclass(frozen=True)
class DetectParams:
    RUN_LEN: int = 3            # colour_run_filter
    MIN_PIXELS: int = 6         # an LED 2 m away is a few pixels across
    MAX_PIXELS: int = 6000
    MAX_ASPECT: int = 4
    MIN_FILL: int = 64          # count / box area >= MIN_FILL / 256 (25 %)
    W: int = 640                # a blob touching the frame edge is clipped: its centroid is biased
    H: int = 480
    EDGE_X: int = 2             # columns at each side counted as the edge (the run filter trims and shifts x)
    mask: model.MaskParams = field(default_factory=model.MaskParams)


@dataclass(frozen=True)
class GeomParams:
    F_Q4: int = 560 * 16        # focal length, 1/16 px (vo_calibrate.py measures it)
    CX_Q4: int = 5112           # 319.5 px
    CY_Q4: int = 3832           # 239.5 px
    SIN_Q14: int = 3406         # camera pitch 12 deg up: round(16384 sin)
    COS_Q14: int = 16026        #                         round(16384 cos)
    DH_MM: int = 150            # LED height above the camera lens (450 - 300 mm)
    MAX_RANGE_MM: int = 4000
    SPACING_MM: int = 300       # LED to LED, left to right: red, green, blue
    PAIR_TOL_SHIFT: int = 2     # pair_ok when |d^2 - S^2| <= S^2 >> 2 (about +-12 % in length)


@dataclass(frozen=True)
class TrackParams:
    GATE_MM: int = 200          # a measurement this far from the prediction is an outlier
    MAX_COAST: int = 3          # frames predicted without a measurement before the track drops
    MAX_OUTLIERS: int = 3       # consecutive outliers that restart the track at the measurement
    SETTLE_FRAMES: int = 4      # updates before velocity is reported as settled
    FPS: int = 50               # camera frame rate (velocity = mm/frame * FPS)


@dataclass(frozen=True)
class StandoffParams:
    CONFIRM: int = 2            # consecutive updates at or past the target


def pitch_q14(deg):
    a = np.deg2rad(deg)
    return int(round(16384 * np.sin(a))), int(round(16384 * np.cos(a)))


def geom_params(f_px=560.0, cx=319.5, cy=239.5, pitch_deg=12.0, dh_mm=150, **kw):
    s, c = pitch_q14(pitch_deg)
    return GeomParams(F_Q4=int(round(f_px * 16)), CX_Q4=int(round(cx * 16)), CY_Q4=int(round(cy * 16)),
                      SIN_Q14=s, COS_Q14=c, DH_MM=int(dh_mm), **kw)


def tdiv(n, d):
    """The RTL's signed division: magnitudes divided (floor), sign restored (truncates toward 0)."""
    q = abs(int(n)) // abs(int(d))
    return q if (n >= 0) == (d > 0) else -q


# ---------------------------------------------------------------- beacon_detect

FLAG_TOO_FEW, FLAG_TOO_MANY, FLAG_ASPECT, FLAG_LOW_FILL, FLAG_EDGE = 1, 2, 4, 8, 16
FLAG_NAMES = ("too_few", "too_many", "aspect", "low_fill", "edge")


def blob_result(xs, ys, frame_good, p: DetectParams = DetectParams()):
    """One colour's result from its (filtered) pixel coordinates in one frame."""
    xs = np.asarray(xs, dtype=np.int64)
    ys = np.asarray(ys, dtype=np.int64)
    n = int(len(xs))
    if n:
        x0, x1, y0, y1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
        sx, sy = int(xs.sum()), int(ys.sum())
    else:
        x0 = x1 = y0 = y1 = 0
        sx = sy = 0
    bw, bh = (x1 - x0 + 1, y1 - y0 + 1) if n else (0, 0)
    flags = 0
    if n < p.MIN_PIXELS or n == 0:
        flags |= FLAG_TOO_FEW
    if n > p.MAX_PIXELS:
        flags |= FLAG_TOO_MANY
    if bw > p.MAX_ASPECT * bh or bh > p.MAX_ASPECT * bw:
        flags |= FLAG_ASPECT
    if (n << 8) < p.MIN_FILL * bw * bh:
        flags |= FLAG_LOW_FILL
    if n and (x0 <= p.EDGE_X or y0 == 0 or x1 >= p.W - 1 - p.EDGE_X or y1 >= p.H - 1):
        flags |= FLAG_EDGE
    den = n if n else 1
    xq = ((sx << 4) // den) & 0x3FFF if n else 0
    yq = ((sy << 4) // den) & 0x3FFF if n else 0
    valid = int(bool(frame_good) and flags == 0)
    return dict(valid=valid, x_q4=xq, y_q4=yq, count=min(n, (1 << 19) - 1), flags=flags,
                bx0=x0, bx1=x1, by0=y0, by1=y1)


def detect_frame(frame565, frame_good=True, p: DetectParams = DetectParams()):
    """beacon_detect on one whole raster frame (H, W) of RGB565: [red, green, blue] results.
    A frame the adapter rejects is closed empty (frame_start and frame_end together, no pixel):
    no pixels counted, frame_good 0."""
    if not frame_good:
        return [blob_result([], [], False, p) for _ in range(3)]
    h, w = frame565.shape
    rgb = frame565.reshape(-1)
    cls = np.stack(model.colour_classes(rgb, p.mask), axis=1)
    xs = np.tile(np.arange(w), h)
    ys = np.repeat(np.arange(h), w)
    filt, fx = model.run_filter(xs, cls, p.RUN_LEN)
    out = []
    for k in range(3):
        sel = filt[:, k]
        out.append(blob_result(fx[sel], ys[sel], frame_good, p))
    return out


# ---------------------------------------------------------------- beacon_geometry

def led_geometry(det, g: GeomParams = GeomParams()):
    """One LED: forward and lateral mm (robot frame) from its Q4 centroid. valid = 0 when it was not
    detected, sits at or below the camera's horizon line for that height, or is out of range."""
    if not det["valid"]:
        return dict(valid=0, forward=0, lateral=0)
    U = det["x_q4"] - g.CX_Q4
    V = g.CY_Q4 - det["y_q4"]
    up = V * g.COS_Q14 + g.F_Q4 * g.SIN_Q14
    fw = g.F_Q4 * g.COS_Q14 - V * g.SIN_Q14
    if up <= 0 or fw <= 0:
        return dict(valid=0, forward=0, lateral=0)
    forward = tdiv(g.DH_MM * fw, up)
    lateral = tdiv(g.DH_MM * U * 16384, up)
    if forward > g.MAX_RANGE_MM:
        return dict(valid=0, forward=0, lateral=0)
    lateral = max(-32767, min(32767, lateral))
    return dict(valid=1, forward=forward, lateral=lateral)


def pair_geometry(leds, g: GeomParams = GeomParams()):
    """With two or more LEDs: the widest pair (i < j, j to the right in the world). pair_ok = their
    distance apart matches the known spacing; heading_q12 = tan(heading) * 4096, positive when the
    robot is turned left (counter-clockwise) of square-on to the LED line."""
    vis = [k for k in range(3) if leds[k]["valid"]]
    if len(vis) < 2:
        return dict(pair_valid=0, pair_ok=0, heading_q12=0, pair=0)
    i, j = vis[0], vis[-1]
    dF = leds[j]["forward"] - leds[i]["forward"]
    dL = leds[j]["lateral"] - leds[i]["lateral"]
    S = g.SPACING_MM * (j - i)
    d2 = dF * dF + dL * dL
    S2 = S * S
    ok = int(abs(d2 - S2) <= (S2 >> g.PAIR_TOL_SHIFT))
    if dL <= 0:
        return dict(pair_valid=1, pair_ok=0, heading_q12=0, pair=(i << 2) | j)
    h = tdiv(-dF * 4096, dL)
    h = max(-32767, min(32767, h))
    return dict(pair_valid=1, pair_ok=ok, heading_q12=h, pair=(i << 2) | j)


def geometry(dets, g: GeomParams = GeomParams()):
    leds = [led_geometry(d, g) for d in dets]
    return leds, pair_geometry(leds, g)


# ---------------------------------------------------------------- beacon_tracker

def sat16(x):
    return max(-32768, min(32767, int(x)))


class Tracker:
    """beacon_tracker: one update per geometry result. State in Q4 mm and Q4 mm/frame. The second
    measurement of a track sets the velocity from the two points (no slow ramp up from zero)."""

    def __init__(self, p: TrackParams = TrackParams()):
        self.p = p
        self.sel = 0
        self.tv = 0
        self.r = 0
        self.v = 0
        self.r0 = 0
        self.coast = 0
        self.outl = 0
        self.age = 0
        self.lat = 0

    def _init(self, m, lat):
        self.tv, self.r, self.v, self.r0 = 1, m << 4, 0, m << 4
        self.coast = self.outl = 0
        self.age = 1
        self.lat = lat

    def _coast(self):
        self.r = self.r + self.v
        self.coast += 1
        if self.coast > self.p.MAX_COAST:
            self.tv = 0
            self.age = 0

    def update(self, leds, sel):
        p = self.p
        if sel != self.sel:
            self.tv = 0
            self.age = 0
            self.sel = sel
        m = leds[sel]
        if m["valid"]:
            if not self.tv:
                self._init(m["forward"], m["lateral"])
            else:
                pred = self.r + self.v
                e = (m["forward"] << 4) - pred
                if abs(e) > (p.GATE_MM << 4):
                    self.outl += 1
                    if self.outl >= p.MAX_OUTLIERS:
                        self._init(m["forward"], m["lateral"])
                    else:
                        self._coast()
                else:
                    if self.age == 1:               # second measurement: velocity from the two points
                        self.v = (m["forward"] << 4) - self.r
                        self.r = m["forward"] << 4
                    else:
                        self.r = pred + (e >> 1)
                        self.v = self.v + (e >> 3)
                    self.coast = 0
                    self.outl = 0
                    self.age = min(self.age + 1, 15)
                    self.lat = self.lat + ((m["lateral"] - self.lat) >> 1)
        elif self.tv:
            self._coast()
        return self.outputs()

    def outputs(self):
        tv = self.tv
        return dict(track_valid=tv,
                    settled=int(tv and self.age >= self.p.SETTLE_FRAMES),
                    range_mm=sat16((self.r + 8) >> 4) if tv else 0,
                    vel_mm_s=sat16((self.v * self.p.FPS) >> 4) if tv else 0,
                    lateral_mm=self.lat if tv else 0,
                    disp_mm=sat16((self.r - self.r0 + 8) >> 4) if tv else 0,
                    coast=self.coast if tv else 0)


# ---------------------------------------------------------------- vo_standoff

class Standoff:
    """vo_standoff: `reached` once the track has been at or past target_mm for CONFIRM consecutive
    updates while enabled (approaching = 0: range >= target, reverse away; 1: range <= target).
    It holds until enable drops. lost = enabled and no track."""

    def __init__(self, p: StandoffParams = StandoffParams()):
        self.p = p
        self.n = 0
        self.reached = 0

    def update(self, enable, approaching, target_mm, trk):
        if not enable:
            self.n = 0
            self.reached = 0
            return dict(reached=0, lost=0)
        hit = trk["track_valid"] and ((trk["range_mm"] <= target_mm) if approaching else (trk["range_mm"] >= target_mm))
        self.n = min(self.n + 1, 7) if hit else 0
        if self.n >= self.p.CONFIRM:
            self.reached = 1
        return dict(reached=self.reached, lost=int(not trk["track_valid"]))


# ---------------------------------------------------------------- the whole chain, frame by frame

def run_frames(frames, sels, goods=None, dp=DetectParams(), g=GeomParams(), tp=TrackParams()):
    """frames: iterable of (H, W) RGB565; sels: selected colour per frame. A list of per-frame dicts."""
    trk = Tracker(tp)
    out = []
    for k, f in enumerate(frames):
        good = True if goods is None else bool(goods[k])
        dets = detect_frame(f, good, dp)
        leds, pair = geometry(dets, g)
        t = trk.update(leds, sels[k])
        out.append(dict(det=dets, leds=leds, pair=pair, trk=t))
    return out
