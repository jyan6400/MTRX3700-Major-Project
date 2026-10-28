"""File formats shared by the golden model, the RTL replay bench and board captures.

STIMULUS (.stim): the camera stream fed to camera_stream_adapter, one event
per line as a 64-bit hex word (comment lines start with '#'):

    [63:40] delta   cycles since the previous event (first event: its cycle)
    [39]    sw1     SW1 (led_centroid gate_bypass) from this cycle on
    [38]    fs      cam_frame_start
    [37]    valid   cam_valid
    [36:21] rgb     cam_pixel (RGB565)
    [20:11] x       cam_x
    [10:2]  y       cam_y (9 bits)

RECORDS (.rec): every observable output, one per line, decimal fields. The
model (golden/model.py), the RTL (golden/sim/vision_replay_tb.sv) and a board
capture (golden/capture.py) all write this format so they compare directly:

    P cyc x y rgb mask lcm filt fx   adapter pixel_valid out, red_mask bit,
                                     led_colour_mask bits, colour_run_filter
                                     bits (r<<2|g<<1|b) and filtered mask_x
    S cyc                            adapter frame_start out
    E cyc good                       adapter frame_end out, frame_good
    D cyc ok err good bad drop per   adapter diagnostics after an S/E edge
    Z cyc ok err good bad drop per   adapter diagnostics at the end of the run
    R cyc valid x y count conf flags bx0 bx1 by0 by1
                                     led_centroid frame_done (the robot's path)
    T cyc col valid x y count conf flags bx0 bx1 by0 by1
                                     tri_led_detect frame_done, col 0/1/2 = R/G/B

A board capture only contains the fields the board exposes; missing fields
are written as -1 and are not compared.
"""
from __future__ import annotations

from collections import defaultdict
from pathlib import Path

import numpy as np

from model import EVENT_DTYPE, RESULT_FIELDS

REC_FIELDS = {
    "P": ("cycle", "x", "y", "rgb", "mask", "lcm", "filt", "fx"),
    "S": ("cycle",),
    "E": ("cycle", "good"),
    "D": ("cycle", "ok", "err", "good", "bad", "drop", "period"),
    "Z": ("cycle", "ok", "err", "good", "bad", "drop", "period"),
    "R": RESULT_FIELDS,
    "T": ("cycle", "col") + RESULT_FIELDS[1:],
}
REC_ORDER = "SPEDRTZ"

# ---------------------------------------------------------------- stimulus


def pack_events(ev: np.ndarray) -> np.ndarray:
    cyc = ev["cycle"].astype(np.int64)
    if len(cyc) and (np.any(np.diff(cyc) < 1) or cyc[0] < 0):
        raise ValueError("events must have strictly increasing, non-negative cycles")
    delta = np.diff(cyc, prepend=0)
    if len(cyc):
        delta[0] = cyc[0]
    if np.any(delta >= (1 << 24)):
        raise ValueError("gap between events exceeds 2^24 cycles")
    w = (delta.astype(np.uint64) << np.uint64(40))
    w |= ev["sw1"].astype(np.uint64) << np.uint64(39)
    w |= ev["fs"].astype(np.uint64) << np.uint64(38)
    w |= ev["valid"].astype(np.uint64) << np.uint64(37)
    w |= ev["rgb"].astype(np.uint64) << np.uint64(21)
    w |= (ev["x"].astype(np.uint64) & np.uint64(0x3FF)) << np.uint64(11)
    w |= (ev["y"].astype(np.uint64) & np.uint64(0x1FF)) << np.uint64(2)
    return w


def write_stim(path, ev: np.ndarray, comment: str = ""):
    w = pack_events(ev)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        for line in comment.splitlines():
            f.write(f"# {line}\n")
        f.write("\n".join(f"{int(v):016x}" for v in w))
        f.write("\n")


def read_stim(path) -> np.ndarray:
    words = []
    for line in open(path):
        s = line.strip()
        if s and not s.startswith("#"):
            words.append(int(s, 16))
    w = np.array(words, dtype=np.uint64)
    ev = np.zeros(len(w), dtype=EVENT_DTYPE)
    delta = (w >> np.uint64(40)).astype(np.int64)
    ev["cycle"] = np.cumsum(delta)
    ev["sw1"] = (w >> np.uint64(39)) & np.uint64(1)
    ev["fs"] = (w >> np.uint64(38)) & np.uint64(1)
    ev["valid"] = (w >> np.uint64(37)) & np.uint64(1)
    ev["rgb"] = (w >> np.uint64(21)) & np.uint64(0xFFFF)
    ev["x"] = (w >> np.uint64(11)) & np.uint64(0x3FF)
    ev["y"] = (w >> np.uint64(2)) & np.uint64(0x1FF)
    return ev


# ---------------------------------------------------------------- records


class Records:
    """Output records grouped by type; each type is an (N, nfields) int64 array."""

    def __init__(self, rows=None):
        self.t = {k: np.zeros((0, len(v)), dtype=np.int64) for k, v in REC_FIELDS.items()}
        if rows is not None:
            grouped = defaultdict(list)
            for k, vals in rows:
                grouped[k].append(vals)
            for k, lst in grouped.items():
                self.t[k] = np.array(lst, dtype=np.int64).reshape(-1, len(REC_FIELDS[k]))

    def __getitem__(self, k):
        return self.t[k]

    def field(self, k, name):
        return self.t[k][:, REC_FIELDS[k].index(name)]

    def count(self):
        return {k: len(v) for k, v in self.t.items()}

    def write(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        lines = []
        for k in REC_ORDER:
            for row in self.t[k]:
                lines.append((int(row[0]), REC_ORDER.index(k), k + " " + " ".join(str(int(v)) for v in row)))
        lines.sort(key=lambda r: (r[0], r[1]))
        with open(path, "w") as f:
            f.write("\n".join(l[2] for l in lines) + "\n")


def read_records(path) -> Records:
    """Read a .rec file (fast path through pandas' C parser when available)."""
    rec = Records()
    width = max(len(v) for v in REC_FIELDS.values())
    try:
        import pandas as pd
        df = pd.read_csv(path, sep=" ", header=None, names=["t"] + list(range(width)),
                         comment="#", dtype={"t": str}, engine="c")
        for k, names in REC_FIELDS.items():
            sub = df[df["t"] == k]
            if len(sub):
                rec.t[k] = sub[list(range(len(names)))].to_numpy(dtype=np.int64)
        return rec
    except ImportError:
        pass
    grouped = defaultdict(list)
    for line in open(path):
        s = line.split()
        if not s or s[0] not in REC_FIELDS:
            continue
        grouped[s[0]].append([int(v) for v in s[1:]])
    for k, lst in grouped.items():
        rec.t[k] = np.array(lst, dtype=np.int64).reshape(-1, len(REC_FIELDS[k]))
    return rec


def _bits3(b):
    b = np.asarray(b, dtype=np.int64)
    return (b[:, 0] << 2) | (b[:, 1] << 1) | b[:, 2]


def chain_records(cr) -> Records:
    """Records for a model.ChainResult."""
    ad = cr.adapter
    rec = Records()
    n = len(ad.p_cycle)
    if n:
        rec.t["P"] = np.stack([ad.p_cycle, ad.p_x, ad.p_y, ad.p_rgb,
                               cr.mask.astype(np.int64), _bits3(cr.lcm), _bits3(cr.filt),
                               cr.fx.astype(np.int64)], axis=1)
    rec.t["S"] = np.array(sorted(set(ad.s_cycle)), dtype=np.int64).reshape(-1, 1)
    rec.t["E"] = np.array(list(zip(ad.e_cycle, ad.e_good)), dtype=np.int64).reshape(-1, 2)
    rec.t["D"] = np.array(ad.diag, dtype=np.int64).reshape(-1, 7)
    rec.t["Z"] = np.array([ad.final], dtype=np.int64).reshape(-1, 7)
    rec.t["R"] = np.array([[r[f] for f in RESULT_FIELDS] for r in cr.results], dtype=np.int64).reshape(-1, len(RESULT_FIELDS))
    rec.t["T"] = np.array([[r["cycle"], k] + [r[f] for f in RESULT_FIELDS[1:]] for k, r in cr.tri],
                          dtype=np.int64).reshape(-1, len(RESULT_FIELDS) + 1)
    return rec
