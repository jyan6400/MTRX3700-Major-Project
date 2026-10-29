"""Board captures from vision_tap (via SignalTap) -> replayable frames + board results.

    python golden/capture.py <export> [-o golden/data/captures/<name>]

<export> is what SignalTap saved (File > Export, see golden/README.md): a VCD
(.vcd, tap_word as one vector or as 56 separate bits), a CSV (.csv), or a
.taplog (one hex tap_word per line, as top_capture_tb writes). The output
directory gets the normalised capture (capture.taplog) and a summary
(summary.json); golden/check_golden.py and the notebook then rebuild the
camera stream and replay it through the golden model and the RTL.

Rebuilding a frame. vision_tap records, for every frame, all pixels of a dense
band of rows plus every pixel anywhere that is loosely red (a strict superset
of red_mask) or that the board's red_mask marked red. Every other pixel is
neither red for the board nor for the model, so it is replaced by black
(RGB565 0x0000, also not red): the rebuilt frame gives exactly the frame
result of the real one. Only frames the board's adapter accepted (frame_good)
are rebuilt; rejected frames are checked to have produced an empty result.
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import model                      # noqa: E402
import stimulus                   # noqa: E402
import vlog                       # noqa: E402

TAP_BITS = 56
LOOSE_R_MIN, LOOSE_MARGIN = 16, 4        # must match vision_tap's parameters

# ---------------------------------------------------------------- reading


def read_taplog(path):
    words = []
    for line in open(path):
        s = line.strip()
        if s and not s.startswith("#"):
            words.append(int(s.split()[0], 16))
    return words


def _parse_value(tok: str):
    """A SignalTap/VCD value token -> int (hex, binary or decimal)."""
    t = tok.strip().strip('"').lower().replace("_", "")
    if t.startswith("0x"):
        return int(t[2:], 16)
    if t.startswith("'h") or t.startswith("h"):
        return int(t.lstrip("'h"), 16)
    if t.startswith("'b") or (t.startswith("b") and set(t[1:]) <= {"0", "1"}):
        return int(t.lstrip("'b"), 2)
    if set(t) <= {"0", "1"} and len(t) > 1:
        return int(t, 2)
    if re.fullmatch(r"[0-9a-f]+", t):
        # SignalTap shows buses in hex by default; a pure-decimal-looking
        # value is ambiguous, so hex is assumed (set the bus radix to hex).
        return int(t, 16)
    raise ValueError(f"cannot read value {tok!r}")


def read_vcd(path):
    """Samples of tap_word from a VCD: one sample per time step, kept only where
    tap_store (if present) is 1. tap_word may be one vector or separate bits."""
    ids = {}                         # id -> (name, width, bit or None)
    word_vec = None
    bit_ids = {}
    store_id = None
    text = open(path).read()
    head, _, body = text.partition("$enddefinitions")
    for m in re.finditer(r"\$var\s+\S+\s+(\d+)\s+(\S+)\s+(.*?)\$end", head, re.S):
        width, vid, ref = int(m.group(1)), m.group(2), " ".join(m.group(3).split())
        name = ref.replace(" ", "")
        if "tap_store" in name:
            store_id = vid
        elif "tap_word" in name:
            bm = re.search(r"tap_word\[(\d+)\]$", name)
            if width == 1 and bm:
                bit_ids[vid] = int(bm.group(1))
            elif width > 1:
                word_vec = vid
    if word_vec is None and len(bit_ids) != TAP_BITS:
        raise ValueError(f"{path}: no tap_word vector and {len(bit_ids)} of {TAP_BITS} tap_word bits")
    cur_word, cur_store = 0, 1
    samples = []
    have = False
    pending = None                   # a vector value waiting for its identifier
    for tok in body.split():
        if pending is not None:      # identifier of the vector value just read
            bits = pending.lower().replace("x", "0").replace("z", "0")
            if tok == word_vec:
                cur_word = int(bits, 2)
            elif tok == store_id:
                cur_store = int(bits, 2) & 1
            pending = None
            continue
        if tok[0] == "#" and tok[1:].isdigit():
            if have and cur_store:
                samples.append(cur_word)
            have = True
            continue
        if tok[0] == "$":
            continue
        if tok[0] in "bB":
            pending = tok[1:]
            continue
        if tok[0] in "rR":            # real values: not used here
            pending = "0"
            continue
        if tok[0] in "01xXzZ" and len(tok) > 1:
            v = 1 if tok[0] == "1" else 0
            vid = tok[1:]
            if vid == store_id:
                cur_store = v
            elif vid in bit_ids:
                b = bit_ids[vid]
                cur_word = (cur_word & ~(1 << b)) | (v << b)
    if have and cur_store:
        samples.append(cur_word)
    return samples


def read_csv(path):
    """Samples of tap_word from a CSV export. Accepts tap_word as one column
    (hex/binary) or as one column per bit; rows with tap_store = 0 are dropped."""
    rows = list(csv.reader(open(path, newline="")))
    hdr_i = next((i for i, r in enumerate(rows) if any("tap_word" in c for c in r)), None)
    if hdr_i is None:
        raise ValueError(f"{path}: no column mentions tap_word")
    hdr = [c.strip() for c in rows[hdr_i]]
    vec = [i for i, c in enumerate(hdr) if "tap_word" in c and not re.search(r"tap_word\[(\d+)\]$", c.replace(" ", ""))]
    bits = {i: int(re.search(r"tap_word\[(\d+)\]$", c.replace(" ", "")).group(1))
            for i, c in enumerate(hdr) if re.search(r"tap_word\[(\d+)\]$", c.replace(" ", ""))}
    store = next((i for i, c in enumerate(hdr) if "tap_store" in c), None)
    out = []
    for r in rows[hdr_i + 1:]:
        if len(r) < len(hdr) or not "".join(r).strip():
            continue
        if store is not None and _parse_value(r[store]) == 0:
            continue
        if vec:
            out.append(_parse_value(r[vec[0]]))
        else:
            w = 0
            for i, b in bits.items():
                w |= (_parse_value(r[i]) & 1) << b
            out.append(w)
    return out


def read_export(path):
    path = Path(path)
    suf = path.suffix.lower()
    if suf == ".vcd":
        words = read_vcd(path)
    elif suf == ".csv":
        words = read_csv(path)
    else:
        words = read_taplog(path)
    # A trigger sample can be stored twice; consecutive equal words are one record
    # (seq toggles on every real record, so real records always differ).
    out = [w for i, w in enumerate(words) if i == 0 or w != words[i - 1]]
    return out


# ---------------------------------------------------------------- decoding

PIXEL, FRAME, RESULT = 0, 1, 2


def decode(w: int) -> dict:
    tag = (w >> 54) & 3
    d = dict(tag=tag, seq=(w >> 53) & 1)
    if tag == PIXEL:
        d.update(rgb=(w >> 37) & 0xFFFF, x=(w >> 27) & 0x3FF, y=(w >> 18) & 0x1FF,
                 accepted=(w >> 17) & 1, is_red=(w >> 16) & 1, fs=(w >> 15) & 1,
                 fe=(w >> 14) & 1, good=(w >> 13) & 1, band=(w >> 12) & 1,
                 loose=(w >> 11) & 1, cyc=w & 0x7FF)
    elif tag == FRAME:
        d.update(period=(w >> 29) & 0xFFFFFF, fe=(w >> 28) & 1, good=(w >> 27) & 1,
                 sw1=(w >> 26) & 1, band_y0=(w >> 16) & 0x3FF, cyc=w & 0x7FF)
    elif tag == RESULT:
        d.update(valid=(w >> 52) & 1, x=(w >> 42) & 0x3FF, y=(w >> 32) & 0x3FF,
                 count=(w >> 13) & 0x7FFFF, latency=(w >> 5) & 0xFF, sw1=(w >> 4) & 1)
    return d


@dataclass
class BoardFrame:
    index: int
    period: int                          # PCLKs since previous frame_start (first: meaningless)
    sw1_at_start: int
    band_y0: int
    pixels: dict = field(default_factory=dict)        # column arrays of the PIXEL records
    close: str = "open"                  # good / coordinate / truncated / open (capture ended)
    result: dict | None = None
    dropped_recorded: int = 0

    @property
    def complete(self):
        return self.close != "open" and self.result is not None

    @property
    def good(self):
        return self.close == "good"


@dataclass
class Capture:
    source: str
    words: list
    records: list
    frames: list
    seq_errors: int
    unassigned_results: int

    # ---------- the rebuilt frames
    def good_frames(self):
        return [f for f in self.frames if f.complete and f.good]

    def rebuilt_image(self, f: BoardFrame, W=640, H=480, fill=0x0000):
        img = np.full((H, W), fill, dtype=np.uint16)
        p = f.pixels
        acc = p["accepted"].astype(bool)
        img[p["y"][acc], p["x"][acc]] = p["rgb"][acc]
        return img

    def recorded_mask(self, f: BoardFrame, W=640, H=480):
        """(H, W) int8: -1 not recorded, else the board's is_red bit."""
        m = np.full((H, W), -1, dtype=np.int8)
        p = f.pixels
        acc = p["accepted"].astype(bool)
        m[p["y"][acc], p["x"][acc]] = p["is_red"][acc]
        return m

    def replay_events(self, timing=stimulus.CAMERA):
        """Camera stream of the rebuilt good frames, with the SW1 the board's
        centroid used for each (from its RESULT record)."""
        fr = self.good_frames()
        if not fr:
            return None, []
        imgs = [self.rebuilt_image(f) for f in fr]
        sw = [f.result["sw1"] for f in fr]
        return stimulus.frames_to_events(imgs, sw, timing), fr

    def board_table(self):
        import pandas as pd
        rows = []
        for f in self.frames:
            r = f.result or {}
            p = f.pixels
            rows.append(dict(frame=f.index, close=f.close, complete=f.complete,
                             period_pclk=f.period if f.index else None,
                             recorded_px=int(len(p.get("x", []))),
                             dense_px=int(p["band"].sum()) if len(p.get("x", [])) else 0,
                             board_red_px=int(p["is_red"].sum()) if len(p.get("x", [])) else 0,
                             sw1=r.get("sw1"), valid=r.get("valid"), x=r.get("x"), y=r.get("y"),
                             count=r.get("count"), latency=r.get("latency")))
        return pd.DataFrame(rows)


def parse(words, source="") -> Capture:
    recs = [decode(w) for w in words]
    # seq must toggle on every record; a repeat means SignalTap dropped or doubled one
    seq_errors = sum(1 for a, b in zip(recs, recs[1:]) if a["seq"] == b["seq"])
    frames = []
    cur = None
    pix_rows = []
    last_close = None            # (frame object) most recent close awaiting a result
    unassigned = 0

    def flush(fr, rows):
        if fr is None:
            return
        keys = ("rgb", "x", "y", "accepted", "is_red", "fs", "fe", "good", "band", "loose", "cyc")
        fr.pixels = {k: np.array([r[k] for r in rows], dtype=np.int64) for k in keys}
        fr.dropped_recorded = int(sum(1 for r in rows if not r["accepted"]))

    for r in recs:
        if r["tag"] == FRAME:
            if cur is not None:
                if r["fe"]:
                    cur.close = "truncated"
                    last_close = cur
                flush(cur, pix_rows)
            cur = BoardFrame(len(frames), r["period"], r["sw1"], r["band_y0"])
            frames.append(cur)
            pix_rows = []
        elif r["tag"] == PIXEL:
            if cur is None:
                continue                    # before the trigger: not a frame we can use
            pix_rows.append(r)
            if r["fe"] and cur.close == "open":
                cur.close = "good" if r["good"] else "coordinate"
                last_close = cur
        elif r["tag"] == RESULT:
            if last_close is not None and last_close.result is None:
                last_close.result = r
                last_close = None
            else:
                unassigned += 1
    flush(cur, pix_rows)
    return Capture(source, list(words), recs, frames, seq_errors, unassigned)


def load(path) -> Capture:
    """Read a SignalTap export or a capture directory (with capture.taplog)."""
    p = Path(path)
    if p.is_dir():
        p = p / "capture.taplog"
    return parse(read_export(p), str(path))


# ---------------------------------------------------------------- checks


def loose_is_superset(mp: model.MaskParams = model.MaskParams()):
    """Every RGB565 code the golden red_mask accepts must be 'loose' in
    vision_tap, or unrecorded pixels could hide model-red pixels."""
    codes = np.arange(65536)
    r, g6, g, b = model.split565(codes)
    loose = (r >= LOOSE_R_MIN) & (r >= g + LOOSE_MARGIN) & (r >= b + LOOSE_MARGIN)
    red = model.red_mask(codes, mp)
    return bool(np.all(loose[red])), int(np.sum(red & ~loose))


def save(cap: Capture, out_dir):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "capture.taplog", "w") as f:
        f.write(f"# vision_tap capture from {cap.source}\n")
        f.write("\n".join(f"{w:014x}" for w in cap.words) + "\n")
    fr = cap.good_frames()            # the replay stream is rebuilt on demand (replay.py), not stored
    summ = dict(source=cap.source, samples=len(cap.words), frames=len(cap.frames),
                complete=sum(f.complete for f in cap.frames), good=len(cap.good_frames()),
                seq_errors=cap.seq_errors, unassigned_results=cap.unassigned_results,
                replayed_frames=[f.index for f in fr])
    (out / "summary.json").write_text(json.dumps(summ, indent=2))
    return summ


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("export")
    ap.add_argument("-o", "--out", help="capture directory to write (default: next to the export)")
    a = ap.parse_args(argv)
    cap = load(a.export)
    out = a.out or str(Path(a.export).with_suffix("")) + "_capture"
    summ = save(cap, out)
    print(json.dumps(summ, indent=2))
    print(cap.board_table().to_string(index=False))


if __name__ == "__main__":
    main()
