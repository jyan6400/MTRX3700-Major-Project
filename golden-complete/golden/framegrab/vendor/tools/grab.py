#!/usr/bin/env python3
"""grab.py -- check one frame grab (rtl/frame_snap.sv's memory, which tcl/grab_jtag.tcl reads over JTAG into
dump.bin) and turn it into files.

    python3 grab.py --file dump.bin --out grab1     tcl/grab_jtag.tcl runs this itself when it finds Python

Options: --out DIR (default .), and the camera geometry for the scene file: --mvfp 0x37 --hstart 160 --vstart 11
(the camera code's registers: ov7670_config.sv).

Writes into DIR:
    frame.png      the W x H picture as the frame buffer held it (RGB565 widened to 8 bits a channel)
    frame.hex      the same pixels for $readmemh: one 16-bit RGB565 word per line (4 hex digits), row-major
    scene.hex      the picture as a scene for the camera model, in exactly the form tools/make_scene.py
                   writes: 656 x 488 24-bit RGB words (6 hex digits a line), row-major, one per sensor pixel.
                   ov7670_model #(.SCENE_FILE("scene.hex")) then sends this frame again, every stored pixel
                   exactly (see "the scene" below)
    results.json   the header, the result words (word 0 first), the CRC, where it came from

Exit status 0 = a whole dump with a good CRC; 1 = no dump, a short dump or a bad CRC (nothing written
except dump.bin); 2 = a usage problem (a missing file).

PIXEL FORMATS. 1 = RGB565 (frame_snap.sv's MODE 0 and 1); 2 = 8-bit grey, two pixels a 16-bit
word, the left one first (frame_snap.sv's MODE 2). A grey frame's frame.png is a grey picture, its frame.hex has
two hex digits a line, and its scene.hex gives the camera model the same grey on red, green and blue.

THE SCENE. The frame buffer keeps camera pixel (2u, 2v) of the 640 x 480 the camera sends as stored
pixel (u, v). The scene puts stored pixel (u, v) on all four sensor pixels that come out as camera pixels
(2u..2u+1, 2v..2v+1), so the camera model sends the frame pixel-doubled: the even rows and columns are the
stored frame exactly, the odd ones repeat them. The model reads the scene through the camera's window and
orientation, so those must be the registers the simulation uses (--mvfp, --hstart, --vstart; defaults =
the camera code's). Sensor pixels outside the window get the nearest edge pixel of the frame.
The camera model places the camera code's window (columns from 160, rows from 11) on its 656 x 488 array
and sends all 640 x 480 pixels of it, so with the defaults a replay is exact in every pixel, as the benches
check (tb/replay_tb.sv). The window is placed on the camera's mirrored and flipped picture, as on the board. For a
window the model cannot show whole (columns beyond its array, where it repeats the edge column; rows above
readout row 11, which are not picture; rows from readout row 496 on, and bytes in the line's gap, never
sent), grab.py prints a NOTE.
"""
import argparse
import binascii
import json
import os
import struct
import sys
import time

import numpy as np
from PIL import Image

MAGIC = b"GRAB"
HEADER = struct.Struct(">4sHHHHH")          # magic, version, width, height, pixel format, number of result words
VERSION, FORMAT_RGB565, FORMAT_GREY8 = 1, 1, 2
FORMAT_NAMES = {FORMAT_RGB565: "RGB565", FORMAT_GREY8: "GREY8"}

# the camera model's geometry (mp/models/ov7670_model.sv, ov7670_video): a 656 x 488 array, the window
# from HSTART 160 and VSTRT 12 on its central 640 x 480, i.e. 8 columns and 4 rows in from the edge
ARRAY_W, ARRAY_H, DEF_HSTART, DEF_VSTART, MARGIN_X, MARGIN_Y = 656, 488, 160, 12, 8, 4
OUT_W, OUT_H = 640, 480                     # VGA, what the camera code configures


class DumpError(Exception):
    pass


def dump_length(width, height, n_results, fmt=FORMAT_RGB565):
    pixel_bytes = 2 * width * height if fmt == FORMAT_RGB565 else width * height
    return HEADER.size + 4 * n_results + pixel_bytes + 2


def parse_header_format(buf, start):
    """(width, height, number of result words, pixel format) of the header at buf[start], or DumpError."""
    magic, version, width, height, fmt, n = HEADER.unpack_from(buf, start)
    if version != VERSION:
        raise DumpError(f"version {version}: this grab.py reads version {VERSION}")
    if fmt not in FORMAT_NAMES:
        raise DumpError(f"pixel format {fmt}: only {FORMAT_RGB565} (RGB565) and {FORMAT_GREY8} (8-bit grey) are known")
    if not (1 <= width <= 4096 and 1 <= height <= 4096):
        raise DumpError(f"implausible size {width} x {height}: not a dump header")
    return width, height, n, fmt


def parse_header(buf, start):
    return parse_header_format(buf, start)[:3]


def decode_at(buf, start):
    """The dump whose magic word is at buf[start]: a dict, or DumpError (header, short, bad CRC)."""
    if len(buf) - start < HEADER.size:
        raise DumpError(f"only {len(buf) - start} bytes after the magic word: the header is incomplete")
    width, height, n, fmt = parse_header_format(buf, start)
    total = dump_length(width, height, n, fmt)
    data = bytes(buf[start:start + total])
    if len(data) < total:
        raise DumpError(f"short dump: {len(data)} of {total} bytes (bytes lost on the way, or the capture stopped early)")
    crc_sent = (data[-2] << 8) | data[-1]
    crc_calc = binascii.crc_hqx(data[:-2], 0xFFFF)
    if crc_sent != crc_calc:
        raise DumpError(f"CRC error: the dump says 0x{crc_sent:04x}, the bytes received give 0x{crc_calc:04x}: "
                        f"at least one byte was corrupted or lost on the way; grab it again")
    results = list(struct.unpack_from(f">{n}I", data, HEADER.size))
    if fmt == FORMAT_RGB565:
        pixels = np.frombuffer(data, dtype=">u2", count=width * height, offset=HEADER.size + 4 * n).astype(np.uint16)
    else:
        pixels = np.frombuffer(data, dtype=np.uint8, count=width * height, offset=HEADER.size + 4 * n).copy()
    return {"offset": start, "bytes": total, "width": width, "height": height, "n_results": n, "format": fmt,
            "results": results, "crc": crc_sent, "pixels": pixels.reshape(height, width)}


def decode(buf):
    """The first whole dump in buf: returns a dict, or raises DumpError (no header, short, bad CRC). A magic word
    that turns out not to start a good dump (the four letters inside other data, or a corrupted dump) is passed
    over; if no good dump follows, the first problem found is the one reported."""
    start = buf.find(MAGIC)
    if start < 0:
        raise DumpError("no 'GRAB' magic word in the data: no dump started")
    first = None
    while start >= 0:
        try:
            return decode_at(buf, start)
        except DumpError as e:
            first = first or e
        start = buf.find(MAGIC, start + 1)
    raise first


def rgb565_to_rgb888(p):
    """As camera_pkg::rgb565_to_rgb888: each channel widened by repeating its top bits."""
    r, g, b = (p >> 11) & 31, (p >> 5) & 63, p & 31
    return np.stack([(r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)], axis=-1).astype(np.uint8)


def frame_rgb(pixels):
    """The frame as 8-bit RGB: RGB565 widened, or grey (8-bit pixels) on all three channels."""
    if pixels.dtype == np.uint8:
        return np.repeat(pixels[..., None], 3, axis=-1)
    return rgb565_to_rgb888(pixels)


def scene_from_frame(pixels, mvfp, hstart, vstart):
    """The 656 x 488 scene that makes the camera model send this frame (see THE SCENE above)."""
    height, width = pixels.shape
    dec_x, dec_y = OUT_W // width, OUT_H // height
    mirror, flip = bool(mvfp & 0x20), bool(mvfp & 0x10)
    # the model (VGA): output (col, row) -> q = col + (hstart - 160), ax = mirror ? 8 + 639 - q : 8 + q; rows likewise,
    # with flip ay = 479 - (row + vstart - 12), without ay = 4 + row + vstart - 12
    dx, dy = hstart - DEF_HSTART, vstart - DEF_VSTART
    ax, ay = np.arange(ARRAY_W), np.arange(ARRAY_H)
    col = np.clip((MARGIN_X + OUT_W - 1 - ax - dx) if mirror else (ax - MARGIN_X - dx), 0, OUT_W - 1)
    row = np.clip((OUT_H - 1 - ay - dy) if flip else (ay - MARGIN_Y - dy), 0, OUT_H - 1)
    u = np.minimum(col // dec_x, width - 1)
    v = np.minimum(row // dec_y, height - 1)
    return frame_rgb(pixels)[v[:, None], u[None, :]]


def model_limits(hstart, vstart):
    """What the camera model cannot show of this window from the scene: (columns beyond its 656 x 488 array,
    where it repeats the edge column, columns never sent because they fall in the line's gap, rows beyond the
    array, rows never sent (readout rows from 496 on)). All 0 for HSTART 152..168, rows from 8 to 16. (Readout
    rows above 11 are not picture on the board either, and the model sends them as the board does: main() notes them.)"""
    first_x, first_y = (hstart - DEF_HSTART) + MARGIN_X, (vstart - DEF_VSTART) + MARGIN_Y
    dark_cols = max(0, -first_x) + max(0, first_x + OUT_W - ARRAY_W)
    dark_rows = max(0, -first_y) + max(0, first_y + OUT_H - ARRAY_H)
    unsent_cols = sum(1 for c in range(OUT_W) if 207 <= (2 * hstart + 2 * c) % 1568 < 274)   # the gap (measured)
    unsent_rows = max(0, vstart + OUT_H - 496)                               # the readout ends at row 495 (measured)
    return dark_cols, unsent_cols, dark_rows, unsent_rows


def write_outputs(d, out, source, mvfp, hstart, vstart):
    os.makedirs(out, exist_ok=True)
    px = d["pixels"]
    fmt = d.get("format", FORMAT_RGB565)
    if fmt == FORMAT_GREY8:
        Image.fromarray(px, "L").save(os.path.join(out, "frame.png"))
    else:
        Image.fromarray(rgb565_to_rgb888(px), "RGB").save(os.path.join(out, "frame.png"))
    digits = 2 if fmt == FORMAT_GREY8 else 4
    with open(os.path.join(out, "frame.hex"), "w") as f:
        f.write("\n".join(f"{p:0{digits}x}" for p in px.ravel().tolist()) + "\n")
    scene = scene_from_frame(px, mvfp, hstart, vstart)
    with open(os.path.join(out, "scene.hex"), "w") as f:      # the layout of tools/make_scene.py
        for y in range(ARRAY_H):
            f.write("\n".join(f"{r:02x}{g:02x}{b:02x}" for r, g, b in scene[y].tolist()) + "\n")
    dark_cols, unsent_cols, dark_rows, unsent_rows = model_limits(hstart, vstart)
    info = {
        "magic": MAGIC.decode(), "version": VERSION, "width": d["width"], "height": d["height"],
        "pixel_format": FORMAT_NAMES[fmt], "n_results": d["n_results"],
        "results": d["results"], "results_hex": [f"0x{w:08x}" for w in d["results"]],
        "crc": f"0x{d['crc']:04x}", "crc_ok": True, "bytes": d["bytes"], "offset_in_source": d["offset"],
        "source": source, "decoded": time.strftime("%Y-%m-%d %H:%M:%S"),
        "scene": {"file": "scene.hex", "size": [ARRAY_W, ARRAY_H], "mvfp": f"0x{mvfp:02x}",
                  "mirror": bool(mvfp & 0x20), "flip": bool(mvfp & 0x10), "hstart": hstart, "vstart": vstart,
                  "model_columns_dark": dark_cols, "model_columns_never_sent": unsent_cols,
                  "model_rows_dark": dark_rows, "model_rows_never_sent": unsent_rows},
    }
    with open(os.path.join(out, "results.json"), "w") as f:
        json.dump(info, f, indent=2)
        f.write("\n")
    return dark_cols, unsent_cols, dark_rows, unsent_rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", required=True, help="the raw bytes of a grab (dump.bin, from tcl/grab_jtag.tcl)")
    ap.add_argument("--out", default=".", help="where to write the files (default: .)")
    ap.add_argument("--mvfp", type=lambda s: int(s, 0), default=0x37, help="the camera's MVFP register (default 0x37: mirror + flip)")
    ap.add_argument("--hstart", type=int, default=160, help="window start column, HSTART*8 + HREF[2:0] (default 160)")
    ap.add_argument("--vstart", type=int, default=11, help="window start row, VSTRT*4 + VREF[1:0] (default 11)")
    a = ap.parse_args(argv)

    try:
        with open(a.file, "rb") as f:
            buf = f.read()
    except OSError as e:
        print(f"grab.py: {e}", file=sys.stderr)
        return 2
    source = a.file
    try:
        d = decode(buf)
    except DumpError as e:
        print(f"grab.py: REJECTED: {e}")
        return 1
    limits = write_outputs(d, a.out, source, a.mvfp, a.hstart, a.vstart)
    print(f"dump OK: {d['width']} x {d['height']} {FORMAT_NAMES[d['format']]}, {d['n_results']} result words, CRC 0x{d['crc']:04x}, "
          f"{d['bytes']} bytes (from byte {d['offset']} of {source})")
    print("results: " + " ".join(f"{w}" for w in d["results"]))
    print(f"wrote {a.out}/frame.png, frame.hex, scene.hex (MVFP 0x{a.mvfp:02x}, window from column {a.hstart}, row {a.vstart}), results.json")
    if any(limits):
        dark_cols, unsent_cols, dark_rows, unsent_rows = limits
        print(f"NOTE: the camera model cannot show all of this window: {dark_cols} column(s) and {dark_rows} row(s) fall beyond "
              f"its 656 x 488 array (it repeats the array's edge there, whatever the scene), and {unsent_cols} column(s) and "
              f"{unsent_rows} row(s) are never sent (the line's gap; the readout ends at row 495). A replay in simulation differs "
              f"there. For a replay that is exact everywhere, use a window from column 152 to 168, such as the camera code's 160 "
              f"(the default), and rows from 11 to 16.")
    if a.vstart < 11:
        print(f"NOTE: the window's first {11 - a.vstart} row(s) are readout rows above 11, which the camera does not send as "
              f"picture (row 10 is a dark blue line, whatever it sees); the camera model sends them as the camera does. The "
              f"camera code's window starts at row 11.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
