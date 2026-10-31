#!/usr/bin/env python3
"""jtag_export.py -- decode the frame grab's memory, saved by hand from Quartus's In-System Memory Content Editor.

    python3 jtag_export.py grab.mif                    (or grab.hex)
    python3 jtag_export.py grab.mif --out grab1

In Quartus, with the design that has the JTAG frame grab (rtl/frame_snap.sv) programmed: press KEY1 and wait for
LEDR3, then Tools > In-System Memory Content Editor, select the instance GRAB, Read Data from In-System Memory
(the button, or right-click), and File > Export Data to File, as a .mif (Memory Initialization File) or a .hex
(Intel HEX). This script turns that file into the same files tools/grab.py writes from tcl/grab_jtag.tcl's dump.bin:

    frame.png      the picture as frame_snap kept it
    frame.hex      the pixels for $readmemh (4 hex digits a line for RGB565, 2 for grey)
    scene.hex      the picture as a scene for the camera model (tools/grab.py: THE SCENE)
    results.json   the header, the result words (word 0 first), the CRC, where it came from

The memory holds the bytes of a grab (frame_snap.sv: THE MEMORY), so the decoding, the CRC check and the files
are tools/grab.py's own. Options --mvfp, --hstart and --vstart are grab.py's (the camera's registers,
for scene.hex). tcl/grab_jtag.tcl does all of this with one command, without the editor.

Exit status 0 = a whole snapshot with a good CRC; 1 = no snapshot, or a bad one; 2 = a file that cannot be read.
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import grab  # noqa: E402

RADIX = {"HEX": 16, "BIN": 2, "OCT": 8, "DEC": 10, "UNS": 10}


class ExportError(Exception):
    pass


def read_mif(text):
    """A Memory Initialization File: returns (width, depth, {address: value})."""
    text = re.sub(r"--[^\n]*", "", text)                     # comments: -- to the end of the line,
    text = re.sub(r"%[^%]*%", "", text)                      # and % ... %
    head, sep, body = text.partition("CONTENT")
    if not sep:
        head, sep, body = text.upper().partition("CONTENT")
    if not sep:
        raise ExportError("no CONTENT section: not a .mif file")
    keys = {k.upper(): v.strip().upper() for k, v in re.findall(r"(\w+)\s*=\s*([^;]+);", head)}
    try:
        width, depth = int(keys["WIDTH"]), int(keys["DEPTH"])
    except (KeyError, ValueError):
        raise ExportError("WIDTH or DEPTH missing from the .mif header")
    ar, dr = RADIX.get(keys.get("ADDRESS_RADIX", "HEX")), RADIX.get(keys.get("DATA_RADIX", "HEX"))
    if ar is None or dr is None:
        raise ExportError(f"unknown radix: ADDRESS_RADIX {keys.get('ADDRESS_RADIX')}, DATA_RADIX {keys.get('DATA_RADIX')}")
    body = re.sub(r"(?i)^\s*BEGIN", "", body.strip())
    body = re.split(r"(?i)\bEND\s*;", body)[0]
    words = {}
    for entry in body.split(";"):
        entry = entry.strip()
        if not entry:
            continue
        where, colon, values = entry.partition(":")
        if not colon:
            raise ExportError(f"cannot read the .mif line '{entry[:40]}'")
        where, values = where.strip(), values.split()
        if where.startswith("["):                            # [first..last] : value (or values, repeated)
            m = re.fullmatch(r"\[\s*(\w+)\s*\.\.\s*(\w+)\s*\]", where)
            if not m:
                raise ExportError(f"cannot read the .mif address range '{where}'")
            first, last = int(m.group(1), ar), int(m.group(2), ar)
            vals = [int(v, dr) for v in values]
            for k, a in enumerate(range(first, last + 1)):
                words[a] = vals[k % len(vals)]
        else:                                                # address : value value ... (consecutive addresses)
            first = int(where, ar)
            for k, v in enumerate(values):
                words[first + k] = int(v, dr)
    return width, depth, words


def read_intel_hex(text, width):
    """An Intel HEX file as Quartus writes memory contents: each record's address counts WORDS (of `width` bits,
    high byte first), extended by type 04 (upper 16 bits) or 02 (segment x 16) records. Returns {address: value}."""
    nbytes = (width + 7) // 8
    words, base = {}, 0
    for n, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ExportError(f"line {n} of the .hex file does not start with ':'")
        rec = bytes.fromhex(line[1:])
        if len(rec) < 5 or len(rec) != rec[0] + 5:
            raise ExportError(f"line {n} of the .hex file has the wrong length")
        if sum(rec) & 0xFF:
            raise ExportError(f"line {n} of the .hex file fails its checksum")
        count, addr, kind, data = rec[0], (rec[1] << 8) | rec[2], rec[3], rec[4:4 + rec[0]]
        if kind == 0:
            for k in range(0, count, nbytes):
                words[base + addr + k // nbytes] = int.from_bytes(data[k:k + nbytes], "big")
        elif kind == 1:
            break
        elif kind == 4:
            base = int.from_bytes(data, "big") << 16
        elif kind == 2:
            base = int.from_bytes(data, "big") << 4
    return words


def snapshot_bytes(words, width=16):
    """The memory's words, from address 0 on, as the bytes of a dump (each word high byte first)."""
    if width != 16:
        raise ExportError(f"the memory is {width} bits wide: the frame grab's GRAB memory is 16")
    if not words:
        raise ExportError("the file holds no words")
    top = max(words) + 1
    return b"".join(words.get(a, 0).to_bytes(2, "big") for a in range(top))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="the memory, exported from the In-System Memory Content Editor (.mif or .hex)")
    ap.add_argument("--out", default=".", help="where to write the files (default: .)")
    ap.add_argument("--width", type=int, default=16, help="word width for a .hex file (the GRAB memory's: 16)")
    ap.add_argument("--mvfp", type=lambda s: int(s, 0), default=0x37, help="the camera's MVFP register (default 0x37: mirror + flip)")
    ap.add_argument("--hstart", type=int, default=160, help="window start column, HSTART*8 + HREF[2:0] (default 160)")
    ap.add_argument("--vstart", type=int, default=11, help="window start row, VSTRT*4 + VREF[1:0] (default 11)")
    a = ap.parse_args(argv)
    try:
        text = open(a.file, encoding="ascii", errors="replace").read()
        if a.file.lower().endswith(".mif") or "CONTENT" in text.upper():
            width, depth, words = read_mif(text)
        else:
            width, words = a.width, read_intel_hex(text, a.width)
        data = snapshot_bytes(words, width)
    except (OSError, ExportError, ValueError) as e:
        print(f"jtag_export.py: {a.file}: {e}", file=sys.stderr)
        return 2
    if not data.startswith(grab.MAGIC):
        print("jtag_export.py: the memory does not start with the magic word 'GRAB': no snapshot has been taken since the "
              "board was programmed. Press KEY1, wait for LEDR3, read the memory again and export it again.")
        return 1
    try:
        d = grab.decode(data)
    except grab.DumpError as e:
        print(f"jtag_export.py: REJECTED: {e}\n  (a snapshot being taken while the memory was read gives this: press KEY1 "
              f"once, wait for LEDR3, then read and export again)")
        return 1
    grab.write_outputs(d, a.out, a.file, a.mvfp, a.hstart, a.vstart)
    print(f"snapshot OK: {d['width']} x {d['height']} {grab.FORMAT_NAMES[d['format']]}, {d['n_results']} result words, "
          f"CRC 0x{d['crc']:04x}")
    print("results: " + " ".join(f"{w}" for w in d["results"]))
    print(f"wrote {a.out}/frame.png, frame.hex, scene.hex, results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
