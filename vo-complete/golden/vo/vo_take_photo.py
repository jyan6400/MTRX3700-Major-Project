#!/usr/bin/env python3
"""vo_take_photo.py -- Group 26 (Wilson, M2): a JTAG photo from vo_grab_top, with the tape-measured pose, checked.

    python golden/vo/vo_take_photo.py cal_800 --Y -800                    800 mm from the LED line, centred on green
    python golden/vo/vo_take_photo.py cal_900r --Y -900 --X 200           200 mm to the right of green
    python golden/vo/vo_take_photo.py look --program                      program vo_grab.sof first, no pose
    python golden/vo/vo_take_photo.py cal_600 --Y -600 --held             read the photo KEY1 took

The pose (mm, degrees): Y = minus the distance from the camera LENS to the wall the LEDs are on, measured square
to it; X = sideways from the green LED, right positive; --psi = turned left of square-on (leave 0 and square the
robot up). Photos go to golden/framegrab/grabs/<name>/ with truth.json, then vo_validate_grab.py runs on them.
Calibrate from four to six such photos: python golden/vo/vo_calibrate.py golden/framegrab/grabs/cal_* --write
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FG = HERE.parent / "framegrab"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(FG / "tools"))
sys.path.insert(0, str(FG))

import take_photo as tp          # noqa: E402
import vo_validate_grab as vv    # noqa: E402

SOF = FG / "quartus" / "output_files" / "vo_grab.sof"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name")
    ap.add_argument("--Y", type=float, help="minus the lens-to-LED-wall distance, mm (e.g. -800)")
    ap.add_argument("--X", type=float, default=0.0, help="sideways from the green LED, right positive, mm")
    ap.add_argument("--psi", type=float, default=0.0, help="turned left of square-on, degrees")
    ap.add_argument("--held", action="store_true")
    ap.add_argument("--program", nargs="?", const=str(SOF), default=None, metavar="SOF")
    ap.add_argument("--quartus")
    ap.add_argument("--out", type=Path, default=tp.GRABS)
    a = ap.parse_args(argv)
    if a.Y is not None and a.Y > 0:
        a.Y = -a.Y                       # a distance typed as positive
    stp = tp.find_tool("quartus_stp", a.quartus)
    if not stp:
        sys.exit("vo_take_photo: quartus_stp not found (give --quartus <its bin64 folder>)")
    if a.program:
        tp.program(a.program, a.quartus)
    out = a.out / a.name
    if not tp.grab_once(out, stp, a.held):
        print(f"FAIL: vo_take_photo: no photo in {out}")
        return 1
    if a.Y is not None:
        json.dump({"pose": {"X": a.X, "Y": a.Y, "psi_deg": a.psi}}, open(out / "truth.json", "w"), indent=2)
    rep, _ = vv.validate(out)
    print(f"{'PASS' if rep.fails == 0 else 'FAIL'}: vo_take_photo {a.name}: {rep.fails} failed, {rep.warns} warning(s); "
          f"see {out / 'vo_validation.png'}")
    return 0 if rep.fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
