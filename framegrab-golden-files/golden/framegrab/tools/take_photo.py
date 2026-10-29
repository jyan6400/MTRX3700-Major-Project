#!/usr/bin/env python3
"""take_photo.py -- Group 26 (Wilson, M2): take a photo with the robot's camera over the USB-Blaster and check it
against the golden model. Run it on the computer the DE1-SoC is plugged into (Quartus 18.1 installed).

    python golden/framegrab/tools/take_photo.py                       photo now -> golden/framegrab/grabs/grab_<time>
    python golden/framegrab/tools/take_photo.py red_1m_centre         into golden/framegrab/grabs/red_1m_centre
    python golden/framegrab/tools/take_photo.py red_1m --program      program vision_grab.sof first
    python golden/framegrab/tools/take_photo.py red_1m --held         no new snapshot: read the one KEY1 took
    python golden/framegrab/tools/take_photo.py red_1m --count 5      five photos, red_1m_1 .. red_1m_5
    python golden/framegrab/tools/take_photo.py --validate-only DIR   only the golden-model check on a folder

Before: compile golden/framegrab/quartus (vision_grab.qpf), program output_files/vision_grab.sof (or --program),
plug the camera in. LEDR0 lit = camera configured, LEDR2 flickers with the frames. The VGA shows the picture
with a green cross on the robot's target. SW1 picks the gates as on the robot (0 V2, 1 V1 bypass); SW9 up holds
the camera's exposure and white balance so the colours stop drifting between photos.

What it does: quartus_stp -t vendor/tcl/grab_jtag.tcl <folder> (FrameGrabJtag's script: flips the request,
waits for the snapshot, reads the memory GRAB into dump.bin and runs vendor/tools/grab.py: frame.png, frame.hex,
scene.hex, results.json), then validate_grab.py on the folder (the RTL replay too, when Verilator is on the
PATH; otherwise run validate_grab.py on Ed or wherever Verilator is, on the same folder).

quartus_stp is looked for on the PATH, in $QUARTUS_ROOTDIR, and in the usual install folders; --quartus DIR
gives the folder that holds it.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

FG = Path(__file__).resolve().parent.parent          # golden/framegrab
sys.path.insert(0, str(FG))
sys.path.insert(0, str(FG / "vendor" / "tools"))

GRABS = FG / "grabs"
TCL = FG / "vendor" / "tcl" / "grab_jtag.tcl"
SOF = FG / "quartus" / "output_files" / "vision_grab.sof"


def find_tool(name, hint=None):
    exe = name + (".exe" if os.name == "nt" else "")
    dirs = []
    if hint:
        dirs.append(Path(hint))
    if os.environ.get("QUARTUS_ROOTDIR"):
        q = Path(os.environ["QUARTUS_ROOTDIR"])
        dirs += [q / "bin64", q / "bin"]
    for base in (Path("C:/intelFPGA_lite/18.1/quartus"), Path("C:/intelFPGA/18.1/quartus"),
                 Path.home() / "intelFPGA_lite/18.1/quartus", Path("/opt/intelFPGA_lite/18.1/quartus"),
                 Path("/opt/intelFPGA/18.1/quartus")):
        dirs += [base / "bin64", base / "bin"]
    for d in dirs:
        if (d / exe).exists():
            return str(d / exe)
    return shutil.which(name)


def program(sof, quartus_dir):
    pgm = find_tool("quartus_pgm", quartus_dir)
    if not pgm:
        sys.exit("take_photo: quartus_pgm not found (give --quartus <folder with quartus_pgm>)")
    if not Path(sof).exists():
        sys.exit(f"take_photo: {sof} not found: compile golden/framegrab/quartus/vision_grab.qpf first")
    print(f"programming {sof} (the DE1-SoC's FPGA is device 2 on the JTAG chain)")
    r = subprocess.run([pgm, "-m", "jtag", "-o", f"p;{sof}@2"])
    if r.returncode != 0:
        sys.exit("take_photo: programming failed (is the board on and the USB-Blaster plugged in?)")
    time.sleep(1.0)


def grab_once(out, stp, held=False):
    out.mkdir(parents=True, exist_ok=True)
    cmd = [stp, "-t", str(TCL), str(out)] + (["--held"] if held else [])
    print(" ".join(cmd))
    r = subprocess.run(cmd)
    if not (out / "dump.bin").exists():
        print(f"take_photo: no dump.bin in {out}: see {out / 'grab_jtag.log'}")
        return False
    if not (out / "results.json").exists():          # grab_jtag.tcl found no Python: decode it here
        import grab as vgrab
        if vgrab.main(["--file", str(out / "dump.bin"), "--out", str(out)]) != 0:
            return False
    return r.returncode == 0 or (out / "results.json").exists()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", nargs="?", help="folder name under golden/framegrab/grabs (default grab_<date>_<time>)")
    ap.add_argument("--held", action="store_true", help="read the snapshot KEY1 already took")
    ap.add_argument("--count", type=int, default=1, help="take this many photos, one after another")
    ap.add_argument("--program", nargs="?", const=str(SOF), default=None, metavar="SOF", help="program the FPGA first")
    ap.add_argument("--quartus", help="the folder with quartus_stp / quartus_pgm")
    ap.add_argument("--out", type=Path, default=GRABS, help=f"where the grab folders go (default {GRABS})")
    ap.add_argument("--no-validate", action="store_true", help="only take the photo")
    ap.add_argument("--replay", dest="replay", action="store_true", default=None, help="insist on the RTL replay")
    ap.add_argument("--no-replay", dest="replay", action="store_false")
    ap.add_argument("--validate-only", nargs="+", type=Path, metavar="DIR", help="only validate these folders")
    a = ap.parse_args(argv)

    import validate_grab as vg
    if a.validate_only:
        return vg.main([str(d) for d in a.validate_only] + ([] if a.replay is None else ["--replay" if a.replay else "--no-replay"]))

    stp = find_tool("quartus_stp", a.quartus)
    if not stp:
        sys.exit("take_photo: quartus_stp not found: install Quartus 18.1 Lite, or give --quartus <its bin64 folder>")
    if a.program:
        program(a.program, a.quartus)
    base = a.name or time.strftime("grab_%Y%m%d_%H%M%S")
    names = [base] if a.count == 1 else [f"{base}_{k}" for k in range(1, a.count + 1)]
    taken = []
    for n in names:
        out = a.out / n
        if grab_once(out, stp, a.held):
            taken.append(out)
        else:
            print(f"FAIL: take_photo: {out}")
    if not taken:
        return 1
    print(f"photos: {', '.join(str(t) for t in taken)} (frame.png in each)")
    if a.no_validate:
        return 0
    args = [str(t) for t in taken] + ([] if a.replay is None else ["--replay" if a.replay else "--no-replay"])
    return vg.main(args)


if __name__ == "__main__":
    sys.exit(main())
