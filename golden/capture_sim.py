"""Make a simulated board capture: the real major_project_top + Ed's OV7670 model
+ vision_tap + a SignalTap emulator (golden/sim/top_capture_tb.sv).

    python golden/capture_sim.py [-o golden/data/captures/sim_photo] [--depth 32768]

Until real SignalTap captures exist this is what the notebook replays, and it
tests the whole capture path (vision_tap, the exports, capture.py) before the
lab: the replay of a simulated capture must match the simulated board exactly.
Needs Verilator 5.050, numpy and Pillow. Takes about a minute.
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

import capture      # noqa: E402
import stimulus     # noqa: E402

TOP_RTL = """
rtl/integration/major_project_top.sv rtl/integration/camera_stream_adapter.sv
rtl/integration/vision_result_cdc.sv rtl/integration/sonar_result_cdc.sv
rtl/image_processing/led_colour_mask.sv rtl/image_processing/red_mask.sv rtl/perception/led_centroid.sv
rtl/navigation/camera_frame_watchdog.sv rtl/navigation/steering_controller.sv rtl/navigation/docking_fsm.sv
rtl/actuation/sonar_freshness.sv rtl/actuation/sonar_filter.sv rtl/actuation/ir_safety_latch.sv
rtl/actuation/motion_interface.sv
vendor/camera/rtl/video_pll.sv vendor/camera/rtl/clock_divider.sv vendor/camera/rtl/ov7670_config.sv
vendor/camera/rtl/ov7670_capture.sv vendor/camera/rtl/i2c_master.sv
vendor/ultrasonic/rtl/sonar_pll.sv vendor/ultrasonic/rtl/sonar_range.sv
vendor/ir/rtl/nec_decoder.sv vendor/chassis/rtl/chassis_drive.sv vendor/chassis/rtl/uart_tx.sv
""".split()

# (scene, SW1): a plain red LED, then two red LEDs with the V2 gates on and off (R9)
DEFAULT_SCENES = [("photo_leds", 0), ("photo_two_red", 0), ("photo_two_red", 1), ("photo_red_left", 1)]


def write_scenes(scenes, out: Path):
    """656x488 RGB888 scene files for ov7670_model (mirrored + flipped, as the
    course register list reads the sensor), via sim/tools/make_led_frames.py."""
    import os
    mod = stimulus._make_led_frames_module()
    out.mkdir(parents=True, exist_ok=True)
    here = os.getcwd()
    try:
        os.chdir(ROOT)
        base = mod.load_base()
        mod.DATA = str(out)
        files = []
        for name in dict.fromkeys(s for s, _ in scenes):
            img = base.copy()
            for led in stimulus.PHOTO_SCENES[name]:
                img = mod.draw_led(img, *led)
            mod.save_scene(name, img)
            files.append(name)
    finally:
        os.chdir(here)
    shutil.copy(out / f"{scenes[0][0]}_scene.hex", out / "scene_init.hex")
    return {n: out / f"{n}_scene.hex" for n in files}


def build(build_dir: Path):
    binp = build_dir / "Vtop_capture_tb"
    srcs = [ROOT / f for f in TOP_RTL] + [ROOT / "vendor/camera/tb/ov7670_model.sv",
                                           HERE / "rtl/vision_tap.sv", HERE / "sim/top_capture_tb.sv"]
    if binp.exists() and all(s.stat().st_mtime < binp.stat().st_mtime for s in srcs):
        return binp
    build_dir.mkdir(parents=True, exist_ok=True)
    cmd = ["bash", str(HERE / "verilator_build.sh"), str(build_dir), "top_capture_tb",
           "-Wno-fatal", "-Wno-CASEINCOMPLETE", "-O2"] + [str(s) for s in srcs]
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    (build_dir / "compile.log").write_text(r.stdout + r.stderr)
    if r.returncode != 0:
        raise RuntimeError(f"top_capture_tb compile failed, see {build_dir/'compile.log'}")
    return binp


def run(out_dir, scenes=DEFAULT_SCENES, depth=32768, frames_per_scene=1):
    out_dir = Path(out_dir)
    work = ROOT / "build/golden/capture"
    files = write_scenes(scenes, work)
    binp = build(ROOT / "build/golden/top_capture_tb")
    prefix = work / "sim"
    args = [str(binp), f"+PREFIX={prefix}", f"+DEPTH={depth}", f"+SCENES={len(scenes)}",
            f"+FRAMES_PER_SCENE={frames_per_scene}"]
    for i, (name, sw1) in enumerate(scenes):
        args += [f"+SCENE{i}={files[name]}", f"+SW1_{i}={sw1}"]
    r = subprocess.run(args, cwd=ROOT, capture_output=True, text=True)
    (work / "top_capture_tb.log").write_text(r.stdout + r.stderr)
    if r.returncode != 0 or "PASS: top_capture_tb" not in r.stdout:
        raise RuntimeError(f"top_capture_tb failed, see {work/'top_capture_tb.log'}\n{r.stdout[-1500:]}")
    cap = capture.load(f"{prefix}.taplog")
    summ = capture.save(cap, out_dir)
    for ext in (".vcd", "_bits.vcd"):                # keep the exports next to it, compressed
        src = Path(f"{prefix}{ext}")
        if src.exists():
            import gzip
            with open(src, "rb") as fi, gzip.open(out_dir / f"export{ext}.gz", "wb") as fo:
                shutil.copyfileobj(fi, fo)
    (out_dir / "scenes.txt").write_text("\n".join(f"{n} SW1={s}" for n, s in scenes) + "\n")
    return summ, r.stdout


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--out", default=str(HERE / "data/captures/sim_photo"))
    ap.add_argument("--depth", type=int, default=32768)
    ap.add_argument("--frames-per-scene", type=int, default=1)
    a = ap.parse_args(argv)
    summ, log = run(a.out, depth=a.depth, frames_per_scene=a.frames_per_scene)
    print(log.strip().splitlines()[-1])
    print(summ)


if __name__ == "__main__":
    main()
