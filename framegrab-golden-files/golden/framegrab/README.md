# golden/framegrab: a photo from the robot, checked against the golden model

Group 26, Wilson (M2). Take a photo with the robot's camera over the USB-Blaster (no extra cable). The board
keeps, with that photo, what the robot's vision chain computed from the **same frame**. Then check those numbers
against the golden model (`golden/model.py`), and replay the photo through the RTL in Verilator.

## Quick start

Python first: `pip install numpy pandas pillow matplotlib`. Run everything from the repo's top folder.

1. **Lab PC, once.** Compile `golden/framegrab/quartus/vision_grab.qpf` in Quartus 18.1 and check that the
   timing has no negative slack. Program `quartus/output_files/vision_grab.sof` with the camera plugged in.
   - LEDR0 on means the camera is configured. LEDR2 flickers with the frames.
   - The VGA shows the picture, with a green cross on the detected LED.
   - SW1 picks the gates (0 = V2, 1 = V1 bypass). SW9 up locks exposure and white balance once the picture
     has settled.
2. **Lab PC, take photos** (Command Prompt):

   ```
   python golden\framegrab\tools\take_photo.py red_1m_centre
   ```

   This takes the photo and checks it. Each photo lands in `golden\framegrab\grabs\red_1m_centre\`; look at
   `validation.png` there. If `quartus_stp` isn't found, add `--quartus C:\intelFPGA_lite\18.1\quartus\bin64`.
   - The last line should be `PASS: validate_grab ...`.
   - FAIL on *integrity* (a rejected or stale frame): take the photo again.
   - FAIL on *KEPT*: the board and the golden model really disagree. Keep that folder.
3. **Ed or a laptop with Verilator 5.050: the replay** (the lab PC has no Verilator, so it skips the replay).
   Commit the grab folders, pull them there, then run:

   ```
   python golden/framegrab/validate_grab.py golden/framegrab/grabs/*
   ```

4. **Notebook.** Open `golden/framegrab/framegrab_validation.ipynb` and Run All. It shows every photo in
   `grabs/`: the checks, the pictures, the board's numbers against the model's, and the threshold plot.
5. **No board.** `python golden/framegrab/framegrab_sim.py` runs the whole flow on a simulated board
   (needs Verilator). `bash run_all_tests.sh` runs it with everything else.

The rest of this file explains each step and each check in detail.

## What it is built from

It is built from three things already in hand. Nothing outside this folder changes, apart from three job lines
in `run_all_tests.sh`.

| Part | Where from | Files |
|---|---|---|
| Camera code | team repo `vendor/camera` (50 fps, as the robot) | `video_pll`, `clock_divider`, `ov7670_config`, `ov7670_capture`, `frame_buffer`, `camera_display`, `vga_sink.qsys`, `tb/ov7670_model` |
| JTAG frame grab | FrameGrabJtag V2026.3 (the `home/` handout), **unchanged** in `vendor/` | `rtl/frame_snap`, `snapshot_ram`, `snapshot_probes`, `ov7670_hold`, `tcl/grab_jtag.tcl`, `tools/grab.py`, `tools/jtag_export.py` |
| Vision chain | team repo `rtl/` (as in `major_project_top`) | `camera_stream_adapter` → `red_mask` → `led_centroid` (SW1 = `gate_bypass`) |

New files:

| File | What it does |
|---|---|
| `rtl/vision_grab_top.sv` | DE1-SoC top: camera, the robot's chain, `frame_snap`, VGA with a green cross on the robot's target |
| `rtl/vision_grab_results.sv` | the 18 result words kept with the photo (FULL + KEPT, below) |
| `quartus/vision_grab.qpf/.qsf/.sdc`, `build.sh` | the Quartus project (FrameGrabJtag's pins and constraints) |
| `tools/take_photo.py` | on the bench: programs the board (optional), takes the photo, runs the checks |
| `validate_grab.py` | the golden-model check of one or more photos |
| `sim/vision_grab_replay_tb.sv` | the replay: `scene.hex` → camera model → the robot's RTL |
| `sim/vision_grab_top_tb.sv` | the board, simulated pins to pins (two photos: KEY1, then the PC's request) |
| `framegrab_sim.py` | the whole flow on the simulated board; `--selftest`; `--broken` |
| `framegrab_validation.ipynb` | the notebook: every photo in `grabs/`, the checks, the pictures |
| `grabs/` | the photos (`sim_*` are from the simulated board) |

## Taking photos on the bench

1. Compile `quartus/vision_grab.qpf` in Quartus 18.1 (or `cd quartus && sh build.sh`). The memory is
   `frame_buffer` (150 M10K) plus `frame_snap` (about 150), out of 397.
2. Plug in the camera and program `quartus/output_files/vision_grab.sof`.
   - LEDR0 lit means the camera is configured. LEDR2 flickers with the frames.
   - The VGA shows the picture, with a green cross where the robot thinks the LED is.
3. Set the switches as on the robot:
   - **SW1**: 0 = V2 gates, 1 = V1 bypass.
   - **SW9** up: the camera holds its exposure and white balance, so the colours stay the same from one photo to the next.
4. From the repo's top folder, on the computer with the USB-Blaster:

   ```
   python golden/framegrab/tools/take_photo.py red_1m_centre          one photo, checked
   python golden/framegrab/tools/take_photo.py red_1m --count 5       five
   python golden/framegrab/tools/take_photo.py red_1m --program       program the .sof first
   python golden/framegrab/tools/take_photo.py red_1m --held          read the photo KEY1 took
   ```

   It runs `quartus_stp -t vendor/tcl/grab_jtag.tcl` (FrameGrabJtag's script) into
   `golden/framegrab/grabs/<name>/` and then `validate_grab.py` on it. It finds `quartus_stp` on the PATH,
   in `$QUARTUS_ROOTDIR`, or in `C:\intelFPGA_lite\18.1\quartus\bin64`. If it is somewhere else, give
   `--quartus <folder>`.
5. With Verilator on the PATH, the replay runs too. Without it (the lab PC, say), copy the folder to Ed or your
   laptop and run:

   ```
   python golden/framegrab/validate_grab.py golden/framegrab/grabs/red_1m_centre
   ```

   Or open `framegrab_validation.ipynb`, which checks every folder in `grabs/`.

Commit `dump.bin`, `results.json`, `frame.png`, `grab_jtag.log` and `validation.*`. `grabs/.gitignore` keeps
out the `.hex` files: `scene.hex` alone is 2.2 MB, and `validate_grab.py` makes them again from `dump.bin`.

Photos worth taking for the report:
- the LED at several distances and positions;
- SW1 both ways;
- two red LEDs (V2 rejects them, V1 steers to the midpoint);
- no LED;
- a red object that is not an LED;
- bright room and dim room.

## What the checks mean

The photo is camera pixel (2u, 2v) of the 640 × 480 frame: a quarter of the pixels the robot saw. So the board
keeps two sets of results.

- **KEPT is exact.** A second `led_centroid` (same RTL, same parameters) is fed only the pixels the photo holds,
  at u = x/2, v = y/2. Every input it saw is in `frame.hex`. So `model.frame_result(frame.hex)` must give the
  board's numbers exactly: valid, x, y, count, confidence, reject flags, bounding box, plus the red pixels'
  coordinate sums.

  This is the real camera's data through the real FPGA, compared with `model.py`. A mismatch is a bug, in the
  RTL or in the model.
- **FULL is approximate.** It is the robot's own `led_centroid` over the full frame. It is compared with the
  model on the pixel-doubled photo, within 3 px and 35 %, and also against KEPT (count ≈ 4 × KEPT,
  x ≈ 2 × u). A miss is reported as WARN, never FAIL. It means the LED blob is only a few pixels across, so
  the quarter of the pixels in the photo cannot pin it down.
- **The replay is exact.** `scene.hex` goes through the repo's camera model, `ov7670_capture` and the robot's
  RTL. Three things must hold:
  - every one of the 307,200 camera pixels is `frame.hex` pixel-doubled;
  - the replay's KEPT words equal the board's;
  - the replay's FULL words equal the model on the doubled photo.

  So FULL is checked exactly too, in simulation, on the real photo.
- **Integrity.** The words are Group 26's (layout 1) and belong to the kept frame (live frame number = result
  frame number + 1). Both centroids reported, the adapter passed the frame (`frame_good`), and all 76,800 kept
  pixels went through. A rejected or stale frame fails here: take the photo again.
- **Truth** (simulated board only). The board's FULL words equal the model on the full-resolution frame the
  simulated camera really sent.

## The result words (layout 1, `vision_grab_results.sv`)

| Word | Contents |
|---|---|
| 0 | {live frame no, result frame no} (16 + 16) |
| 1 | status: 0x26, layout 0x01, last_error, stream_ok, gate_bypass (SW1), frame_good, kept_done, full_done |
| 2–6 | FULL: {x, y}, red_count, {valid, flags, confidence}, {bbox x min, max}, {bbox y min, max} |
| 7–11 | KEPT: the same, in photo coordinates |
| 12, 13 | KEPT: sum of u, sum of v over red kept pixels |
| 14 | kept pixels through the adapter (76,800) |
| 15–17 | adapter: {good_frames, bad_frames}, dropped_pixels, frame_period |

The adapter counters run from reset. The camera's first frames, sent before it is configured, count as bad
frames and dropped pixels, so they are not zero on a good board.

## Without a board

```
python golden/framegrab/framegrab_sim.py               the simulated board: two photos, grab.py, every check (~1 min)
python golden/framegrab/framegrab_sim.py --scene photo_leds
python golden/framegrab/framegrab_sim.py --selftest    the checks themselves: right words pass, wrong ones fail (seconds)
python golden/framegrab/framegrab_sim.py --broken      a broken mask in the replay must be CAUGHT
```

`run_all_tests.sh` runs all three. On the simulated board, the camera model shows a lab photo with LEDs on it at
full 640 × 480, so the odd pixels differ from the even ones, as on a real camera. The two photos are KEY1 with
SW1 down, then the PC's request (what `grab_jtag.tcl` does) with SW1 up.

## Not done here

- Not compiled in Quartus. This workspace has no Quartus. Simulation proves the logic (Verilator, pins to
  pins), not the fit or the timing: check `build.sh`'s timing line.
- The photo is 320 × 240, so the FULL results cannot be checked exactly against the photo itself. The replay
  is the exact check of FULL. KEPT is the exact check of the board.
- `tri_led_detect` (V3 candidate) is not on the board: `vision_grab_top` is the robot's red path only.
