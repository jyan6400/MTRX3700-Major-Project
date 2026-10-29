# Golden model and replay (R7)

Owner: Wilson (M2, image processing). Covers the whole vision chain of `major_project_top`:

```
ov7670_capture stream ─▶ camera_stream_adapter ─▶ red_mask ─▶ led_centroid ─▶ (vision CDC, navigation)
                                              └─▶ tri_led_detect (led_colour_mask R/G/B ─▶ colour_run_filter ─▶ 3 × led_centroid)
```

`tri_led_detect` is not in the top yet (V3 candidate); it is modelled and checked anyway.

| What | Where |
|---|---|
| Golden model (bit-exact, numpy) | `model.py` |
| Notebook: model, model vs RTL, board vs model vs RTL | `image_processing_golden.ipynb` |
| Headless check (PASS/FAIL/CAUGHT) | `check_golden.py` (run by `../run_all_tests.sh`) |
| RTL replay bench (Verilator) | `sim/vision_replay_tb.sv`, `run_rtl_replay.sh` |
| Capture probe for the board | `rtl/vision_tap.sv`, `rtl/vision_tap_hookup.svh` |
| SignalTap export → frames → replay | `capture.py`, `replay.py` |
| Simulated board capture (whole top from the camera pins) | `capture_sim.py`, `sim/top_capture_tb.sv` |
| Stimulus generators | `stimulus.py` |
| File formats | `vlog.py` |
| Deliberately broken module for CAUGHT | `broken/led_colour_mask_broken.sv` |
| Captures | `data/captures/<name>/` (`sim_photo` = simulated board) |

## Running it

```bash
pip install numpy pandas pillow matplotlib jupyter     # once
jupyter notebook golden/image_processing_golden.ipynb  # run all cells
python golden/check_golden.py --quick                  # the same checks, headless
python golden/check_golden.py --broken                 # must print CAUGHT
python golden/check_golden.py --system                 # whole robot from the camera pins
./run_all_tests.sh                                     # everything (R6), < 5 min
```

The model-vs-RTL parts need Verilator 5.050 and bash on `PATH` (WSL or Ed). Without them the notebook says so and runs the model-only parts plus the board-vs-model checks of a capture. On Windows without WSL, run `python golden/check_golden.py` in an Ed workspace for the RTL comparison.

## What the golden model is

`model.py` is written from each module's specified behaviour, not copied register by register from the RTL:

* **camera_stream_adapter**: a frame-level state machine over input events (`frame_start` pulses and valid pixels, each stamped with its PCLK cycle). Lock to the first `frame_start`; every pixel must be at the next raster position; every frame is closed exactly once (good on pixel (639,479), *empty close* on a coordinate error or a truncated frame); dropped-pixel, good/bad-frame, last-error and frame-period diagnostics.
* **led_colour_mask / red_mask**: a pure function of the RGB565 word (dominant-channel test).
* **led_centroid**: the closed-form frame result (count, sums, bounding box, gates, floor divisions, confidence = ⌊256·n/area⌋ saturated at 255), with SW1 bypass, plus two timing rules: `frame_done` 35 PCLKs after the close, and no result for a close within 34 PCLKs of an accepted one.
* **colour_run_filter**: a pixel passes when it and the previous two pixels of the same row segment are set; the filtered x is shifted left by 1.

Cycle convention (shared by model, RTL bench and capture): an input at cycle *c* is sampled on PCLK edge *c*; outputs registered on that edge are reported at *c*.

## Formats (`vlog.py`)

* **`.stim`**: one camera-stream event per line, 64-bit hex: `[63:40]` cycles since the previous event, `[39]` SW1, `[38]` frame_start, `[37]` valid, `[36:21]` RGB565, `[20:11]` x, `[10:2]` y.
* **`.rec`**: every observable output, one per line. `P` accepted pixel + mask bits, `S`/`E` adapter frame_start/frame_end (+ frame_good), `D`/`Z` adapter diagnostics, `R` led_centroid result (valid, x, y, count, confidence, flags, bounding box), `T` tri-colour results. The model and the RTL bench both write this; the comparison is exact, record by record.
* **`.taplog`**: one `vision_tap` record per line (hex), what `capture.py` normalises every export to.

## Photos over JTAG (the easy way): `golden/framegrab/`

Use this first. Program `golden/framegrab/quartus/vision_grab.qpf` and run
`python golden/framegrab/tools/take_photo.py <name>`. The board sends back a 320 × 240 photo and the robot's
results for that same frame. The photo is then checked against the golden model:
* exact on the board's half-resolution (KEPT) centroid;
* exact again through an RTL replay of the photo;
* approximate on the robot's full-resolution result.

It uses no SignalTap, no project edits and no exports. `golden/framegrab/README.md` has the details;
`framegrab_validation.ipynb` is its notebook.

The SignalTap path below is still useful when the full-resolution stream itself is in question: every red pixel
plus dense rows, and a bit-exact rebuild of the full frame.

## Recording on the DE1-SoC (SignalTap)

`vision_tap` watches the vision chain and raises `tap_store` for one PCLK whenever something is worth recording, with a 56-bit record on `tap_word`. SignalTap stores only those cycles, so a 32K-sample buffer holds about five complete frames.

Per frame it records:
* a FRAME record at every `frame_start`: frame period, SW1, dense-band position, truncated close;
* PIXEL records with the board's verdict (adapter accepted, `is_red`, any close), for:
  * every pixel of 8 dense rows, following the last valid target;
  * every pixel anywhere that is loosely red (R5 ≥ 16, R5 ≥ G5 + 4, R5 ≥ B5 + 4: a strict superset of `red_mask`), or that the board's `red_mask` marked red;
  * every closing pixel;
  * the first 16 dropped pixels;
* a RESULT record at every `frame_done`: valid, x, y, `red_count`, latency since the close, SW1.

Unrecorded pixels are therefore not red for the board **and** not red for the model, so each recorded frame is rebuilt with them black, and the rebuilt frame has exactly the real frame's centroid result. `capture.loose_is_superset()` checks the superset claim for the current thresholds. If anyone lowers `RED_MIN`/`RED_MARGIN` below the loose values, raise `LOOSE_R_MIN`/`LOOSE_MARGIN` in `vision_tap` and in `capture.py` to match.

1. **Quartus**: add `golden/rtl/vision_tap.sv` to the project, and paste `golden/rtl/vision_tap_hookup.svh` into `rtl/integration/major_project_top.sv` after `u_led_centroid`. That's a copy-paste: the tap drives nothing.
2. **SignalTap** (`Tools > Signal Tap Logic Analyzer`, new file `golden.stp`):
   * Clock: `cam_pclk` (the `GPIO_1[21]` net in the top).
   * Data (Setup tab, add nodes, filter *Signal Tap: pre-synthesis*): `u_vision_tap|tap_word_r[55..0]` and `u_vision_tap|tap_store_r`.
   * Sample depth **32K** (16K if the fitter runs out of M10K). RAM type Auto.
   * Storage qualifier: type **Input port**, signal `u_vision_tap|tap_store_r`.
   * Trigger: `u_vision_tap|tap_trigger_r` = 1 (basic AND, trigger position *pre-trigger 0 %* / start).
   * Save, recompile, program the `.sof`, *Run Analysis* with the LED in view. Record runs with SW1 off and on.
3. **Export**: in the data tab, `File > Export…`, choose **VCD** (preferred) or **CSV**. For CSV, set the bus `tap_word_r` radix to hex first.
4. **Convert**: `python golden/capture.py export.vcd -o golden/data/captures/<name>`. This prints the board's per-frame table. Then run `python golden/check_golden.py`, or set `CAPTURE = "data/captures/<name>"` in the notebook.

The parser accepts a 56-bit vector or 56 separate bits, with or without a `tap_store` column, and drops duplicate samples. It is tested on simulated VCD and CSV exports. The exact Quartus 18.1 export layout has **not** been seen yet. If a real export doesn't parse, send it over and the reader will be adjusted.

## Results so far

All on Verilator 5.050:
* **Model vs RTL**: exact on every record:
  * 8.6 M pixels and 28 frame results over the gate/edge/all-codes/near-threshold frames in both SW1 modes;
  * lab-photo frames at camera timing (frame period 799,680 PCLKs);
  * 10 adapter fault cases;
  * 40 random fuzz streams.
* **Simulated board** (the real `major_project_top` + Ed's OV7670 model + `vision_tap`): board == model == RTL on every recorded pixel and every frame result. Latency 35 PCLKs, 50.02 fps, 2-PCLK pixel cadence. Covers SW1 off and on, including two red LEDs: V2 rejects the frame (fill 3/255), and the bypass steers to their midpoint.
* **Capture path with adapter faults** (`check_golden.py`, `capture_faults`): `vision_tap` inside the replay bench, fed a 640×480 stream with a truncated frame and a coordinate error. `capture.py` classifies every close (good / truncated / coordinate), the rejected frames give no target, and the four good frames replay exactly.
* **Broken mask** (`broken/led_colour_mask_broken.sv`): CAUGHT on RGB565 0xA320, the code the original bug got wrong.
