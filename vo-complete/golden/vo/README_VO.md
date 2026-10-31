# Beacon visual odometry (Group 26, Wilson, M2)

The robot measures where it is, and how fast it is moving, from the three bay LEDs. It uses no extra
sensor, no frame buffer and no RAM. The V4 behaviour it makes possible: after docking, reverse until the
robot is a set distance from the bay's LED, instead of reversing for a fixed time. SW9 is the bypass: it
switches back to the timed V3 reverse.

```
camera (PCLK)  camera_stream_adapter -> beacon_detect: R, G, B blobs -> centroids in 1/16 px
                                                     |  vo_mailbox_cdc (one result a frame, whole)
CLOCK_50       beacon_geometry: each LED's forward and lateral mm, the pair check, the heading
               -> beacon_tracker: the bay's LED: range, velocity, lateral, displacement (alpha-beta)
               -> vo_standoff: "at the set distance" for the sequencer
```

| File | What |
|---|---|
| `rtl/vo/beacon_vo.sv` | the whole chain, one instance (ports below) |
| `rtl/vo/beacon_detect.sv` | the colour mask and run filter (the existing V2 modules), then per colour: count, coordinate sums and box; one shared divider; gates |
| `rtl/vo/beacon_geometry.sv` | pixels to mm, no trig at run time |
| `rtl/vo/beacon_tracker.sv` | alpha-beta filter using shifts only |
| `rtl/vo/vo_standoff.sv` | the V4 decision |
| `rtl/vo/vo_divider.sv`, `vo_mailbox_cdc.sv` | a shared sequential divider; the toggle-handshake CDC |
| `rtl/vo/vo_hex_display.sv` | range (or velocity) on HEX3..0, the LED's letter on HEX5 |
| `rtl/vo/vo_calibration.svh` | the camera calibration, written by `vo_calibrate.py` |
| `golden/vo/vo_model.py` | the golden model, bit-exact |
| `golden/vo/vo_scene.py` | draws the LEDs from any robot pose (exact ground truth) |
| `golden/vo/vo_check.py` | RTL vs model (fuzz and drive scenes), accuracy vs truth, `--broken` |
| `golden/framegrab/rtl/vo_grab_top.sv`, `quartus/vo_grab.qpf` | the bench build: the VO, HEX readout, VGA crosses, JTAG photo |
| `golden/vo/vo_take_photo.py`, `vo_validate_grab.py`, `vo_calibrate.py`, `vo_grab_sim.py` | photos, the hardware check, calibration, the bench build simulated |
| `golden/vo/vo_golden.ipynb` | the notebook: predicted performance, RTL vs model, hardware photos |

## How it measures

The LEDs are DH = 150 mm above the camera lens: 450 mm LEDs, 300 mm camera. The camera is tilted up by
the pitch angle. An LED at pixel (x, y) is seen at an elevation angle, and the elevation of something at a
known height gives its distance. In integer form, with no trigonometry at run time:

```
U = x - cx,  V = cy - y                        (1/16 px)
up = V cos(pitch) + f sin(pitch)               (Q4 x Q14)
fw = f cos(pitch) - V sin(pitch)
forward = DH * fw / up,   lateral = DH * U / up      (mm)
```

This is exact pinhole geometry. The model and RTL error is only the centroid's quantisation: under ±5 mm
in simulation. Each LED gives its own position. Two LEDs give heading:
`tan(heading) = -dForward/dLateral`. They also give a consistency check: the two LEDs must come out 300 mm
apart. One LED lit is enough for range, so it works when the ESP32 lights only the bay's LED.

Sensitivity is about distance² / (DH × f) mm per pixel. With f ≈ 560 px that is about 3 mm per pixel at
0.5 m, 12 at 1 m and 27 at 1.5 m. Centroids are sub-pixel, so real errors are smaller, but beyond about
1.5 m the elevation angle is too small to give a good range.

**Tilt the camera up.** Docked at 250 mm, the LED sits about 31° up. A level camera sees only about 23° up
if f ≈ 560, less with a narrower lens. Tilt about 12° and calibrate.

## Measured performance (fill from hardware)

| Target | Threshold | Goal | Predicted (sim) | Measured |
|---|---|---|---|---|
| Range error, 0.4–1.5 m, three tutor-chosen tape positions, worst counts | ≤ 80 mm | ≤ 30 mm | ≤ 5 mm (`vo_check`) | `vo_take_photo` + notebook |
| Stand-off spread, three reverses from the dock, largest distance between end positions | ≤ 100 mm | ≤ 50 mm | stand-off decision 2 frames after the target (40 ms; about 12 mm at 0.3 m/s) | floor marks, SW9 off vs on |
| VO latency, frame end to range out | ≤ 2 frames | ≤ 1 ms | about 12 µs (210 PCLK + 330 CLOCK_50 clocks) | SignalTap |

## Bench use: before the VO drives the robot

1. Compile `golden/framegrab/quartus/vo_grab.qpf`. It is the frame-grab build with the VO, HEX readout and
   VGA crosses. Program it.
2. Set SW[5:4] to the LED to track: 0 red, 1 green, 2 blue. HEX shows `G 0812`, which is green at 812 mm.
   SW2 up shows velocity in mm/s instead. SW9 up holds exposure and white balance.
3. **Calibrate.** Light all three LEDs, square the robot up, then take photos at four to six tape-measured
   spots between 0.4 and 1.5 m, some of them off-centre. Measure from the lens to the LED wall:

   ```
   python golden/vo/vo_take_photo.py cal_500  --Y 500
   python golden/vo/vo_take_photo.py cal_800  --Y 800 --X 200
   python golden/vo/vo_take_photo.py cal_1200 --Y 1200
   ...
   python golden/vo/vo_calibrate.py golden/framegrab/grabs/cal_* --write
   ```

   This rewrites `rtl/vo/vo_calibration.svh`. Recompile `vo_grab` and the robot. The fit prints the
   reprojection error and each photo's range error before and after: those go in the
   simulation-to-hardware section of the report.
4. **Check accuracy** at new spots, not the calibration spots:
   `vo_take_photo.py test_700 --Y 700`. Each photo is checked:
   - KEPT: exact, the board's detection against the model, on the photo;
   - GEOM: exact, the board's mm against the model, from the board's own centroids;
   - against the tape.

## Integrating into `major_project_top` (Jason and Wilson)

1. **Instance.** Add `rtl/vo/*.sv` (minus `vo_hex_display` unless you add HEX pins) to the qsf, with
   `SEARCH_PATH` set to `rtl/vo`. After the adapter:

   ```systemverilog
   `include "vo_calibration.svh"
   (* async_reg = "true" *) logic [1:0] vo_bypass_sync;          // SW9: 1 = timed V3 reverse
   always_ff @(posedge CLOCK_50 or negedge rst_n_50)
       if (!rst_n_50) vo_bypass_sync <= 2'b00; else vo_bypass_sync <= {vo_bypass_sync[0], SW[9]};
   wire vo_reached, vo_lost, vo_track_valid;
   wire signed [15:0] vo_range_mm, vo_vel_mm_s, vo_lateral_mm;
   beacon_vo #(.F_Q4(`VO_F_Q4), .CX_Q4(`VO_CX_Q4), .CY_Q4(`VO_CY_Q4), .SIN_Q14(`VO_SIN_Q14),
               .COS_Q14(`VO_COS_Q14), .DH_MM(`VO_DH_MM)) u_vo (
       .pclk(cam_pclk), .rst_n_pclk(rst_n_cam),
       .frame_start(frame_start), .frame_end(frame_end), .frame_good(cam_frame_good),
       .pixel_valid(pixel_valid), .pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y),
       .dbg_mask(),
       .clk(CLOCK_50), .rst_n(rst_n_50), .sel(seq_bay_index),
       .standoff_enable(seq_state == 4'd6),        // S_REV_RUN
       .standoff_approaching(1'b0), .standoff_target_mm(16'd900),
       .vo_updated(), .track_valid(vo_track_valid), .track_settled(),
       .range_mm(vo_range_mm), .vel_mm_s(vo_vel_mm_s), .lateral_mm(vo_lateral_mm), .disp_mm(),
       .standoff_reached(vo_reached), .standoff_lost(vo_lost),
       .led_valid(), .led_forward_mm(), .led_lateral_mm(), .pair_valid(), .pair_ok(), .heading_q12(),
       .det_dropped(), .cdc_overrun(), .track_coast(),
       .det_done(), .det_valid(), .det_x_q4(), .det_y_q4(), .det_count(), .det_flags());
   ```

   SW9 currently switches the temporal debug LEDs. Move that to a KEY, or drop it.

2. **Sequencer** (`multi_bay_sequencer.sv`). Add inputs `vo_mode` (`!vo_bypass_sync[1]`), `vo_reached` and
   `vo_lost`, and a `REVERSE_MAX_MS` parameter below the motion supervisor's 1500 ms cap (1400, say). In
   `S_REV_RUN`, replace `else if (timer >= REV_CYC)` with:

   ```systemverilog
   else if ((vo_mode && !vo_lost) ? (vo_reached || timer >= REV_MAX_CYC) : (timer >= REV_CYC))
       next_state = S_REV_STOP;
   ```

   This gives three behaviours:
   - SW9 up: exactly the old timed reverse.
   - VO lost mid-reverse: falls back to the timer.
   - Every case: the supervisor's own cap and the IR stop still apply.

3. **Tests.** Keep `top_multi_bay_tb` passing with SW9 up (unchanged behaviour). Add a case with
   `vo_reached` forced, then one with `vo_lost`. `golden/vo/vo_check.py` covers the VO itself.

**Docking to green or blue.** The docking path today is `red_mask` → `led_centroid`: red only. With R, G
and B bays, docking needs the selected colour too. Either select the colour in `red_mask`'s
`TARGET_COLOUR` by bay, or steer on `vo_lateral_mm` of the selected LED, which is already in mm.

## Resources (estimated; Quartus has the final word)

- No M10K, no frame buffer.
- About 1,500 ALMs: three colours of 19/28/28-bit sums plus boxes, two sequential dividers (32/19 and
  44/32 bits), the tracker.
- About 6–8 DSP blocks: the 48-bit products and the pair squares.
- At 40 MHz PCLK and 50 MHz, the critical paths are the 44-bit divider's subtract and the 48-bit products.
  Both are registered. If timing is tight, the products can take an extra register stage.

## Limits, honestly

- **Colour thresholds.** Range depends on the colour mask finding the LED. The thresholds are the V2 lab
  values, and green and blue have not been tuned on real LEDs yet. Tune them on photos from the bench
  build.
- **LED in frame.** The robot needs the LED in frame: tilt, and keep within about 1.5 m. A clipped blob
  (touching the frame edge) is rejected on purpose, because its centroid is biased.
- **Elevation geometry.** Range assumes the floor is flat and the LEDs are at the measured height. A
  10 mm error in DH is a 7 % range error. Calibration fits DH from the photos.
- **Velocity.** Velocity is the alpha-beta filter's: it lags a sudden change by a few frames. The
  sequencer uses range, not velocity.
- **Generic visual odometry.** Generic VO (feature flow) is not here: it is the redundancy step after the
  HD claim is secured.

## OWNERS.md lines (Wilson, M2)

| File | Owner | Origin |
|---|---|---|
| `rtl/vo/beacon_vo.sv`, `beacon_detect.sv`, `beacon_geometry.sv`, `beacon_tracker.sv`, `vo_standoff.sv`, `vo_divider.sv`, `vo_mailbox_cdc.sv`, `vo_hex_display.sv`, `vo_calibration.svh` | Wilson | new |
| `sim/vo/beacon_vo_scene_tb.sv`, `beacon_math_tb.sv`, `sim/vo/broken/beacon_geometry_broken.sv` | Wilson | new |
| `golden/vo/*` | Wilson | new |
| `golden/framegrab/rtl/vo_grab_top.sv`, `vo_grab_results.sv`, `sim/vo_grab_top_tb.sv`, `quartus/vo_grab.*` | Wilson | new; adapted from FrameGrabJtag V2026.3's `top_level_jtag` and the course camera project |
| `rtl/image_processing/led_colour_mask.sv`, `colour_run_filter.sv` (reused) | Wilson | V2, unchanged |
