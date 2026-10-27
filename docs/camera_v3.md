# Camera acquisition, V3: reliability during movement and frame-dropout recovery (M1)

Owner: Luke (M1, camera capture and stream interface). Status: RTL and simulation complete; **not yet verified on hardware**.

## Problem

V2 `camera_stream_adapter` rejects malformed frames one at a time, but nothing looked at the stream over time:

- A single rejected frame (motion blur, vibration, a ribbon glitch while the robot reverses) flips `stream_ok` immediately, so LEDR7 flickered and there was no stable "camera healthy" signal.
- If the camera stopped producing frames (hung sensor, ribbon contact lost and restored), nothing restarted it. `ov7670_config` only runs after KEY0 or power-up, so the camera stayed dead until someone pressed reset.

## Design: `rtl/integration/camera_health_monitor.sv`

Runs on CLOCK_50, not PCLK, because PCLK can stop when the camera fails. Frame events arrive as bit 0 of the adapter's `good_frames` / `bad_frames` counters (one change per frame), each through a two-flop synchroniser plus an edge-detect flop. Only single bits cross, so no multi-bit value is ever captured mid-change.

| State | Meaning | Leaves when |
|---|---|---|
| `WAIT_CFG` (0) | camera being configured | `cfg_done` -> ACQUIRE; 200 ms timeout -> RECONFIG |
| `ACQUIRE` (1) | configured, confirming | 3 good frames in a row -> OK; 200 ms without -> RECONFIG |
| `OK` (2) | `camera_ok` = 1 | 100 ms with no frame at all, or 4 rejected frames in a row -> LOST |
| `LOST` (3) | dropout | a good frame -> ACQUIRE; 200 ms without -> RECONFIG (unless SW3) |
| `RECONFIG` (4) | `reconfig` high for 100 us | -> WAIT_CFG |

`reconfig` is synchronised into the SCCB clock domain in `major_project_top` and ORed into `ov7670_config`'s reset. That module then pulses the camera's RESET# pin and writes all 78 registers again (about 35 ms), exactly as after KEY0. A camera that never comes back is retried once per timeout cycle, indefinitely.

Movement tolerance: isolated or paired rejected frames and a single missing frame do not drop `camera_ok`. A rejected frame still proves the stream is alive, so it restarts the silence timer. Only a sustained loss counts as a dropout.

### Interface

| Port | Dir | Description |
|---|---|---|
| `good_toggle_async`, `bad_toggle_async` | in | `cam_good_frames[0]`, `cam_bad_frames[0]` (PCLK domain) |
| `cfg_done` | in | `ov7670_config` done, already synchronised to CLOCK_50 |
| `auto_recover_en` | in | `~SW[3]` synchronised: SW3 ON = bypass, report only |
| `camera_ok` | out | healthy: 3 confirmed good frames and no dropout since |
| `reconfig` | out | restart camera configuration (level, 100 us) |
| `state[2:0]` | out | state above, for SignalTap |
| `good_interval[23:0]` | out | CLOCK_50 cycles between the last two good frames (frame period) |
| `last_outage[27:0]` | out | CLOCK_50 cycles from `camera_ok` falling to returning, last outage |
| `dropouts[7:0]`, `recoveries[7:0]` | out | OK -> LOST transitions; camera reconfigurations (saturating) |

Parameters (CLOCK_50 cycles): `LOST_TIMEOUT` 5,000,000 (100 ms = 5 frames at 50 fps), `MAX_BAD_RUN` 4, `CONFIRM_FRAMES` 3, `RECOVER_TIMEOUT` 10,000,000 (200 ms), `RECONFIG_HOLD` 5,000 (100 us). `major_project_top` exposes `CAM_LOST_TIMEOUT` and `CAM_RECOVER_TIMEOUT` so benches can shorten them; the defaults are the values above.

### Top-level changes

- New instance `u_camera_health`; `ov7670_config` reset = `~rst_n_sccb | cam_reconfig_sccb[1]`.
- **SW3**: camera auto-recovery bypass (ON = bypass), following the SW1/SW2 convention.
- **LEDR7** now shows `camera_ok` (stable, confirmed) instead of the per-frame `stream_ok`.
- `safe_motor_enable` is **unchanged**. Suggested for M4 (Jason): AND `camera_ok` into it, so the robot cannot drive on stale vision during a dropout. It only removes permission.

## Verification (Verilator 5.050)

`./sim/run_camera_tests.sh --full`: all PASS, broken adapter CAUGHT, 2 min 45 s from clean.

| Bench | What it shows | Result |
|---|---|---|
| `sim/unit/camera_health_monitor_tb.sv` | 14 scenarios against a camera + config model on an unrelated clock (14 ns vs 10 ns): start-up, isolated/paired bad frames, one missing frame, bad burst self-recovery, frame gap, lockup recovered by reconfiguration, SW3 bypass, dead camera retried, configuration stuck, reset. Per-cycle invariants: `camera_ok` only in OK, `reconfig` only in RECONFIG, every pulse exactly `RECONFIG_HOLD`, counters equal the bench's own counts (~30,000 checks) | PASS |
| `sim/integration/top_camera_recovery_tb.sv` | Real `major_project_top` driving the Ed OV7670 model over the real pins (SCCB, RESET#, XCLK, video). The bench freezes the video pins; the freeze clears only on a RESET# pulse | PASS |
| `camera_stream_adapter_tb`, `camera_capture_vision_tb` | V2 benches, unchanged | PASS |
| `major_project_top_tb`, `top_r9_bypass_tb`, `top_uart_safety_tb`, `top_uart_latency_tb` | Existing top-level benches with the new top | PASS (same as `main`) |

Mutation check: the unit bench fails on three deliberately broken monitors (never asserts `reconfig`; drops `camera_ok` on one bad frame; ignores SW3).

### Measured in simulation (top_camera_recovery_tb)

| Quantity | Value |
|---|---|
| Configuration + 3 confirmed frames after reset | LEDR7 on at 109.4 ms |
| Frame period (`good_interval`) | 19.99 ms (50 fps) |
| Video stops -> LEDR7 off | 95.0 ms (bound: `LOST_TIMEOUT` 100 ms) |
| RESET# + full SCCB reconfiguration -> LEDR7 on | 84.4 ms |
| Expected worst-case outage on hardware (defaults) | 100 + 200 + ~85 = **~385 ms** |

## Bug found during development (report Q6 candidate)

- **Symptom:** `camera_ok` dropped during two consecutive rejected frames, which should be tolerated (unit bench scenario "isolated / paired bad frames").
- **Cause:** in OK, only a *good* frame restarted the silence timer, so two rejected frames plus the following gap exceeded `LOST_TIMEOUT`, although the stream was alive.
- **Fix:** any frame event restarts the silence timer; consecutive rejected frames are handled separately by `MAX_BAD_RUN`.
- **Regression test:** the paired-bad-frame pattern in scenario 3 of `camera_health_monitor_tb`.

## Hardware checks still to do

1. Program, confirm LEDR7 lights (about 0.1 s after reset) and stays on while the robot drives.
2. Unplug and replug the camera ribbon (robot disarmed): LEDR7 goes off; with SW3 OFF it returns without pressing KEY0. With SW3 ON it stays off until KEY0.
3. In SignalTap, read `cam_good_interval` (frame period) and `cam_last_outage` (recovery time) for the datasheet.

Possible datasheet target: *camera dropout recovery time* (ms), from re-seating the camera ribbon to LEDR7 back on, three trials, worst counts; threshold <= 1000 ms, goal <= 400 ms. Bypass (SW3 ON): no recovery without KEY0.
