`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Major Project - Group 26
// Self-checking testbench for camera_health_monitor (V3 camera reliability).
//
// The monitor runs on `clk` (10 ns). A camera model runs on its own
// unrelated clock `pclk` (14 ns) and produces good/bad frame events the way
// camera_stream_adapter does (bit 0 of its good/bad frame counters), so the
// crossing into the monitor is exercised with genuinely asynchronous clocks.
// A configuration model stands in for ov7670_config: `reconfig` clears
// cfg_done, and cfg_done returns CFG_TIME cycles after `reconfig` falls.
//
// Camera faults the model can produce:
//   pattern queue  per-frame: 0 good, 1 bad (rejected), 2 missing (no frame)
//   lockup         no frames until the camera is reconfigured
//   dead           no frames at all, reconfiguration does not help
//   cfg_stuck      configuration never completes
//
// Checked every cycle (invariants):
//   camera_ok == (state == OK); reconfig == (state == RECONFIG)
//   every reconfig pulse is exactly RECONFIG_HOLD cycles
//   dropouts / recoveries equal the bench's own count of camera_ok falls
//   and reconfig pulses
// Scenarios check the behaviour and measurements, and print PASS.
//
// Run from the repository root (Verilator 5.050):
//   $ verilator --binary --timing --top-module camera_health_monitor_tb \
//       --Mdir build/unit/camera_health_monitor \
//       rtl/integration/camera_health_monitor.sv sim/unit/camera_health_monitor_tb.sv
//   $ ./build/unit/camera_health_monitor/Vcamera_health_monitor_tb

module camera_health_monitor_tb;

    localparam int LOST_TIMEOUT    = 300;
    localparam int MAX_BAD_RUN     = 3;
    localparam int CONFIRM_FRAMES  = 3;
    localparam int RECOVER_TIMEOUT = 1500;
    localparam int RECONFIG_HOLD   = 20;
    localparam int CFG_TIME        = 200;     // clk cycles to configure
    localparam int FRAME_PCLKS     = 100;     // 100 x 14 ns = 140 clk cycles

    localparam logic [2:0] S_WAIT_CFG = 3'd0;
    localparam logic [2:0] S_ACQUIRE  = 3'd1;
    localparam logic [2:0] S_OK       = 3'd2;
    localparam logic [2:0] S_LOST     = 3'd3;
    localparam logic [2:0] S_RECONFIG = 3'd4;

    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic pclk = 1'b0;
    always #7 pclk = ~pclk;

    logic rst_n = 1'b0;
    logic auto_recover_en = 1'b1;

    // ------------------------------------------------------------------
    // Configuration model (clk domain)
    // ------------------------------------------------------------------
    logic cfg_done = 1'b0;
    int   cfg_count = CFG_TIME;
    bit   cfg_stuck = 0;
    wire  reconfig;

    always @(posedge clk) begin
        if (!rst_n) begin
            cfg_done  <= 1'b0;
            cfg_count <= CFG_TIME;
        end else if (reconfig) begin
            cfg_done  <= 1'b0;
            cfg_count <= CFG_TIME;
        end else if (cfg_count > 0) begin
            cfg_count <= cfg_count - 1;
        end else if (!cfg_stuck) begin
            cfg_done <= 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // Camera model (pclk domain)
    // ------------------------------------------------------------------
    logic [15:0] good_frames = 16'd0;
    logic [15:0] bad_frames  = 16'd0;
    bit lockup = 0, dead = 0;
    int pattern[$];
    int frames_sent = 0;

    // Reconfiguration clears a lockup (not a dead camera).
    always @(posedge clk) if (reconfig) lockup = 0;

    initial begin
        forever begin
            repeat (FRAME_PCLKS) @(posedge pclk);
            if (!lockup && !dead) begin
                automatic int kind = 0;
                if (pattern.size() > 0) kind = pattern.pop_front();
                if (kind == 0) good_frames <= good_frames + 1'b1;
                if (kind == 1) bad_frames  <= bad_frames + 1'b1;
                frames_sent++;
            end
        end
    end

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    wire        camera_ok;
    wire [2:0]  state;
    wire [23:0] good_interval;
    wire [27:0] last_outage;
    wire [7:0]  dropouts, recoveries;

    camera_health_monitor #(
        .LOST_TIMEOUT(LOST_TIMEOUT),
        .MAX_BAD_RUN(MAX_BAD_RUN),
        .CONFIRM_FRAMES(CONFIRM_FRAMES),
        .RECOVER_TIMEOUT(RECOVER_TIMEOUT),
        .RECONFIG_HOLD(RECONFIG_HOLD)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .good_toggle_async(good_frames[0]),
        .bad_toggle_async(bad_frames[0]),
        .cfg_done(cfg_done),
        .auto_recover_en(auto_recover_en),
        .camera_ok(camera_ok),
        .reconfig(reconfig),
        .state(state),
        .good_interval(good_interval),
        .last_outage(last_outage),
        .dropouts(dropouts),
        .recoveries(recoveries)
    );

    // ------------------------------------------------------------------
    // Invariants, every cycle
    // ------------------------------------------------------------------
    int  cycle = 0;
    bit  ok_q = 0, rc_q = 0;
    int  rc_len = 0;
    int  ok_falls = 0, rc_pulses = 0;
    int  invariant_checks = 0;
    bit  ok_fell_flag = 0;          // set on any camera_ok fall; scenarios clear it

    always @(negedge clk) begin
        cycle++;
        if (!rst_n) begin
            ok_q = 0; rc_q = 0; rc_len = 0; ok_falls = 0; rc_pulses = 0;
        end else begin
            if (camera_ok !== (state == S_OK))
                $fatal(1, "FAIL invariant: camera_ok=%b in state %0d", camera_ok, state);
            if (reconfig !== (state == S_RECONFIG))
                $fatal(1, "FAIL invariant: reconfig=%b in state %0d", reconfig, state);
            if (state > S_RECONFIG)
                $fatal(1, "FAIL invariant: illegal state %0d", state);

            if (reconfig) rc_len++;
            if (rc_q && !reconfig) begin
                if (rc_len != RECONFIG_HOLD)
                    $fatal(1, "FAIL invariant: reconfig pulse %0d cycles, expected %0d",
                           rc_len, RECONFIG_HOLD);
                rc_len = 0;
            end
            if (!rc_q && reconfig) rc_pulses++;
            if (ok_q && !camera_ok) begin ok_falls++; ok_fell_flag = 1; end

            if (int'(dropouts) != ok_falls || int'(recoveries) != rc_pulses)
                $fatal(1, "FAIL invariant: dropouts=%0d (bench %0d) recoveries=%0d (bench %0d)",
                       dropouts, ok_falls, recoveries, rc_pulses);

            ok_q = camera_ok;
            rc_q = reconfig;
            invariant_checks++;
        end
    end

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    int checks = 0;

    task automatic pass(input string name);
        checks++;
        $display("PASS [%0d]: %s", checks, name);
    endtask

    task automatic wait_cycles(input int n);
        repeat (n) @(negedge clk);
    endtask

    // Wait until camera_ok == v, failing after `limit` cycles. Returns cycles waited.
    task automatic wait_ok(input bit v, input int limit, input string what, output int waited);
        waited = 0;
        while (camera_ok !== v) begin
            @(negedge clk);
            waited++;
            if (waited > limit)
                $fatal(1, "FAIL %s: camera_ok did not become %b within %0d cycles (state %0d)",
                       what, v, limit, state);
        end
    endtask

    task automatic wait_frames(input int n);
        automatic int target = frames_sent + n;
        while (frames_sent < target) @(negedge clk);
        wait_cycles(10);            // let the event cross
    endtask

    // ------------------------------------------------------------------
    // Scenarios
    // ------------------------------------------------------------------
    int w, t0, d0, r0, attempts;

    initial begin
        if ($test$plusargs("wave")) begin
            $dumpfile("camera_health_monitor.fst");
            $dumpvars(0, camera_health_monitor_tb);
        end

        wait_cycles(5);
        rst_n = 1;

        // 1. Start-up: configure, then CONFIRM_FRAMES good frames.
        if (camera_ok) $fatal(1, "FAIL start-up: camera_ok out of reset");
        wait_ok(1, CFG_TIME + (CONFIRM_FRAMES + 2) * 140, "start-up", w);
        if (recoveries != 0 || dropouts != 0)
            $fatal(1, "FAIL start-up: recoveries=%0d dropouts=%0d", recoveries, dropouts);
        pass($sformatf("start-up: camera_ok after configuration + %0d good frames (%0d cycles)",
                       CONFIRM_FRAMES, w));

        wait_frames(2);
        if (good_interval < 24'd138 || good_interval > 24'd142)
            $fatal(1, "FAIL frame interval: %0d cycles, expected 140 +/- 2", good_interval);
        pass($sformatf("good_interval measures the frame period (%0d cycles, true 140)",
                       good_interval));

        // 2. Movement glitches: isolated and paired bad frames.
        ok_fell_flag = 0;
        pattern = '{1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0};
        wait_frames(14);
        if (ok_fell_flag || !camera_ok)
            $fatal(1, "FAIL glitches: camera_ok dropped on isolated bad frames");
        pass("isolated / paired bad frames (motion blur) do not drop camera_ok");

        // 3. One missing frame (gap 280 < LOST_TIMEOUT 300).
        ok_fell_flag = 0;
        pattern = '{2, 0, 0, 2, 0, 0};
        wait_frames(8);
        if (ok_fell_flag || !camera_ok)
            $fatal(1, "FAIL missing frame: camera_ok dropped on one missing frame");
        pass("a single missing frame is tolerated");

        // 4. Burst of MAX_BAD_RUN bad frames: dropout, recovers by itself.
        d0 = int'(dropouts); r0 = int'(recoveries);
        pattern = '{1, 1, 1};
        wait_ok(0, 5 * 140, "bad burst", w);
        if (int'(dropouts) != d0 + 1) $fatal(1, "FAIL bad burst: dropouts=%0d", dropouts);
        wait_ok(1, (CONFIRM_FRAMES + 2) * 140, "bad burst recovery", w);
        if (int'(recoveries) != r0)
            $fatal(1, "FAIL bad burst: camera was reconfigured (recoveries=%0d)", recoveries);
        if (last_outage < 28'd380 || last_outage > 28'd460)
            $fatal(1, "FAIL bad burst: last_outage=%0d, expected about 420", last_outage);
        pass($sformatf("%0d bad frames in a row: dropout, self-recovered without reconfig (outage %0d cycles)",
                       MAX_BAD_RUN, last_outage));

        // 5. Two missing frames (gap 420 > 300): dropout by timeout.
        d0 = int'(dropouts);
        pattern = '{2, 2};
        wait_ok(0, 4 * 140, "frame gap", w);
        wait_ok(1, (CONFIRM_FRAMES + 2) * 140, "frame gap recovery", w);
        if (int'(dropouts) != d0 + 1) $fatal(1, "FAIL frame gap: dropouts=%0d", dropouts);
        pass("frames stopping for > LOST_TIMEOUT is a dropout; recovers when frames return");

        // 6. Camera lockup: only a reconfiguration brings it back.
        r0 = int'(recoveries);
        t0 = cycle;
        lockup = 1;
        wait_ok(0, LOST_TIMEOUT + 300, "lockup", w);
        wait_ok(1, RECOVER_TIMEOUT + RECONFIG_HOLD + CFG_TIME + (CONFIRM_FRAMES + 3) * 140 + 100,
                "lockup recovery", w);
        if (int'(recoveries) != r0 + 1)
            $fatal(1, "FAIL lockup: recoveries=%0d, expected %0d", recoveries, r0 + 1);
        if (last_outage < 28'(RECOVER_TIMEOUT + RECONFIG_HOLD + CFG_TIME))
            $fatal(1, "FAIL lockup: last_outage=%0d too short", last_outage);
        pass($sformatf("camera lockup: reconfigured once, camera_ok back after %0d cycles (outage %0d)",
                       cycle - t0, last_outage));

        // 7. Bypass (SW3 ON): report only, never reconfigure.
        auto_recover_en = 0;
        r0 = int'(recoveries);
        lockup = 1;
        wait_ok(0, LOST_TIMEOUT + 300, "bypass lockup", w);
        wait_cycles(4 * RECOVER_TIMEOUT);
        if (int'(recoveries) != r0 || state != S_LOST)
            $fatal(1, "FAIL bypass: recoveries=%0d state=%0d", recoveries, state);
        pass("bypass: stream lost, no reconfiguration in 4 x RECOVER_TIMEOUT");
        lockup = 0;
        wait_ok(1, (CONFIRM_FRAMES + 2) * 140, "bypass return", w);
        pass("bypass: camera_ok returns when frames return by themselves");
        auto_recover_en = 1;

        // 8. Dead camera: retried once per timeout, indefinitely.
        r0 = int'(recoveries);
        dead = 1;
        wait_ok(0, LOST_TIMEOUT + 300, "dead camera", w);
        wait_cycles(10_000);
        attempts = int'(recoveries) - r0;
        if (attempts < 4 || attempts > 7 || camera_ok)
            $fatal(1, "FAIL dead camera: %0d reconfigurations in 10000 cycles", attempts);
        pass($sformatf("dead camera: %0d reconfiguration attempts in 10000 cycles, camera_ok stays 0",
                       attempts));
        dead = 0;
        wait_ok(1, 2 * RECOVER_TIMEOUT + 2000, "dead camera returns", w);
        pass("camera that comes back is accepted again");

        // 9. Configuration never completes: WAIT_CFG times out and retries.
        r0 = int'(recoveries);
        cfg_stuck = 1;
        lockup = 1;
        wait_cycles(LOST_TIMEOUT + RECOVER_TIMEOUT + 500);
        if (state != S_WAIT_CFG || int'(recoveries) != r0 + 1)
            $fatal(1, "FAIL cfg stuck: state=%0d recoveries=%0d", state, recoveries);
        wait_cycles(RECOVER_TIMEOUT + 200);
        if (int'(recoveries) != r0 + 2)
            $fatal(1, "FAIL cfg stuck: WAIT_CFG did not retry (recoveries=%0d)", recoveries);
        pass("configuration stuck: WAIT_CFG times out and reconfigures again");
        cfg_stuck = 0;
        wait_ok(1, RECOVER_TIMEOUT + 2 * CFG_TIME + 1000, "cfg unstuck", w);
        pass("camera_ok after configuration completes");

        // 10. Reset in operation clears everything.
        @(negedge clk) rst_n = 0;
        wait_cycles(3);
        if (camera_ok || reconfig || dropouts != 0 || recoveries != 0 ||
            state != S_WAIT_CFG || last_outage != 0)
            $fatal(1, "FAIL reset: state not cleared");
        @(negedge clk) rst_n = 1;
        wait_ok(1, CFG_TIME + (CONFIRM_FRAMES + 2) * 140, "after reset", w);
        pass("reset clears counters; normal start-up afterwards");

        $display("-----------------------------------------------");
        $display("PASS: camera_health_monitor_tb (%0d scenario checks, %0d cycle checks)",
                 checks, invariant_checks);
        $display("-----------------------------------------------");
        $finish;
    end

    initial begin
        #2ms;
        $fatal(1, "FAIL: timeout");
    end

endmodule

`default_nettype wire
