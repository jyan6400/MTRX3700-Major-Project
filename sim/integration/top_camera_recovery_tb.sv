`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Major Project - Group 26
// Top-level camera dropout recovery test (V3, M1 camera).
//
// The real major_project_top drives the Ed OV7670 model over the real pins:
// XCLK, RESET#, PWDN and SCCB from ov7670_config, video back on PCLK, VSYNC,
// HREF and D[7:0]. Nothing inside the design is forced.
//
// Fault: a camera lockup. The bench freezes the video pins (PCLK, VSYNC,
// HREF, D held low), as a camera that has hung or a ribbon that has lost
// contact would. The lockup clears only when the camera's RESET# pin is
// pulsed, so the camera comes back only if the design reconfigures it.
//
// Checks:
//   1. start-up: SCCB configuration, then LEDR7 (camera_ok) lights
//   2. lockup: LEDR7 goes out, the design pulses RESET#, reconfigures the
//      camera over SCCB, and LEDR7 lights again; one dropout, one recovery
//   3. SW3 ON (bypass): lockup is reported but RESET# is never pulsed;
//      SW3 OFF again: the design recovers the camera
//
// Timeouts are shortened (LOST 100 ms as on the board, RECOVER 100 ms instead
// of 200 ms) to keep the run short.

module top_camera_recovery_tb;

    logic CLOCK_50 = 1'b0;
    always #10 CLOCK_50 = ~CLOCK_50;

    logic [3:0] KEY = 4'b1110;
    logic [9:0] SW  = 10'd0;
    logic IRDA_RXD  = 1'b1;
    wire  [9:0] LEDR;
    tri   [35:0] GPIO_1;

    // ---------------- camera on the ribbon, with a fault switch ----------
    wire m_pclk, m_vsync, m_href;
    wire [7:0] m_d;
    bit  lockup = 0;

    assign GPIO_1[21]    = lockup ? 1'b0 : m_pclk;
    assign GPIO_1[22]    = lockup ? 1'b0 : m_href;
    assign GPIO_1[23]    = lockup ? 1'b0 : m_vsync;
    assign GPIO_1[19:12] = lockup ? 8'd0 : m_d;
    assign GPIO_1[29]    = 1'b0;                 // sonar echo idle
    pullup(GPIO_1[24]);                          // SCCB data, open drain

    int frames_done, write_count, error_count;

    ov7670_model #(.VERBOSE(0)) cam (
        .sio_c(GPIO_1[25]), .sio_d(GPIO_1[24]), .xclk(GPIO_1[20]),
        .reset_n(GPIO_1[11]), .pwdn(GPIO_1[10]),
        .pclk(m_pclk), .vsync(m_vsync), .href(m_href), .d(m_d),
        .scene_mode(1), .scene_x0(0), .scene_y0(0), .scene_code(8'hA5),
        .frames_done(frames_done), .write_count(write_count), .error_count(error_count)
    );

    // A pulse on RESET# clears the lockup (the camera restarts).
    int reset_pulses = 0;
    always @(negedge GPIO_1[11]) begin
        if (KEY[0]) reset_pulses++;
        lockup = 0;
    end

    major_project_top #(
        .CAMERA_FPS(50),
        .UART_PERIOD_MS(1),
        .CAM_LOST_TIMEOUT(5_000_000),
        .CAM_RECOVER_TIMEOUT(5_000_000)
    ) dut (
        .CLOCK_50(CLOCK_50),
        .KEY(KEY),
        .SW(SW),
        .IRDA_RXD(IRDA_RXD),
        .GPIO_1(GPIO_1),
        .LEDR(LEDR)
    );

    // ---------------- helpers ----------------
    int checks = 0;

    function automatic real ms(input longint cycles);
        return real'(cycles) / 50_000.0;
    endfunction

    task automatic pass(input string name);
        checks++;
        $display("PASS [%0d]: %s", checks, name);
    endtask

    // Wait for LEDR7 == v, at most limit_ms milliseconds.
    task automatic wait_led7(input bit v, input int limit_ms, input string what);
        int polls = 0;
        while (LEDR[7] !== v) begin
            #10us;
            polls++;
            if (polls > limit_ms * 100)
                $fatal(1, "FAIL %s: LEDR7 not %b within %0d ms (health state %0d, cfg_done %b)",
                       what, v, limit_ms, dut.cam_health_state, dut.cfg_done);
        end
    endtask

    realtime t0;
    int r0, d0, p0;

    initial begin
        if ($test$plusargs("wave")) begin
            $dumpfile("top_camera_recovery.fst");
            $dumpvars(1, dut.u_camera_health);
        end

        #200ns KEY[0] = 1'b0;
        #30us  KEY[0] = 1'b1;

        // 1. Start-up.
        wait_led7(1, 300, "start-up");
        if (error_count != 0)
            $fatal(1, "FAIL start-up: camera model reported %0d SCCB errors", error_count);
        if (dut.cam_recoveries != 0 || dut.cam_dropouts != 0)
            $fatal(1, "FAIL start-up: recoveries=%0d dropouts=%0d",
                   dut.cam_recoveries, dut.cam_dropouts);
        #25ms;
        pass($sformatf("start-up: configured over SCCB (%0d writes), LEDR7 on at %.1f ms, frame period %.2f ms",
                       write_count, $realtime / 1ms, ms(longint'(dut.cam_good_interval))));

        // 2. Lockup recovered by the design.
        r0 = dut.cam_recoveries; d0 = dut.cam_dropouts; p0 = reset_pulses;
        t0 = $realtime;
        lockup = 1;
        wait_led7(0, 150, "lockup detected");
        $display("       LEDR7 off %.1f ms after the video stopped", ($realtime - t0) / 1ms);
        wait_led7(1, 400, "lockup recovery");
        if (reset_pulses != p0 + 1 || dut.cam_recoveries != r0 + 1 || dut.cam_dropouts != d0 + 1)
            $fatal(1, "FAIL lockup: RESET# pulses=%0d recoveries=%0d dropouts=%0d",
                   reset_pulses - p0, dut.cam_recoveries - r0, dut.cam_dropouts - d0);
        if (error_count != 0)
            $fatal(1, "FAIL lockup: SCCB errors during reconfiguration: %0d", error_count);
        pass($sformatf("lockup: RESET# pulsed once, camera reconfigured, LEDR7 back after %.1f ms (outage %.1f ms)",
                       ($realtime - t0) / 1ms, ms(longint'(dut.cam_last_outage))));

        // 3. Bypass: report only.
        #25ms;
        SW[3] = 1'b1;
        r0 = dut.cam_recoveries; p0 = reset_pulses;
        lockup = 1;
        wait_led7(0, 150, "bypass lockup detected");
        #300ms;
        if (reset_pulses != p0 || dut.cam_recoveries != r0 || LEDR[7] !== 1'b0)
            $fatal(1, "FAIL bypass: camera was reconfigured (pulses %0d)", reset_pulses - p0);
        pass("SW3 ON: lockup reported (LEDR7 off), camera not reconfigured for 300 ms");

        SW[3] = 1'b0;
        t0 = $realtime;
        wait_led7(1, 300, "bypass released");
        if (reset_pulses != p0 + 1)
            $fatal(1, "FAIL bypass release: RESET# pulses=%0d", reset_pulses - p0);
        pass($sformatf("SW3 OFF: camera reconfigured, LEDR7 back after %.1f ms", ($realtime - t0) / 1ms));

        $display("-----------------------------------------------");
        $display("PASS: top_camera_recovery_tb (%0d checks)", checks);
        $display("-----------------------------------------------");
        $finish;
    end

endmodule

`default_nettype wire
