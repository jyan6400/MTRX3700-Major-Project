`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Major Project - Group 26
// camera_health_monitor (V3, M1 camera): camera reliability during movement
// and automatic recovery from frame dropouts.
//
// Clock domain: CLOCK_50 (any free-running clock). It must NOT run on the
// camera PCLK: when the camera stalls, PCLK may stop, and the monitor still
// has to notice and act.
//
// Inputs from camera_stream_adapter (PCLK domain):
//   good_toggle_async  bit 0 of good_frames: changes once per good frame
//   bad_toggle_async   bit 0 of bad_frames:  changes once per rejected frame
// Each is a single bit that changes at most once per frame (milliseconds
// apart), so a two-flop synchroniser per bit is a safe crossing; no
// multi-bit value crosses. A third flop gives the edge (= one frame event).
//
//   cfg_done           ov7670_config done, already synchronised to clk
//   auto_recover_en    1: reconfigure the camera when the stream is lost
//                      0: bypass (R9): report only, never reconfigure
//
// States:
//   WAIT_CFG  camera being configured; waits for cfg_done
//   ACQUIRE   configured; needs CONFIRM_FRAMES good frames in a row
//   OK        camera_ok = 1. Tolerates isolated bad/missing frames
//             (motion blur, vibration, a glitch on the ribbon):
//             only LOST_TIMEOUT with no frame at all (stream silent), or
//             MAX_BAD_RUN rejected frames in a row, count as a dropout
//   LOST      dropout. If good frames return by themselves it goes back
//             through ACQUIRE; if none arrive within RECOVER_TIMEOUT it
//             reconfigures the camera (when auto_recover_en)
//   RECONFIG  holds `reconfig` high for RECONFIG_HOLD cycles. The top level
//             synchronises it into the SCCB domain and ORs it into
//             ov7670_config's reset, which pulses the camera's RESET# and
//             writes every register again (about 35 ms)
// WAIT_CFG and ACQUIRE also time out after RECOVER_TIMEOUT and reconfigure
// again, so a camera that never comes back is retried indefinitely,
// one attempt per RECOVER_TIMEOUT.
//
// Measurements (for SignalTap / HEX / datasheet):
//   good_interval  cycles between the last two good frames (frame period)
//   last_outage    cycles from losing camera_ok to regaining it, last outage
//   dropouts       number of OK -> LOST transitions      (saturating)
//   recoveries     number of camera reconfigurations    (saturating)
//
// Assumption: RECONFIG_HOLD is longer than the synchroniser delay into the
// SCCB domain plus one SCCB clock (3 x 10 us at 100 kHz), so cfg_done has
// gone low before WAIT_CFG starts looking for it. The default is 100 us.
//
// Defaults are for CLOCK_50 and a 50 frame/s camera (20 ms per frame).

module camera_health_monitor #(
    parameter int LOST_TIMEOUT    = 5_000_000,   // 100 ms = 5 frame periods
    parameter int MAX_BAD_RUN     = 4,           // bad frames in a row
    parameter int CONFIRM_FRAMES  = 3,           // good frames in a row
    parameter int RECOVER_TIMEOUT = 10_000_000,  // 200 ms
    parameter int RECONFIG_HOLD   = 5_000,       // 100 us
    parameter int INTERVAL_W      = 24,          // 335 ms at 50 MHz
    parameter int OUTAGE_W        = 28           // 5.3 s at 50 MHz
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        good_toggle_async,
    input  wire        bad_toggle_async,
    input  wire        cfg_done,
    input  wire        auto_recover_en,

    output logic       camera_ok,
    output logic       reconfig,
    output logic [2:0] state,

    output logic [INTERVAL_W-1:0] good_interval,
    output logic [OUTAGE_W-1:0]   last_outage,
    output logic [7:0]            dropouts,
    output logic [7:0]            recoveries
);

    localparam logic [2:0] S_WAIT_CFG = 3'd0;
    localparam logic [2:0] S_ACQUIRE  = 3'd1;
    localparam logic [2:0] S_OK       = 3'd2;
    localparam logic [2:0] S_LOST     = 3'd3;
    localparam logic [2:0] S_RECONFIG = 3'd4;

    // Wide enough for the longest timer.
    localparam int MAX_TL = (RECOVER_TIMEOUT > LOST_TIMEOUT)
                          ? RECOVER_TIMEOUT : LOST_TIMEOUT;
    localparam int MAX_T  = (MAX_TL > RECONFIG_HOLD) ? MAX_TL : RECONFIG_HOLD;
    localparam int TW = $clog2(MAX_T + 2);
    localparam int RW = (MAX_BAD_RUN > CONFIRM_FRAMES)
                      ? $clog2(MAX_BAD_RUN + 1) : $clog2(CONFIRM_FRAMES + 1);

    // ------------------------------------------------------------------
    // Frame events from the PCLK domain
    // ------------------------------------------------------------------
    (* async_reg = "true" *) logic [1:0] good_sync;
    (* async_reg = "true" *) logic [1:0] bad_sync;
    logic good_q, bad_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            good_sync <= 2'b00;
            bad_sync  <= 2'b00;
            good_q    <= 1'b0;
            bad_q     <= 1'b0;
        end else begin
            good_sync <= {good_sync[0], good_toggle_async};
            bad_sync  <= {bad_sync[0],  bad_toggle_async};
            good_q    <= good_sync[1];
            bad_q     <= bad_sync[1];
        end
    end

    wire good_evt = good_sync[1] ^ good_q;
    wire bad_evt  = bad_sync[1]  ^ bad_q;

    // ------------------------------------------------------------------
    // Supervisor
    // ------------------------------------------------------------------
    logic [TW-1:0]         timer;        // cycles in this state / since last good frame
    logic [RW-1:0]         run;          // good run (ACQUIRE) or bad run (OK)
    logic [INTERVAL_W-1:0] since_good;   // cycles since the last good frame
    logic                  seen_good;    // a good frame since reset
    logic [OUTAGE_W-1:0]   outage;       // cycles since camera_ok fell

    wire timer_max = (timer == '1);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_WAIT_CFG;
            camera_ok     <= 1'b0;
            reconfig      <= 1'b0;
            timer         <= '0;
            run           <= '0;
            since_good    <= '0;
            seen_good     <= 1'b0;
            outage        <= '0;
            good_interval <= '0;
            last_outage   <= '0;
            dropouts      <= 8'd0;
            recoveries    <= 8'd0;
        end else begin
            if (!timer_max)
                timer <= timer + 1'b1;

            // Frame period, measured on every good frame in any state.
            if (good_evt) begin
                if (seen_good)
                    good_interval <= since_good;
                since_good <= INTERVAL_W'(1);
                seen_good  <= 1'b1;
            end else if (since_good != '1) begin
                since_good <= since_good + 1'b1;
            end

            // Outage length: counts while camera_ok is low after the
            // first OK (start-up is not an outage).
            if (!camera_ok && outage != '0 && outage != '1)
                outage <= outage + 1'b1;

            case (state)
                S_WAIT_CFG: begin
                    if (cfg_done) begin
                        state <= S_ACQUIRE;
                        timer <= '0;
                        run   <= '0;
                    end else if (timer >= TW'(RECOVER_TIMEOUT) && auto_recover_en) begin
                        state      <= S_RECONFIG;
                        timer      <= '0;
                        reconfig   <= 1'b1;
                        if (recoveries != 8'hFF) recoveries <= recoveries + 1'b1;
                    end
                end

                S_ACQUIRE: begin
                    if (good_evt) begin
                        timer <= '0;
                        if (run == RW'(CONFIRM_FRAMES - 1)) begin
                            state     <= S_OK;
                            camera_ok <= 1'b1;
                            run       <= '0;
                            if (outage != '0)
                                last_outage <= outage;
                            outage <= '0;
                        end else begin
                            run <= run + 1'b1;
                        end
                    end else if (bad_evt) begin
                        run <= '0;
                    end else if (timer >= TW'(RECOVER_TIMEOUT) && auto_recover_en) begin
                        state      <= S_RECONFIG;
                        timer      <= '0;
                        reconfig   <= 1'b1;
                        if (recoveries != 8'hFF) recoveries <= recoveries + 1'b1;
                    end
                end

                S_OK: begin
                    if (good_evt) begin
                        timer <= '0;
                        run   <= '0;
                    end else if ((bad_evt && run == RW'(MAX_BAD_RUN - 1)) ||
                                 timer >= TW'(LOST_TIMEOUT)) begin
                        state     <= S_LOST;
                        camera_ok <= 1'b0;
                        timer     <= '0;
                        run       <= '0;
                        outage    <= OUTAGE_W'(1);
                        if (dropouts != 8'hFF) dropouts <= dropouts + 1'b1;
                    end else if (bad_evt) begin
                        // A rejected frame still shows the stream is
                        // alive: restart the silence timer, count the run.
                        run   <= run + 1'b1;
                        timer <= '0;
                    end
                end

                S_LOST: begin
                    if (good_evt) begin
                        // Stream came back by itself.
                        state <= S_ACQUIRE;
                        timer <= '0;
                        run   <= RW'(1);
                        if (CONFIRM_FRAMES == 1) begin
                            state       <= S_OK;
                            camera_ok   <= 1'b1;
                            run         <= '0;
                            last_outage <= outage;
                            outage      <= '0;
                        end
                    end else if (timer >= TW'(RECOVER_TIMEOUT) && auto_recover_en) begin
                        state      <= S_RECONFIG;
                        timer      <= '0;
                        reconfig   <= 1'b1;
                        if (recoveries != 8'hFF) recoveries <= recoveries + 1'b1;
                    end
                end

                S_RECONFIG: begin
                    if (timer >= TW'(RECONFIG_HOLD - 1)) begin
                        state    <= S_WAIT_CFG;
                        reconfig <= 1'b0;
                        timer    <= '0;
                    end
                end

                default: begin
                    state     <= S_WAIT_CFG;
                    camera_ok <= 1'b0;
                    reconfig  <= 1'b0;
                    timer     <= '0;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
