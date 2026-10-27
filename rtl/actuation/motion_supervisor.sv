`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// MTRX3700 Major Project - Group 26 - V3 (repeated multi-bay docking)
// Workstream M5: sensors, safety and chassis actuation (Advay Hassan)
//
// motion_supervisor: the single final gate between navigation and the
// chassis UART, adding supervised, bounded reverse for the V3 timed retreat.
//
// Position in the design (CLOCK_50 domain):
//
//   motion_interface (forward demand) --+
//   V3 navigation FSM (reverse demand) -+--> [motion_supervisor] --> chassis_drive --> UGV02
//   IR latch, SW0 arm, camera watchdog,-+
//   sonar_filter (valid / forward_clear / distance)
//
// It replaces the inline safe_motor_enable / chassis_left / chassis_right
// logic of the V2 top level and keeps those wire names for the existing
// top-level benches.
//
// ---------------------------------------------------------------------------
// Rules, in priority order (each is combinational from registered state, so
// a stop reaches chassis_drive in the same clock as its cause)
// ---------------------------------------------------------------------------
//   1. STOP dominates every direction: reset, SW0 disarm (arm = 0), the
//      latched IR emergency stop or motion_permitted = 0 force zero output
//      and enable = 0. A stop also ends any reverse session.
//
//   2. FORWARD is exactly the V2 rule: fwd_enable && camera_target_fresh &&
//      sonar_valid && sonar_forward_clear (V2 sonar_filter, 250 mm). The
//      forward channel must be non-negative: a negative forward PWM is an
//      unauthorised reverse, refused and reported (UNAUTHORISED). Forward is
//      also refused while rev_request is high.
//
//   3. REVERSE needs an explicit, live authorisation:
//        * a rising edge of rev_request starts ONE session (1 clock later);
//          holding it high cannot restart a session that has ended, and a
//          request held high through a reset does not count as an edge;
//        * reverse_enable (operator switch: "rear corridor cleared") is on;
//        * sonar_valid - invalid or stale sonar ends the session. The only
//          reverse-specific exemption is that the 250 mm FORWARD obstacle stop
//          does not apply, so the robot can back away from the dock;
//        * rev_left/rev_right are both <= 0 (a positive wheel, or any forward
//          demand during the session, is ILLEGAL);
//        * the front distance has not fallen more than WRONG_WAY_MM below the
//          furthest seen in this session (robot moving the wrong way);
//        * commanded reverse time < REV_MAX_MS.
//      The first failed check ends the session with a status code; a new
//      rising edge of rev_request is required to reverse again. Speed is
//      clamped to REV_MAX_PWM per wheel.
//
//   4. DIRECTION CHANGE: between forward and reverse output the command is
//      held at zero for DIR_DWELL_MS, so chassis_drive (one command every
//      10 ms) sends at least one zero-speed packet and the wheels settle.
//
// The front sonar does NOT see behind the robot. Reverse is only for a
// supervised, physically cleared corridor: reverse_enable must be switched on
// by the operator and should be off at all other times.
//
// ---------------------------------------------------------------------------
// Latency (all through chassis_drive's 2-register input and 10 ms schedule)
// ---------------------------------------------------------------------------
//   * Stop / timeout / fault -> outputs zero: 0 clocks (combinational).
//   * Outputs -> next UART command: at most one UART period (10 ms) + 2 clocks;
//     the zero packet itself takes about 1.8 ms on the wire.
//   * rev_request rising edge -> reverse output: 1 clock (plus any dwell).
//   * Commanded reverse is cut after exactly REV_MAX_MS of nonzero output.
//
// Status codes (reverse_status), held until the next session starts:
//   0 NONE  1 TIMEOUT  2 STOPPED (IR/disarm/reset during reverse)
//   3 NOT_ENABLED (operator switch off)  4 SONAR (invalid/stale)
//   5 WRONG_WAY  6 ILLEGAL (positive reverse wheel or forward demand during
//   reverse)  7 UNAUTHORISED (negative forward demand without a session)
// ============================================================================

module motion_supervisor #(
    parameter int unsigned CLK_HZ       = 50_000_000,
    parameter int          FWD_MAX_PWM  = 255,        // chassis T:11 limit
    parameter int          REV_MAX_PWM  = 60,         // reverse speed limit per wheel
    parameter int unsigned REV_MAX_MS   = 1500,       // commanded reverse time per session
    parameter int unsigned DIR_DWELL_MS = 100,        // zero hold between forward and reverse
    parameter logic [15:0] WRONG_WAY_MM = 16'd60      // front range shrinkage that ends reverse
) (
    input  wire               clk,
    input  wire               rst_n,                  // asynchronous assert, active low

    // Stop sources: any one stops all motion
    input  wire               arm,                    // SW0, synchronised in the top level
    input  wire               emergency_stop,         // latched IR stop
    input  wire               motion_permitted,       // !emergency_stop from ir_safety_latch

    // Forward permission (V2)
    input  wire               camera_target_fresh,
    input  wire               sonar_valid,
    input  wire               sonar_forward_clear,
    input  wire        [15:0] distance_mm,

    // Forward demand (motion_interface)
    input  wire               fwd_enable,
    input  wire signed [15:0] fwd_left_pwm,
    input  wire signed [15:0] fwd_right_pwm,

    // Reverse authorisation and demand (V3 navigation FSM)
    input  wire               rev_request,            // level; rising edge starts a session
    input  wire signed [15:0] rev_left_pwm,           // <= 0
    input  wire signed [15:0] rev_right_pwm,          // <= 0
    input  wire               reverse_enable,         // operator switch (asynchronous)

    // To chassis_drive
    output logic              drive_enable,
    output logic signed [9:0] left_cmd,
    output logic signed [9:0] right_cmd,

    // Status to navigation and diagnostics
    output logic [2:0]        mode,                   // 0 idle 1 fwd 2 rev 3 dwell 4 stop
    output logic              stop_active,
    output logic              reverse_permitted,      // session live and all checks pass
    output logic              reverse_active,         // reverse being commanded now
    output logic              reverse_timeout,        // last session hit REV_MAX_MS
    output logic              reverse_fault,          // last session ended by a check
    output logic [2:0]        reverse_status,
    output logic [15:0]       reverse_ms              // commanded reverse time this session
);

    // ------------------------------------------------------------------
    // Constants
    // ------------------------------------------------------------------

    localparam int unsigned CYC_PER_MS = (CLK_HZ / 1000 > 0) ? CLK_HZ / 1000 : 1;
    localparam int unsigned DWELL_CYC  = DIR_DWELL_MS * CYC_PER_MS;

    localparam int PS_W = (CYC_PER_MS > 1) ? $clog2(CYC_PER_MS) : 1;
    localparam int DW_W = (DWELL_CYC > 0) ? $clog2(DWELL_CYC + 1) : 1;

    localparam logic [PS_W-1:0] PS_LAST    = PS_W'(CYC_PER_MS - 1);
    localparam logic [DW_W-1:0] DWELL_DONE = DW_W'(DWELL_CYC);
    localparam logic [15:0]     REV_LIMIT  = 16'(REV_MAX_MS);

    localparam logic signed [15:0] FWD_CEIL  = 16'(FWD_MAX_PWM);
    localparam logic signed [15:0] REV_FLOOR = 16'(-REV_MAX_PWM);

    localparam logic [2:0] MODE_IDLE    = 3'd0;
    localparam logic [2:0] MODE_FORWARD = 3'd1;
    localparam logic [2:0] MODE_REVERSE = 3'd2;
    localparam logic [2:0] MODE_DWELL   = 3'd3;
    localparam logic [2:0] MODE_STOP    = 3'd4;

    localparam logic [2:0] ST_NONE         = 3'd0;
    localparam logic [2:0] ST_TIMEOUT      = 3'd1;
    localparam logic [2:0] ST_STOPPED      = 3'd2;
    localparam logic [2:0] ST_NOT_ENABLED  = 3'd3;
    localparam logic [2:0] ST_SONAR        = 3'd4;
    localparam logic [2:0] ST_WRONG_WAY    = 3'd5;
    localparam logic [2:0] ST_ILLEGAL      = 3'd6;
    localparam logic [2:0] ST_UNAUTHORISED = 3'd7;

    localparam logic [1:0] DIR_NONE = 2'd0;
    localparam logic [1:0] DIR_FWD  = 2'd1;
    localparam logic [1:0] DIR_REV  = 2'd2;

    localparam logic [1:0] S_IDLE   = 2'd0;
    localparam logic [1:0] S_ACTIVE = 2'd1;
    localparam logic [1:0] S_ENDED  = 2'd2;

    // ------------------------------------------------------------------
    // Operator switch synchroniser
    // ------------------------------------------------------------------

    (* async_reg = "true" *) logic rev_en_meta;
    (* async_reg = "true" *) logic rev_en_sync;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rev_en_meta <= 1'b0;
            rev_en_sync <= 1'b0;
        end else begin
            rev_en_meta <= reverse_enable;
            rev_en_sync <= rev_en_meta;
        end
    end

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------

    logic [1:0]      sess;          // reverse session
    logic            rev_req_q;     // rev_request one clock ago (edge detect)
    logic [2:0]      status;
    logic [15:0]     rev_ms_q;      // commanded reverse ms this session
    logic [PS_W-1:0] ms_div;        // clocks within the current ms
    logic [15:0]     peak_mm;       // furthest front range seen this session
    logic [1:0]      last_dir;      // direction of the last nonzero output
    logic [DW_W-1:0] zero_cnt;      // clocks the output has been zero (saturating)

    // ------------------------------------------------------------------
    // Saturation helpers
    // ------------------------------------------------------------------

    function automatic logic signed [9:0] sat_fwd(input logic signed [15:0] v);
        if (v <= 16'sd0)
            sat_fwd = 10'sd0;
        else if (v > FWD_CEIL)
            sat_fwd = FWD_CEIL[9:0];
        else
            sat_fwd = v[9:0];
    endfunction

    function automatic logic signed [9:0] sat_rev(input logic signed [15:0] v);
        if (v >= 16'sd0)
            sat_rev = 10'sd0;
        else if (v < REV_FLOOR)
            sat_rev = REV_FLOOR[9:0];
        else
            sat_rev = v[9:0];
    endfunction

    // ------------------------------------------------------------------
    // Combinational decision
    // ------------------------------------------------------------------

    wire stop_override = !rst_n || !arm || emergency_stop || !motion_permitted;

    wire fwd_nonzero      = fwd_enable && ((fwd_left_pwm != 16'sd0) || (fwd_right_pwm != 16'sd0));
    wire fwd_has_negative = fwd_enable && ((fwd_left_pwm <  16'sd0) || (fwd_right_pwm <  16'sd0));
    wire rev_has_positive = (rev_left_pwm > 16'sd0) || (rev_right_pwm > 16'sd0);

    wire [16:0] dist_plus_margin = {1'b0, distance_mm} + {1'b0, WRONG_WAY_MM};
    wire        wrong_way        = sonar_valid && (dist_plus_margin < {1'b0, peak_mm});

    wire dwell_done = (zero_cnt >= DWELL_DONE);

    logic [2:0] end_reason;

    always_comb begin
        end_reason = ST_NONE;
        if (stop_override)
            end_reason = ST_STOPPED;
        else if (!rev_en_sync)
            end_reason = ST_NOT_ENABLED;
        else if (rev_has_positive || fwd_nonzero)
            end_reason = ST_ILLEGAL;
        else if (!sonar_valid)
            end_reason = ST_SONAR;
        else if (wrong_way)
            end_reason = ST_WRONG_WAY;
        else if (rev_ms_q >= REV_LIMIT)
            end_reason = ST_TIMEOUT;
    end

    wire rev_ok      = (sess == S_ACTIVE) && rev_request && (end_reason == ST_NONE);
    wire rev_dir_ok  = (last_dir != DIR_FWD) || dwell_done;
    wire rev_allowed = rev_ok && rev_dir_ok;

    wire fwd_ok      = !stop_override && !rev_request && (sess != S_ACTIVE) &&
                       fwd_enable && !fwd_has_negative &&
                       camera_target_fresh && sonar_valid && sonar_forward_clear;
    wire fwd_dir_ok  = (last_dir != DIR_REV) || dwell_done;
    wire fwd_allowed = fwd_ok && fwd_dir_ok;

    logic signed [9:0] rev_l;
    logic signed [9:0] rev_r;
    logic signed [9:0] fwd_l;
    logic signed [9:0] fwd_r;

    always_comb begin
        rev_l = sat_rev(rev_left_pwm);
        rev_r = sat_rev(rev_right_pwm);
        fwd_l = sat_fwd(fwd_left_pwm);
        fwd_r = sat_fwd(fwd_right_pwm);

        drive_enable = 1'b0;
        left_cmd     = 10'sd0;
        right_cmd    = 10'sd0;

        if (rev_allowed) begin
            drive_enable = 1'b1;
            left_cmd     = rev_l;
            right_cmd    = rev_r;
        end else if (fwd_allowed) begin
            drive_enable = 1'b1;
            left_cmd     = fwd_l;
            right_cmd    = fwd_r;
        end
    end

    wire out_nonzero = (left_cmd != 10'sd0) || (right_cmd != 10'sd0);
    wire rev_moving  = rev_allowed && out_nonzero;

    always_comb begin
        if (stop_override)
            mode = MODE_STOP;
        else if (rev_moving)
            mode = MODE_REVERSE;
        else if (fwd_allowed && out_nonzero)
            mode = MODE_FORWARD;
        else if ((rev_ok && !rev_dir_ok) || (fwd_ok && !fwd_dir_ok))
            mode = MODE_DWELL;
        else
            mode = MODE_IDLE;
    end

    assign stop_active       = stop_override;
    assign reverse_permitted = rev_ok;
    assign reverse_active    = rev_moving;
    assign reverse_status    = status;
    assign reverse_timeout   = (status == ST_TIMEOUT);
    assign reverse_fault     = (status != ST_NONE) && (status != ST_TIMEOUT);
    assign reverse_ms        = rev_ms_q;

    // ------------------------------------------------------------------
    // Sequential state
    // ------------------------------------------------------------------

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sess      <= S_IDLE;
            rev_req_q <= 1'b1;   // a request held through reset is NOT a new edge
            status    <= ST_NONE;
            rev_ms_q  <= 16'd0;
            ms_div    <= '0;
            peak_mm   <= 16'd0;
            last_dir  <= DIR_NONE;
            zero_cnt  <= '0;
        end else begin
            rev_req_q <= rev_request;

            // Reverse session.
            case (sess)
                S_IDLE: begin
                    if (rev_request && !rev_req_q) begin
                        sess     <= S_ACTIVE;
                        status   <= ST_NONE;
                        rev_ms_q <= 16'd0;
                        ms_div   <= '0;
                        peak_mm  <= sonar_valid ? distance_mm : 16'd0;
                    end
                end

                S_ACTIVE: begin
                    if (!rev_request) begin
                        sess <= S_IDLE;
                    end else if (end_reason != ST_NONE) begin
                        sess   <= S_ENDED;
                        status <= end_reason;
                    end else begin
                        if (sonar_valid && (distance_mm > peak_mm))
                            peak_mm <= distance_mm;

                        if (rev_moving) begin
                            if (ms_div == PS_LAST) begin
                                ms_div <= '0;
                                if (rev_ms_q != 16'hFFFF)
                                    rev_ms_q <= rev_ms_q + 16'd1;
                            end else begin
                                ms_div <= ms_div + 1'b1;
                            end
                        end
                    end
                end

                default: begin   // S_ENDED: wait for the request to be withdrawn
                    if (!rev_request)
                        sess <= S_IDLE;
                end
            endcase

            // A negative forward demand outside a session is refused and reported.
            if (!stop_override && fwd_has_negative && (sess != S_ACTIVE))
                status <= ST_UNAUTHORISED;

            // Direction memory and zero-dwell timer.
            if (out_nonzero) begin
                zero_cnt <= '0;
                last_dir <= rev_allowed ? DIR_REV : DIR_FWD;
            end else if (!dwell_done) begin
                zero_cnt <= zero_cnt + 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
