`timescale 1ns/1ps
`default_nettype none

// Group 26 - beacon visual odometry (Wilson, M2): range, velocity and position of the selected LED.
//
// One update per frame (beacon_geometry's done). An alpha-beta filter on the selected LED's
// forward distance, in Q4 mm and Q4 mm per frame, with shifts only (alpha = 1/2, beta = 1/8).
// The track's second measurement sets the velocity from the two points (no ramp up from zero):
//     pred = r + v;   e = 16 * measured - pred
//     |e| <= GATE:    r = pred + e/2,   v = v + e/8,   lateral smoothed by halves
//     |e| >  GATE:    an outlier: predict only (coast); MAX_OUTLIERS in a row restart the track there
//     no measurement: coast (r = r + v); more than MAX_COAST frames in a row drop the track
// A new selected LED (bay) drops the track: the next measurement starts a new one.
// Outputs (16-bit, saturated):  range_mm (rounded), vel_mm_s = v * FPS / 16 (negative = closing),
// lateral_mm (smoothed), disp_mm = range now - range at the track's start (the odometry: how far
// the robot has moved along its line of sight since it locked on), settled = 4+ updates (velocity
// meaningful). Golden model: vo_model.Tracker.
module beacon_tracker #(
    parameter int GATE_MM       = 200,
    parameter int MAX_COAST     = 3,
    parameter int MAX_OUTLIERS  = 3,
    parameter int SETTLE_FRAMES = 4,
    parameter int FPS           = 50
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         update,               // one clock: beacon_geometry done
    input  wire  [1:0]  sel,                  // selected LED: 0 red, 1 green, 2 blue (the bay)
    input  wire  [2:0]  led_valid,
    input  wire  [47:0] forward_mm,           // 3 x 16
    input  wire  [47:0] lateral_mm,           // 3 x 16 signed

    output logic        updated,              // one clock after update: outputs are new
    output logic        track_valid,
    output logic        settled,
    output logic signed [15:0] range_mm,
    output logic signed [15:0] vel_mm_s,
    output logic signed [15:0] lateral_out_mm,
    output logic signed [15:0] disp_mm,
    output logic [2:0]  coast
);
    logic        tv;
    logic [1:0]  sel_q;
    logic signed [23:0] r, v, r0;
    logic [2:0]  coast_q;
    logic [1:0]  outl;
    logic [3:0]  age;
    logic signed [15:0] lat;

    // the measurement
    wire        m_ok  = led_valid[sel];
    wire [15:0] m_f   = forward_mm[sel*16 +: 16];
    wire signed [15:0] m_l = lateral_mm[sel*16 +: 16];
    wire signed [23:0] m_q4 = signed'({4'b0, m_f, 4'b0});

    wire signed [23:0] pred = r + v;
    wire signed [24:0] e    = 25'(m_q4) - 25'(pred);
    wire [24:0]  e_abs      = e[24] ? 25'(-e) : 25'(e);
    wire         outlier    = e_abs > 25'(GATE_MM * 16);
    wire signed [16:0] dlat = 17'(m_l) - 17'(lat);

    function automatic logic signed [15:0] sat16(input logic signed [31:0] x);
        if (x > 32767)       return 16'sd32767;
        else if (x < -32768) return -16'sd32768;
        else                 return 16'(x);
    endfunction

    wire new_bay = (sel != sel_q);
    wire tv_n    = tv && !new_bay;           // the track, once a bay change has dropped it
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tv <= 1'b0; sel_q <= '0; r <= '0; v <= '0; r0 <= '0;
            coast_q <= '0; outl <= '0; age <= '0; lat <= '0;
            updated <= 1'b0;
        end else begin
            updated <= update;
            if (update) begin
                // a new bay: forget the old LED's track
                if (new_bay) begin
                    age   <= '0;
                    sel_q <= sel;
                end
                if (m_ok) begin
                    if (!tv_n || (outlier && outl == 2'(MAX_OUTLIERS - 1))) begin
                        tv <= 1'b1; r <= m_q4; v <= '0; r0 <= m_q4;
                        coast_q <= '0; outl <= '0; age <= 4'd1; lat <= m_l;
                    end else if (outlier) begin
                        outl <= outl + 2'd1;
                        r <= pred;
                        coast_q <= coast_q + 3'd1;
                        if (coast_q + 3'd1 > 3'(MAX_COAST)) begin tv <= 1'b0; age <= '0; end
                    end else begin
                        if (age == 4'd1) begin           // second measurement: two-point velocity
                            r <= m_q4;
                            v <= 24'(m_q4 - r);
                        end else begin
                            r <= 24'(pred + (e >>> 1));
                            v <= 24'(v + (e >>> 3));
                        end
                        coast_q <= '0;
                        outl <= '0;
                        if (age != 4'd15) age <= age + 4'd1;
                        lat <= 16'(lat + (dlat >>> 1));
                    end
                end else if (tv_n) begin
                    r <= pred;
                    coast_q <= coast_q + 3'd1;
                    if (coast_q + 3'd1 > 3'(MAX_COAST)) begin tv <= 1'b0; age <= '0; end
                end else begin
                    tv <= 1'b0;
                end
            end
        end
    end

    // outputs from the state
    always_comb begin
        track_valid    = tv;
        settled        = tv && (age >= 4'(SETTLE_FRAMES));
        range_mm       = tv ? sat16(32'((r + 24'sd8) >>> 4)) : 16'sd0;
        vel_mm_s       = tv ? sat16(32'((32'(v) * FPS) >>> 4)) : 16'sd0;
        lateral_out_mm = tv ? lat : 16'sd0;
        disp_mm        = tv ? sat16(32'((r - r0 + 24'sd8) >>> 4)) : 16'sd0;
        coast          = tv ? coast_q : 3'd0;
    end
endmodule

`default_nettype wire
