`timescale 1ns/1ps
`default_nettype none

// Group 26 - beacon visual odometry (Wilson, M2): the whole chain in one instance.
//
//   PCLK:      camera_stream_adapter outputs -> beacon_detect (R, G, B centroids, 1/16 px)
//   PCLK->50:  vo_mailbox_cdc (one 160-bit result a frame, whole)
//   CLOCK_50:  beacon_geometry (mm, robot frame) -> beacon_tracker (range, velocity, lateral,
//              displacement of the bay's LED) -> vo_standoff (V4: at the set distance)
//
// No frame buffer, no RAM. Resources (estimated, Cyclone V): three 28-bit sum pairs and boxes,
// two sequential dividers (32/19 and 44/32 bits), a handful of 16-48 bit multipliers.
// Latency: the result is out about 210 PCLKs + 4 CLOCK_50 + 330 CLOCK_50 clocks after the frame
// closes (about 12 us): well under one frame.
//
// Integration: README_VO.md. sel is the sequencer's bay index (bay 0/1/2 = red/green/blue LED).
module beacon_vo #(
    // beacon_detect
    parameter int RUN_LEN        = 3,
    parameter int MIN_PIXELS     = 6,
    parameter int MAX_PIXELS     = 6000,
    parameter int MAX_ASPECT     = 4,
    parameter int MIN_FILL       = 64,
    // beacon_geometry (calibration: golden/vo/vo_calibrate.py prints these)
    parameter int F_Q4           = 8960,
    parameter int CX_Q4          = 5112,
    parameter int CY_Q4          = 3832,
    parameter int SIN_Q14        = 3406,
    parameter int COS_Q14        = 16026,
    parameter int DH_MM          = 150,
    parameter int MAX_RANGE_MM   = 4000,
    parameter int SPACING_MM     = 300,
    parameter int PAIR_TOL_SHIFT = 2,
    // beacon_tracker
    parameter int GATE_MM        = 200,
    parameter int MAX_COAST      = 3,
    parameter int MAX_OUTLIERS   = 3,
    parameter int SETTLE_FRAMES  = 4,
    parameter int FPS            = 50,
    // vo_standoff
    parameter int CONFIRM        = 2
) (
    // camera side (PCLK)
    input  wire        pclk,
    input  wire        rst_n_pclk,
    input  wire        frame_start,
    input  wire        frame_end,
    input  wire        frame_good,
    input  wire        pixel_valid,
    input  wire [15:0] pixel_rgb565,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_y,
    output wire [2:0]  dbg_mask,            // filtered {red, green, blue}: for a VGA overlay

    // navigation side (CLOCK_50)
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  sel,                 // the bay: 0 red, 1 green, 2 blue
    input  wire        standoff_enable,
    input  wire        standoff_approaching,
    input  wire [15:0] standoff_target_mm,

    output wire        vo_updated,          // one clock a frame: the outputs below are new
    output wire        track_valid,
    output wire        track_settled,
    output wire signed [15:0] range_mm,
    output wire signed [15:0] vel_mm_s,
    output wire signed [15:0] lateral_mm,
    output wire signed [15:0] disp_mm,
    output wire        standoff_reached,
    output wire        standoff_lost,

    // per LED and pair (CLOCK_50), for debug, logging and the frame grab
    output wire [2:0]  led_valid,
    output wire [47:0] led_forward_mm,
    output wire [47:0] led_lateral_mm,
    output wire        pair_valid,
    output wire        pair_ok,
    output wire signed [15:0] heading_q12,
    output wire        det_dropped,         // PCLK: a frame closed while detect was busy
    output wire        cdc_overrun,         // PCLK: a result arrived while the last was in flight
    output wire [2:0]  track_coast,         // CLOCK_50: frames the track has been predicted without a measurement

    // the detection itself (PCLK), for the frame grab and a VGA overlay
    output wire        det_done,
    output wire [2:0]  det_valid,
    output wire [41:0] det_x_q4,
    output wire [41:0] det_y_q4,
    output wire [56:0] det_count,
    output wire [14:0] det_flags
);
    // ---------------- PCLK: detection ----------------
    wire         d_done, d_good;
    wire [2:0]   d_valid;
    wire [41:0]  d_x, d_y;
    wire [56:0]  d_count;
    wire [14:0]  d_flags;
    wire [119:0] d_bbox;
    beacon_detect #(.RUN_LEN(RUN_LEN), .MIN_PIXELS(MIN_PIXELS), .MAX_PIXELS(MAX_PIXELS),
                    .MAX_ASPECT(MAX_ASPECT), .MIN_FILL(MIN_FILL)) u_detect (
        .clk(pclk), .rst_n(rst_n_pclk),
        .frame_start(frame_start), .frame_end(frame_end), .frame_good(frame_good),
        .pixel_valid(pixel_valid), .pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y),
        .done(d_done), .res_good(d_good), .valid(d_valid), .x_q4(d_x), .y_q4(d_y),
        .count(d_count), .flags(d_flags), .bbox(d_bbox), .dropped(det_dropped), .dbg_mask(dbg_mask));

    // ---------------- PCLK -> CLOCK_50 ----------------
    wire         m_valid;
    wire [159:0] m_data;
    vo_mailbox_cdc #(.W(160)) u_cdc (
        .src_clk(pclk), .src_rst_n(rst_n_pclk), .src_valid(d_done),
        .src_data({d_good, d_valid, d_x, d_y, d_count, d_flags}), .src_overrun(cdc_overrun),
        .dst_clk(clk), .dst_rst_n(rst_n), .dst_valid(m_valid), .dst_data(m_data));
    wire [2:0]  c_valid = m_data[158:156];
    wire [41:0] c_x     = m_data[155:114];
    wire [41:0] c_y     = m_data[113:72];

    // ---------------- CLOCK_50: geometry, tracker, stand-off ----------------
    wire g_done, g_busy;
    beacon_geometry #(.F_Q4(F_Q4), .CX_Q4(CX_Q4), .CY_Q4(CY_Q4), .SIN_Q14(SIN_Q14), .COS_Q14(COS_Q14),
                      .DH_MM(DH_MM), .MAX_RANGE_MM(MAX_RANGE_MM), .SPACING_MM(SPACING_MM),
                      .PAIR_TOL_SHIFT(PAIR_TOL_SHIFT)) u_geom (
        .clk(clk), .rst_n(rst_n), .start(m_valid), .det_valid(c_valid), .det_x_q4(c_x), .det_y_q4(c_y),
        .done(g_done), .led_valid(led_valid), .forward_mm(led_forward_mm), .lateral_mm(led_lateral_mm),
        .pair_valid(pair_valid), .pair_ok(pair_ok), .pair(), .heading_q12(heading_q12), .busy(g_busy));

    wire [2:0] t_coast;
    assign track_coast = t_coast;
    assign det_done  = d_done;
    assign det_valid = d_valid;
    assign det_x_q4  = d_x;
    assign det_y_q4  = d_y;
    assign det_count = d_count;
    assign det_flags = d_flags;
    beacon_tracker #(.GATE_MM(GATE_MM), .MAX_COAST(MAX_COAST), .MAX_OUTLIERS(MAX_OUTLIERS),
                     .SETTLE_FRAMES(SETTLE_FRAMES), .FPS(FPS)) u_track (
        .clk(clk), .rst_n(rst_n), .update(g_done), .sel(sel),
        .led_valid(led_valid), .forward_mm(led_forward_mm), .lateral_mm(led_lateral_mm),
        .updated(vo_updated), .track_valid(track_valid), .settled(track_settled),
        .range_mm(range_mm), .vel_mm_s(vel_mm_s), .lateral_out_mm(lateral_mm), .disp_mm(disp_mm),
        .coast(t_coast));

    vo_standoff #(.CONFIRM(CONFIRM)) u_standoff (
        .clk(clk), .rst_n(rst_n), .enable(standoff_enable), .approaching(standoff_approaching),
        .target_mm(standoff_target_mm), .updated(vo_updated), .track_valid(track_valid),
        .range_mm(range_mm), .reached(standoff_reached), .lost(standoff_lost));
endmodule

`default_nettype wire
