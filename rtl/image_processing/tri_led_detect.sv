`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Major Project - V2 (Group 26, M2 image processing)
// Red / green / blue LED detector for the three-bay demonstration:
//
//   led_colour_mask -> colour_run_filter -> 3 x led_centroid (M3, V2 gated)
//
// Runs entirely in PCLK domain and takes the camera_stream_adapter
// outputs directly. All three centroid engines see the same frame_end,
// so their frame_done pulses coincide; one frame_done is exported.
// Per colour: target_valid is a 1-cycle pulse aligned with frame_done;
// x / y / count / confidence / reject flags hold until the next one.
//
// Why the run filter matters here: led_centroid's V2 fill gate rejects a
// frame whose red pixels do not fill their bounding box. A handful of
// isolated false pixels far from the LED (e.g. bright window-frame edges
// misread as blue) stretch that box and would REJECT a genuine LED. The
// filter removes those pixels before they reach the centroid.
//
// Latency: mask + filter add 0 cycles; each result appears 34 PCLK
// cycles after frame_end (led_centroid V2 pipeline).
//
// Not instantiated in major_project_top yet: the static top tracks one
// colour through red_mask. This module is the drop-in for V3 bay
// sequencing (Jason/Marcus), together with a wider vision CDC.

module tri_led_detect #(
    parameter int          RUN_LEN        = 3,
    parameter logic [31:0] MIN_PIXELS     = 32'd20,
    parameter logic [31:0] MAX_PIXELS     = 32'd40000,
    parameter int unsigned MAX_ASPECT     = 4,
    parameter logic [7:0]  MIN_CONFIDENCE = 8'd64
) (
    input  wire        clk,          // camera PCLK
    input  wire        rst_n,
    input  wire        gate_bypass,  // 1 = V1 acceptance in led_centroid

    input  wire        frame_start,
    input  wire        frame_end,
    input  wire        pixel_valid,
    input  wire [15:0] pixel_rgb565,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_y,

    // Debug / overlay taps (filtered mask, aligned with the input pixel)
    output wire        dbg_is_red,
    output wire        dbg_is_green,
    output wire        dbg_is_blue,

    output wire        frame_done,

    output wire        red_valid,
    output wire [9:0]  red_x,
    output wire [9:0]  red_y,
    output wire [31:0] red_count,
    output wire [7:0]  red_confidence,
    output wire [3:0]  red_reject,

    output wire        green_valid,
    output wire [9:0]  green_x,
    output wire [9:0]  green_y,
    output wire [31:0] green_count,
    output wire [7:0]  green_confidence,
    output wire [3:0]  green_reject,

    output wire        blue_valid,
    output wire [9:0]  blue_x,
    output wire [9:0]  blue_y,
    output wire [31:0] blue_count,
    output wire [7:0]  blue_confidence,
    output wire [3:0]  blue_reject
);

    wire       m_valid, m_r, m_g, m_b;
    wire [9:0] m_x, m_y;

    led_colour_mask u_mask (
        .pixel_rgb565 (pixel_rgb565),
        .pixel_x      (pixel_x),
        .pixel_y      (pixel_y),
        .pixel_valid  (pixel_valid),
        .mask_valid   (m_valid),
        .is_red       (m_r),
        .is_green     (m_g),
        .is_blue      (m_b),
        .mask_x       (m_x),
        .mask_y       (m_y)
    );

    wire       f_valid, f_r, f_g, f_b;
    wire [9:0] f_x, f_y;

    colour_run_filter #(.RUN_LEN(RUN_LEN)) u_filt (
        .clk         (clk),
        .rst_n       (rst_n),
        .pixel_valid (m_valid),
        .is_red_in   (m_r),
        .is_green_in (m_g),
        .is_blue_in  (m_b),
        .pixel_x     (m_x),
        .pixel_y     (m_y),
        .mask_valid  (f_valid),
        .is_red      (f_r),
        .is_green    (f_g),
        .is_blue     (f_b),
        .mask_x      (f_x),
        .mask_y      (f_y)
    );

    assign dbg_is_red   = f_r;
    assign dbg_is_green = f_g;
    assign dbg_is_blue  = f_b;

    wire done_g_unused, done_b_unused;

    led_centroid #(
        .MIN_PIXELS(MIN_PIXELS), .MAX_PIXELS(MAX_PIXELS),
        .MAX_ASPECT(MAX_ASPECT), .MIN_CONFIDENCE(MIN_CONFIDENCE)
    ) u_cent_red (
        .clk(clk), .rst_n(rst_n), .gate_bypass(gate_bypass),
        .frame_start(frame_start), .frame_end(frame_end),
        .pixel_valid(f_valid), .is_red(f_r), .pixel_x(f_x), .pixel_y(f_y),
        .frame_done(frame_done), .target_valid(red_valid),
        .target_x(red_x), .target_y(red_y), .red_count(red_count),
        .confidence(red_confidence), .reject_flags(red_reject),
        .bbox_x_min(), .bbox_x_max(), .bbox_y_min(), .bbox_y_max()
    );

    led_centroid #(
        .MIN_PIXELS(MIN_PIXELS), .MAX_PIXELS(MAX_PIXELS),
        .MAX_ASPECT(MAX_ASPECT), .MIN_CONFIDENCE(MIN_CONFIDENCE)
    ) u_cent_green (
        .clk(clk), .rst_n(rst_n), .gate_bypass(gate_bypass),
        .frame_start(frame_start), .frame_end(frame_end),
        .pixel_valid(f_valid), .is_red(f_g), .pixel_x(f_x), .pixel_y(f_y),
        .frame_done(done_g_unused), .target_valid(green_valid),
        .target_x(green_x), .target_y(green_y), .red_count(green_count),
        .confidence(green_confidence), .reject_flags(green_reject),
        .bbox_x_min(), .bbox_x_max(), .bbox_y_min(), .bbox_y_max()
    );

    led_centroid #(
        .MIN_PIXELS(MIN_PIXELS), .MAX_PIXELS(MAX_PIXELS),
        .MAX_ASPECT(MAX_ASPECT), .MIN_CONFIDENCE(MIN_CONFIDENCE)
    ) u_cent_blue (
        .clk(clk), .rst_n(rst_n), .gate_bypass(gate_bypass),
        .frame_start(frame_start), .frame_end(frame_end),
        .pixel_valid(f_valid), .is_red(f_b), .pixel_x(f_x), .pixel_y(f_y),
        .frame_done(done_b_unused), .target_valid(blue_valid),
        .target_x(blue_x), .target_y(blue_y), .red_count(blue_count),
        .confidence(blue_confidence), .reject_flags(blue_reject),
        .bbox_x_min(), .bbox_x_max(), .bbox_y_min(), .bbox_y_max()
    );

endmodule

`default_nettype wire
