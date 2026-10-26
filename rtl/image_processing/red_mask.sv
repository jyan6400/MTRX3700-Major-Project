`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Major Project - V2 (Group 26, image processing)
// Target-LED segmentation for the OV7670 RGB565 pixel stream.
//
// V2 rewrite: thin wrapper around led_colour_mask, which classifies
// every pixel as red, green or blue LED light. The port list is EXACTLY
// the V1 one used by major_project_top, so the top-level needs no edits
// (an extra unconnected output would raise Verilator PINMISSING and
// break the existing top-level regression builds).
//
//   is_red - pixel belongs to the TARGET colour (red by default).
//            Name kept for V1 port compatibility; it drives the
//            existing led_centroid.
//
// All three colour classifications are available, with the noise filter
// and three centroids, from tri_led_detect.sv for when the top-level is
// opened up for multi-bay work.
//
// TARGET_COLOUR selects which classification drives is_red:
//   0 = red (default, bit-identical to the V1 red_mask defaults)
//   1 = green
//   2 = blue
//
// The red defaults below are the V1 parameters (RED_MIN 20, green
// ceiling 24 on 6-bit green, BLUE_MAX 12, RED_MARGIN 8), so the default
// build reproduces V1 exactly; red_mask_tb proves this exhaustively over
// all 65,536 RGB565 codes against a copy of the V1 module.
//
// Combinational: zero latency, coordinates pass through unchanged.

module red_mask #(
    parameter logic [1:0] TARGET_COLOUR = 2'd0,

    parameter logic [5:0] RED_MIN        = 6'd20,
    parameter logic [5:0] RED_GREEN6_MAX = 6'd24,   // 6-bit green, as V1
    parameter logic [5:0] RED_BLUE_MAX   = 6'd12,
    parameter logic [5:0] RED_MARGIN     = 6'd8,

    parameter logic [5:0] GREEN_MIN      = 6'd18,
    parameter logic [5:0] GREEN_RED_MAX  = 6'd14,
    parameter logic [5:0] GREEN_BLUE_MAX = 6'd20,
    parameter logic [5:0] GREEN_MARGIN   = 6'd6,

    parameter logic [5:0] BLUE_MIN       = 6'd18,
    parameter logic [5:0] BLUE_RED_MAX   = 6'd14,
    parameter logic [5:0] BLUE_GREEN6_MAX= 6'd45,   // 6-bit green
    parameter logic [5:0] BLUE_MARGIN    = 6'd6
) (
    input  wire [15:0] pixel_rgb565,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_y,
    input  wire        pixel_valid,

    output logic       mask_valid,
    output logic       is_red,
    output wire  [9:0] mask_x,
    output wire  [9:0] mask_y
);

    wire cls_red, cls_green, cls_blue;

    led_colour_mask #(
        .RED_MIN       (RED_MIN),
        .RED_GREEN6_MAX(RED_GREEN6_MAX),
        .RED_BLUE_MAX  (RED_BLUE_MAX),
        .RED_MARGIN    (RED_MARGIN),
        .GREEN_MIN     (GREEN_MIN),
        .GREEN_RED_MAX (GREEN_RED_MAX),
        .GREEN_BLUE_MAX(GREEN_BLUE_MAX),
        .GREEN_MARGIN  (GREEN_MARGIN),
        .BLUE_MIN      (BLUE_MIN),
        .BLUE_RED_MAX  (BLUE_RED_MAX),
        .BLUE_GREEN6_MAX(BLUE_GREEN6_MAX),
        .BLUE_MARGIN   (BLUE_MARGIN)
    ) u_classify (
        .pixel_rgb565 (pixel_rgb565),
        .pixel_x      (pixel_x),
        .pixel_y      (pixel_y),
        .pixel_valid  (pixel_valid),
        .mask_valid   (mask_valid),
        .is_red       (cls_red),
        .is_green     (cls_green),
        .is_blue      (cls_blue),
        .mask_x       (mask_x),
        .mask_y       (mask_y)
    );

    always_comb begin
        case (TARGET_COLOUR)
            2'd1:    is_red = cls_green;
            2'd2:    is_red = cls_blue;
            default: is_red = cls_red;
        endcase
    end

endmodule

`default_nettype wire
