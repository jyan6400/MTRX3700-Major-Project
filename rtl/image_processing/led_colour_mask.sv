`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Major Project - V2 (Group 26, image processing)
// Three-colour LED segmentation for the OV7670 RGB565 pixel stream.
//
// Classifies each pixel as red, green or blue LED light with a
// "dominant channel" test:
//
//   primary channel        >= <COL>_MIN
//   each other channel     <= its <COL>_..._MAX ceiling
//   primary channel        >= each other channel + <COL>_MARGIN
//
// Units:
//   * MIN and MARGIN compare in 5-bit space (green reduced to 5 bits
//     by dropping its LSB) so the three channels are comparable.
//   * Ceilings on the GREEN channel (RED_GREEN6_MAX, BLUE_GREEN6_MAX)
//     use the full 6-bit green value, as V1 red_mask did. This keeps the
//     default red classification bit-identical to V1.
//   * All other ceilings are 5-bit.
//
// Because a pixel must beat BOTH other channels by a positive margin,
// at most one of is_red / is_green / is_blue can be high at a time
// (provided every MARGIN >= 1). White, black, yellow, cyan and magenta
// pixels are rejected by construction.
//
// Combinational: zero clock latency, coordinates pass straight through.
//
// NOTE on saturation: a bright LED core often clips to near-white
// (all channels 31) and is rejected here. The coloured halo around it
// is still accepted, and the centroid of a ring is its centre, so the
// target position is still correct. If the halo is too thin, lower the
// OV7670 exposure rather than loosening these thresholds.

module led_colour_mask #(
    // Red LED (defaults = V1 red_mask)
    parameter logic [5:0] RED_MIN          = 6'd20,   // 5-bit red
    parameter logic [5:0] RED_GREEN6_MAX   = 6'd24,   // 6-bit green
    parameter logic [5:0] RED_BLUE_MAX     = 6'd12,   // 5-bit blue
    parameter logic [5:0] RED_MARGIN       = 6'd8,
    // Green LED (real green LEDs bleed a fair amount of blue)
    parameter logic [5:0] GREEN_MIN        = 6'd18,   // 5-bit green
    parameter logic [5:0] GREEN_RED_MAX    = 6'd14,
    parameter logic [5:0] GREEN_BLUE_MAX   = 6'd20,
    parameter logic [5:0] GREEN_MARGIN     = 6'd6,
    // Blue LED (real blue LEDs read slightly cyan on cheap sensors)
    parameter logic [5:0] BLUE_MIN         = 6'd18,
    parameter logic [5:0] BLUE_RED_MAX     = 6'd14,
    parameter logic [5:0] BLUE_GREEN6_MAX  = 6'd45,   // 6-bit green
    parameter logic [5:0] BLUE_MARGIN      = 6'd6
) (
    input  wire [15:0] pixel_rgb565,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_y,
    input  wire        pixel_valid,

    output logic       mask_valid,
    output logic       is_red,
    output logic       is_green,
    output logic       is_blue,
    output wire  [9:0] mask_x,
    output wire  [9:0] mask_y
);

    // 7-bit working width: channel (<=63) + margin (<=63) never overflows.
    wire [6:0] r  = {2'b0, pixel_rgb565[15:11]};
    wire [6:0] g6 = {1'b0, pixel_rgb565[10:5]};
    wire [6:0] g  = {2'b0, pixel_rgb565[10:6]};   // 5-bit green
    wire [6:0] b  = {2'b0, pixel_rgb565[4:0]};

    wire [6:0] red_min   = {1'b0, RED_MIN};
    wire [6:0] green_min = {1'b0, GREEN_MIN};
    wire [6:0] blue_min  = {1'b0, BLUE_MIN};

    wire red_ok =
        (r  >= red_min) &&
        (g6 <= {1'b0, RED_GREEN6_MAX}) &&
        (b  <= {1'b0, RED_BLUE_MAX}) &&
        (r  >= g + {1'b0, RED_MARGIN}) &&
        (r  >= b + {1'b0, RED_MARGIN});

    wire green_ok =
        (g  >= green_min) &&
        (r  <= {1'b0, GREEN_RED_MAX}) &&
        (b  <= {1'b0, GREEN_BLUE_MAX}) &&
        (g  >= r + {1'b0, GREEN_MARGIN}) &&
        (g  >= b + {1'b0, GREEN_MARGIN});

    wire blue_ok =
        (b  >= blue_min) &&
        (r  <= {1'b0, BLUE_RED_MAX}) &&
        (g6 <= {1'b0, BLUE_GREEN6_MAX}) &&
        (b  >= r + {1'b0, BLUE_MARGIN}) &&
        (b  >= g + {1'b0, BLUE_MARGIN});

    always_comb begin
        mask_valid = pixel_valid;
        is_red     = pixel_valid && red_ok;
        is_green   = pixel_valid && green_ok;
        is_blue    = pixel_valid && blue_ok;
    end

    assign mask_x = pixel_x;
    assign mask_y = pixel_y;

endmodule

`default_nettype wire
