`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Major Project - V1
// Red LED segmentation for OV7670 RGB565 pixel stream.
//
// Combinational output: no additional clock-cycle latency.
// pixel_valid gates is_red; coordinates pass through unchanged.

module red_mask_v1_ref #(
    parameter logic [4:0] RED_MIN    = 5'd20,
    parameter logic [5:0] GREEN_MAX  = 6'd24,
    parameter logic [4:0] BLUE_MAX   = 5'd12,
    parameter logic [5:0] RED_MARGIN = 6'd8
) (
    input  wire [15:0] pixel_rgb565,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_y,
    input  wire        pixel_valid,

    output logic       mask_valid,
    output logic       is_red,
    output wire [9:0]  mask_x,
    output wire [9:0]  mask_y
);

    wire [4:0] red        = pixel_rgb565[15:11];
    wire [5:0] green      = pixel_rgb565[10:5];
    wire [4:0] blue       = pixel_rgb565[4:0];
    wire [4:0] green_5bit = green[5:1];

    // 6-bit arithmetic: max channel value 31 + margin 8 = 39.
    wire [5:0] red_extended   = {1'b0, red};
    wire [5:0] green_extended = {1'b0, green_5bit};
    wire [5:0] blue_extended  = {1'b0, blue};

    always_comb begin
        mask_valid = pixel_valid;

        is_red = pixel_valid &&
                 (red   >= RED_MIN) &&
                 (green <= GREEN_MAX) &&
                 (blue  <= BLUE_MAX) &&
                 (red_extended >= (green_extended + RED_MARGIN)) &&
                 (red_extended >= (blue_extended  + RED_MARGIN));
    end

    assign mask_x = pixel_x;
    assign mask_y = pixel_y;

endmodule

`default_nettype wire
