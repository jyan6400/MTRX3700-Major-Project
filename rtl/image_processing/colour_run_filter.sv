`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Major Project - V2 (Group 26, image processing)
// Horizontal run-length noise filter for the three colour masks.
//
// A pixel is kept only if it AND the previous RUN_LEN-1 valid pixels on
// the same row were all classified the same colour. Isolated speckles
// and runs shorter than RUN_LEN are removed. An LED blob (many pixels
// wide) survives with only its left RUN_LEN-1 pixels trimmed per row.
//
// Zero added latency: the decision uses registered history plus the
// current pixel, so frame_start/frame_end/pixel_valid need no delay and
// stay aligned with the output.
//
// Trimming the left edge of every run shifts a blob's centroid right by
// exactly (RUN_LEN-1)/2 pixels. mask_x is shifted left by X_COMP to
// cancel that; use an odd RUN_LEN so the compensation is exact.
//
// History is cleared at x == 0 so runs never wrap between rows.
// Runs in PCLK domain. RUN_LEN = 1 makes this a pass-through.

module colour_run_filter #(
    parameter int RUN_LEN = 3
) (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       pixel_valid,
    input  wire       is_red_in,
    input  wire       is_green_in,
    input  wire       is_blue_in,
    input  wire [9:0] pixel_x,
    input  wire [9:0] pixel_y,

    output logic      mask_valid,
    output logic      is_red,
    output logic      is_green,
    output logic      is_blue,
    output logic [9:0] mask_x,
    output logic [9:0] mask_y
);

    localparam int        HIST   = (RUN_LEN > 1) ? RUN_LEN - 1 : 1;
    localparam logic [9:0] X_COMP = 10'((RUN_LEN - 1) / 2);

    assign mask_valid = pixel_valid;
    assign mask_y     = pixel_y;
    /* verilator lint_off UNSIGNED */  // constant-true when X_COMP == 0
    assign mask_x     = (pixel_x >= X_COMP) ? (pixel_x - X_COMP) : 10'd0;
    /* verilator lint_on UNSIGNED */

    generate
        if (RUN_LEN <= 1) begin : g_bypass
            assign is_red   = pixel_valid && is_red_in;
            assign is_green = pixel_valid && is_green_in;
            assign is_blue  = pixel_valid && is_blue_in;
        end else begin : g_filter
            logic [HIST-1:0] hist_r, hist_g, hist_b;

            // History is meaningless at the first pixel of a row.
            wire row_start = (pixel_x == 10'd0);
            wire [HIST-1:0] eff_r = row_start ? '0 : hist_r;
            wire [HIST-1:0] eff_g = row_start ? '0 : hist_g;
            wire [HIST-1:0] eff_b = row_start ? '0 : hist_b;

            assign is_red   = pixel_valid && is_red_in   && (&eff_r);
            assign is_green = pixel_valid && is_green_in && (&eff_g);
            assign is_blue  = pixel_valid && is_blue_in  && (&eff_b);

            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    hist_r <= '0;
                    hist_g <= '0;
                    hist_b <= '0;
                end else if (pixel_valid) begin
                    // Shift in the raw classification; keep the newest HIST.
                    hist_r <= HIST'({eff_r, is_red_in});
                    hist_g <= HIST'({eff_g, is_green_in});
                    hist_b <= HIST'({eff_b, is_blue_in});
                end
            end
        end
    endgenerate

endmodule

`default_nettype wire
