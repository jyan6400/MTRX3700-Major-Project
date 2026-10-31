`timescale 1ns/1ps
`default_nettype none

// Group 26 - the result words the JTAG frame grab keeps with each photo (Wilson, M2).
//
// frame_snap (golden/framegrab/vendor/rtl, FrameGrabJtag V2026.3) keeps one
// 320 x 240 picture: camera pixel (2u, 2v) of the 640 x 480 frame, for u < 320,
// v < 240. It also keeps `results`, sampled at the next frame's first pixel.
// This module puts the robot's vision results for that same frame there.
//
// Two sets of results, for two different checks:
//
//   FULL  the robot's own led_centroid result, from the whole 640 x 480 frame
//         (as in major_project_top: camera_stream_adapter -> red_mask ->
//         led_centroid, SW1 = gate_bypass). The grab only holds a quarter of
//         those pixels, so the golden model can only predict FULL approximately
//         from the photo. The replay (scene.hex through the camera model and
//         the same RTL) checks it exactly in simulation.
//
//   KEPT  a second led_centroid, same RTL, same parameters, fed only the pixels
//         the grab keeps (even x, even y), with coordinates u = x/2, v = y/2.
//         Every input it saw is in frame.hex, so the golden model predicts these
//         words EXACTLY from the photo: the board's real camera data, the
//         board's real red_mask and led_centroid, against model.py.
//         Plus the red pixels' coordinate sums (an exact check that does not
//         go through the divider) and the number of kept pixels (76,800 when
//         the frame was whole).
//
// The words are latched when the centroids report (frame_done, 35 PCLKs after
// the adapter closes a frame), so at the next frame's first pixel, where
// frame_snap samples them, they belong to the frame it has just kept. Word 0
// carries the result's frame number and the live one: live - result = 1 means
// the words are that frame's (a frame with no frame_done would leave them
// stale, and validate_grab.py says so).
//
// Word layout (golden/framegrab/validate_grab.py decodes it, LAYOUT 1):
//    0  {live frame no[15:0], result frame no[15:0]}  (adapter frame_starts)
//    1  status {8'h26, 8'h01 layout, 3'b0, last_error[1:0], stream_ok,
//               gate_bypass, frame_good, 4'b0, kept_done, full_done, 2'b0}
//    2  FULL {6'b0, target_x, 6'b0, target_y}
//    3  FULL red_count
//    4  FULL {target_valid, 3'b0, reject_flags[3:0], 16'b0, confidence}
//    5  FULL {6'b0, bbox_x_min, 6'b0, bbox_x_max}
//    6  FULL {6'b0, bbox_y_min, 6'b0, bbox_y_max}
//    7  KEPT {6'b0, target_u, 6'b0, target_v}
//    8  KEPT red_count
//    9  KEPT {target_valid, 3'b0, reject_flags, 16'b0, confidence}
//   10  KEPT {6'b0, bbox_u_min, 6'b0, bbox_u_max}
//   11  KEPT {6'b0, bbox_v_min, 6'b0, bbox_v_max}
//   12  KEPT sum of u over red kept pixels
//   13  KEPT sum of v over red kept pixels
//   14  kept pixels the adapter passed in the frame (76,800 if whole)
//   15  {good_frames, bad_frames}                   (adapter diagnostics)
//   16  {16'b0, dropped_pixels}
//   17  {8'b0, frame_period}                        (PCLKs between frame_starts)

module vision_grab_results #(
    parameter int N_RESULTS = 18
) (
    input  wire         clk,               // camera PCLK
    input  wire         rst_n,
    input  wire         gate_bypass,       // SW1, synchronised (as the robot's led_centroid sees it)

    // camera_stream_adapter outputs
    input  wire         frame_start,
    input  wire         frame_end,
    input  wire         frame_good,
    input  wire         stream_ok,
    input  wire [1:0]   last_error,
    input  wire [15:0]  good_frames,
    input  wire [15:0]  bad_frames,
    input  wire [15:0]  dropped_pixels,
    input  wire [23:0]  frame_period,

    // red_mask outputs (what the robot's led_centroid is fed)
    input  wire         mask_valid,
    input  wire         is_red,
    input  wire [9:0]   mask_x,
    input  wire [9:0]   mask_y,

    // the robot's led_centroid
    input  wire         full_done,
    input  wire         full_valid,
    input  wire [9:0]   full_x,
    input  wire [9:0]   full_y,
    input  wire [31:0]  full_count,
    input  wire [7:0]   full_conf,
    input  wire [3:0]   full_flags,
    input  wire [9:0]   full_bx0, full_bx1, full_by0, full_by1,

    output wire [32*N_RESULTS-1:0] results,     // word 0 in the top bits (frame_snap's order)
    output wire         kept_done              // the KEPT centroid's frame_done (for benches)
);
    // ---------------- KEPT: led_centroid on the pixels the grab keeps ----------
    wire       kept_px = mask_valid && !mask_x[0] && !mask_y[0];
    wire [9:0] u = {1'b0, mask_x[9:1]};
    wire [9:0] v = {1'b0, mask_y[9:1]};

    wire        k_valid;
    wire [9:0]  k_x, k_y, k_bx0, k_bx1, k_by0, k_by1;
    wire [31:0] k_count;
    wire [7:0]  k_conf;
    wire [3:0]  k_flags;

    led_centroid u_kept_centroid (
        .clk(clk), .rst_n(rst_n), .gate_bypass(gate_bypass),
        .frame_start(frame_start), .frame_end(frame_end),
        .pixel_valid(kept_px), .is_red(is_red), .pixel_x(u), .pixel_y(v),
        .frame_done(kept_done), .target_valid(k_valid), .target_x(k_x), .target_y(k_y),
        .red_count(k_count), .confidence(k_conf), .reject_flags(k_flags),
        .bbox_x_min(k_bx0), .bbox_x_max(k_bx1), .bbox_y_min(k_by0), .bbox_y_max(k_by1)
    );

    // ---------------- KEPT sums, frame by frame -------------------------------
    // Same framing as led_centroid: frame_start clears (its own cycle's pixel
    // counts), a pixel on the frame_end cycle counts. An empty close
    // (frame_start and frame_end together, no pixel) gives zeros.
    logic [31:0] sum_u, sum_v, n_kept;
    wire  [31:0] red_u  = (kept_px && is_red) ? 32'(u) : 32'd0;
    wire  [31:0] red_v  = (kept_px && is_red) ? 32'(v) : 32'd0;
    wire  [31:0] one    = kept_px ? 32'd1 : 32'd0;
    wire  [31:0] sum_u_n  = frame_start ? red_u : sum_u + red_u;
    wire  [31:0] sum_v_n  = frame_start ? red_v : sum_v + red_v;
    wire  [31:0] n_kept_n = frame_start ? one   : n_kept + one;

    logic [15:0] frame_no;                 // adapter frame_starts so far (live)
    logic [15:0] fe_no;                    // the frame number at the last frame_end
    logic        fe_good;
    logic [31:0] fe_sum_u, fe_sum_v, fe_n_kept;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_u <= '0; sum_v <= '0; n_kept <= '0;
            frame_no <= '0; fe_no <= '0; fe_good <= 1'b0;
            fe_sum_u <= '0; fe_sum_v <= '0; fe_n_kept <= '0;
        end else begin
            sum_u  <= sum_u_n;
            sum_v  <= sum_v_n;
            n_kept <= n_kept_n;
            if (frame_start) frame_no <= frame_no + 16'd1;
            if (frame_end) begin
                fe_no     <= frame_start ? frame_no + 16'd1 : frame_no;
                fe_good   <= frame_good;
                fe_sum_u  <= sum_u_n;
                fe_sum_v  <= sum_v_n;
                fe_n_kept <= n_kept_n;
            end
        end
    end

    // ---------------- latch every word when the centroids report --------------
    localparam int NW = 17;                // words 1..17 (word 0 is built live)
    logic [32*NW-1:0] words;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            words <= '0;
        end else if (full_done || kept_done) begin
            words <= {
                // 1 status
                8'h26, 8'h01, 3'b0, last_error, stream_ok, gate_bypass, fe_good,
                4'b0, kept_done, full_done, 2'b0,
                // 2..6 FULL
                6'b0, full_x, 6'b0, full_y,
                full_count,
                full_valid, 3'b0, full_flags, 16'b0, full_conf,
                6'b0, full_bx0, 6'b0, full_bx1,
                6'b0, full_by0, 6'b0, full_by1,
                // 7..11 KEPT
                6'b0, k_x, 6'b0, k_y,
                k_count,
                k_valid, 3'b0, k_flags, 16'b0, k_conf,
                6'b0, k_bx0, 6'b0, k_bx1,
                6'b0, k_by0, 6'b0, k_by1,
                // 12..14 KEPT sums
                fe_sum_u, fe_sum_v, fe_n_kept,
                // 15..17 adapter diagnostics
                good_frames, bad_frames,
                16'b0, dropped_pixels,
                8'b0, frame_period
            };
        end
    end

    logic [15:0] result_no;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                       result_no <= '0;
        else if (full_done || kept_done)  result_no <= fe_no;
    end

    generate
        if (N_RESULTS != NW + 1) begin : n_results_must_be_18
            N_RESULTS_must_be_18 stop_here ();
        end
    endgenerate

    assign results = {frame_no, result_no, words};

endmodule

`default_nettype wire
