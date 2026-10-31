`timescale 1ns/1ps
`default_nettype none

// Group 26 - vision_tap (Wilson, M2): records the real vision chain for the
// golden-model replay (R7), through SignalTap.
//
// Sits beside the vision chain in major_project_top, PCLK domain, and watches
// it; it drives nothing the robot uses. Every cycle on which something worth
// recording happens it raises tap_store for one cycle with a 56-bit record on
// tap_word. SignalTap stores only those cycles (storage qualifier = tap_store),
// so a 32K-sample buffer holds several complete frames.
//
// What is recorded, per frame:
//   FRAME   every cam_frame_start: frame period, SW1, band position, and
//           whether the adapter closed a truncated frame on that cycle
//   PIXEL   the camera-stream pixel the adapter saw (RGB565, x, y) together
//           with the board's verdict on it (adapter accepted it, red_mask bit,
//           and any frame_start/frame_end/frame_good the adapter raised):
//             - every pixel of BAND_ROWS dense rows (all classes, so the
//               replay checks non-red pixels too), the band following the
//               last valid target when BAND_FOLLOW = 1;
//             - every pixel anywhere that is "loosely red" (a strict superset
//               of red_mask with its default thresholds, see LOOSE_*) or that
//               the board's red_mask marked red;
//             - every pixel on which the adapter closed a frame;
//             - the first DROP_LIMIT pixels per frame the adapter dropped.
//   RESULT  every led_centroid frame_done: target valid/x/y, red_count and
//           the measured latency from the frame close to frame_done.
// Pixels that are not recorded are, by construction, not red for the board
// AND not red for the golden model, so a recorded frame can be rebuilt with
// those pixels black and replayed through the model and the RTL: the frame
// result must equal the board's. golden/capture.py does that.
//
// Record layout (tap_word[55:0]):
//   [55:54] tag  00 PIXEL, 01 FRAME, 10 RESULT
//   [53]    seq  toggles on every stored record (no two consecutive samples
//                are equal, so value-change exports such as VCD never merge them)
//   PIXEL : [52:37] rgb565  [36:27] x  [26:18] y  [17] accepted (pixel_valid)
//           [16] is_red  [15] frame_start  [14] frame_end  [13] frame_good
//           [12] in_band  [11] loose  [10:0] cycle (low 11 bits)
//   FRAME : [52:29] period (PCLKs since the previous frame_start, saturating)
//           [28] frame_end (truncated close)  [27] frame_good  [26] SW1
//           [25:16] band first row  [10:0] cycle (low 11 bits)
//   RESULT: [52] target_valid  [51:42] target_x  [41:32] target_y
//           [31:13] red_count[18:0]  [12:5] latency (PCLKs, saturating)  [4] SW1
//
// Timing: one register stage aligns the camera stream with the adapter's
// registered outputs, and tap_word/tap_store are registered: two PCLKs of
// latency, nothing combinational to the robot's logic. Cost: about 120 ALMs
// and no memory (the memory is SignalTap's).

module vision_tap #(
    parameter int          BAND_ROWS    = 8,     // dense rows per frame, 0 = none
    parameter bit          BAND_FOLLOW  = 1'b1,  // centre the band on the last valid target
    parameter int          BAND_Y0      = 236,   // band start until a target is seen
    parameter logic [4:0]  LOOSE_R_MIN  = 5'd16, // loose red: R5 >= 16,
    parameter logic [4:0]  LOOSE_MARGIN = 5'd4,  //   R5 >= G5 + 4 and R5 >= B5 + 4
    parameter int          DROP_LIMIT   = 16
) (
    input  wire        clk,             // camera PCLK
    input  wire        rst_n,

    // camera stream (the adapter's inputs)
    input  wire        cam_frame_start,
    input  wire        cam_valid,
    input  wire [15:0] cam_pixel,
    input  wire [9:0]  cam_x,
    input  wire [8:0]  cam_y,

    // adapter outputs and red_mask
    input  wire        frame_start,
    input  wire        frame_end,
    input  wire        frame_good,
    input  wire        pixel_valid,
    input  wire        is_red,

    // led_centroid result and SW1 as led_centroid sees it
    input  wire        result_done,
    input  wire        target_valid,
    input  wire [9:0]  target_x,
    input  wire [9:0]  target_y,
    input  wire [31:0] red_count,
    input  wire        bypass,

    output wire  [55:0] tap_word,
    output wire         tap_store,
    output wire         tap_trigger     // one PCLK with each FRAME record
);

    // Nothing in the design reads these, only SignalTap: keep Quartus from
    // removing them (and with them the whole tap) as unused logic.
    (* noprune *) logic [55:0] tap_word_r;
    (* noprune *) logic        tap_store_r;
    (* noprune *) logic        tap_trigger_r;
    assign tap_word    = tap_word_r;
    assign tap_store   = tap_store_r;
    assign tap_trigger = tap_trigger_r;

    // ---- stage 1: camera stream delayed to line up with the adapter outputs
    logic        d_fs, d_valid;
    logic [15:0] d_pixel;
    logic [9:0]  d_x;
    logic [8:0]  d_y;
    logic [10:0] cyc, d_cyc;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_fs <= 1'b0; d_valid <= 1'b0; d_pixel <= '0; d_x <= '0; d_y <= '0;
            cyc <= '0; d_cyc <= '0;
        end else begin
            d_fs <= cam_frame_start; d_valid <= cam_valid; d_pixel <= cam_pixel;
            d_x <= cam_x; d_y <= cam_y;
            cyc <= cyc + 11'd1; d_cyc <= cyc;
        end
    end

    // ---- classification of the delayed pixel
    wire [4:0] r5 = d_pixel[15:11];
    wire [4:0] g5 = d_pixel[10:6];
    wire [4:0] b5 = d_pixel[4:0];
    wire       loose = (r5 >= LOOSE_R_MIN) &&
                       ({1'b0, r5} >= {1'b0, g5} + {1'b0, LOOSE_MARGIN}) &&
                       ({1'b0, r5} >= {1'b0, b5} + {1'b0, LOOSE_MARGIN});

    logic [9:0] band_y0;
    wire        in_band = (BAND_ROWS > 0) &&
                          ({1'b0, d_y} >= band_y0) &&
                          ({1'b0, d_y} <  band_y0 + 10'(BAND_ROWS));

    logic [4:0]  drop_cnt;
    wire         dropped    = d_valid && !d_fs && !pixel_valid;
    wire         keep_drop  = dropped && (drop_cnt < 5'(DROP_LIMIT));
    wire         pix_event  = d_valid && !d_fs &&
                              (in_band || loose || is_red || frame_end || frame_start || keep_drop);

    // ---- counters
    logic [23:0] period;
    logic [7:0]  latency;
    logic        seq;

    // A RESULT can coincide with a PIXEL record (a coordinate-error close
    // 35 cycles earlier, dropped pixels still arriving); it then waits one
    // cycle. Camera pixels arrive at most every other PCLK.
    logic        res_pend;
    logic [52:0] res_payload;
    // red_count <= 640*480 = 307,200 < 2^19, so bits [31:19] are always 0.
    wire  [52:0] res_now = {target_valid, target_x, target_y, red_count[18:0], latency, bypass, 4'd0};
    wire         unused_count_msbs = |red_count[31:19];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tap_word_r <= '0; tap_store_r <= 1'b0; tap_trigger_r <= 1'b0;
            period <= '0; latency <= '1; seq <= 1'b0;
            drop_cnt <= '0; band_y0 <= 10'(BAND_Y0);
            res_pend <= 1'b0; res_payload <= '0;
        end else begin
            tap_store_r <= 1'b0;
            tap_trigger_r <= 1'b0;

            if (period != '1) period <= period + 24'd1;
            if (frame_end)               latency <= 8'd1;
            else if (latency != 8'hFF)   latency <= latency + 8'd1;

            if (keep_drop) drop_cnt <= drop_cnt + 5'd1;

            if (result_done && target_valid && BAND_FOLLOW)
                band_y0 <= (target_y < 10'(BAND_ROWS / 2)) ? 10'd0 :
                           ((target_y - 10'(BAND_ROWS / 2)) > 10'(480 - BAND_ROWS)) ? 10'(480 - BAND_ROWS) :
                           (target_y - 10'(BAND_ROWS / 2));

            if (d_fs) begin
                tap_word_r  <= {2'b01, ~seq, period, frame_end, frame_good, bypass, band_y0,
                                5'd0, d_cyc};
                tap_store_r <= 1'b1;
                tap_trigger_r <= 1'b1;
                seq         <= ~seq;
                period      <= 24'd1;
                drop_cnt    <= '0;
                if (result_done) begin res_pend <= 1'b1; res_payload <= res_now; end
            end else if (pix_event) begin
                tap_word_r <= {2'b00, ~seq, d_pixel, d_x, d_y, pixel_valid, is_red, frame_start,
                              frame_end, frame_good, in_band, loose, d_cyc};
                tap_store_r <= 1'b1;
                seq       <= ~seq;
                if (result_done) begin res_pend <= 1'b1; res_payload <= res_now; end
            end else if (res_pend) begin
                tap_word_r <= {2'b10, ~seq, res_payload};    // the waiting one first
                tap_store_r <= 1'b1;
                seq       <= ~seq;
                res_pend  <= result_done;
                if (result_done) res_payload <= res_now;
            end else if (result_done) begin
                tap_word_r <= {2'b10, ~seq, res_now};
                tap_store_r <= 1'b1;
                seq       <= ~seq;
            end
        end
    end

endmodule

`default_nettype wire
