`timescale 1ns/1ps
`default_nettype none

// Group 26 - the result words the JTAG frame grab keeps with each photo for the beacon VO
// (Wilson, M2). Layout 2 (golden/vo/vo_validate_grab.py decodes it). All in the camera's PCLK.
//
//   FULL  beacon_detect inside beacon_vo, the robot's VO: all 640 x 480 pixels
//   KEPT  a second beacon_detect fed only the pixels the photo keeps (even x, even y) at u = x/2,
//         v = y/2: the golden model predicts these EXACTLY from frame.hex
//   GEOM  beacon_geometry's millimetres (checked exactly by re-running the model's geometry on the
//         FULL centroids), and the tracker's range, velocity, lateral and displacement
//
// The detection words latch at detect done, the geometry and tracker words at the tracker's update
// (both about 600 PCLKs after the frame ends), so at the next frame's first pixel, where frame_snap
// samples them, they belong to the frame it has just kept.
//
//   0  {live frame no[15:0], result frame no[15:0]}
//   1  {8'h26, 8'h02, 8'b0, sel[1:0], frame_good, both_done, 4'b0}
//   2..7    FULL, per colour k = 0, 1, 2:  {x_q4[13:0], y_q4[13:0], 3'b0, valid}, {flags[4:0], 8'b0, count[18:0]}
//   8..13   KEPT, the same
//   14..16  per colour {forward_mm[15:0], lateral_mm[15:0]}
//   17      {11'b0, pair_ok, pair_valid, led_valid[2:0], heading_q12[15:0]}
//   18      {range_mm[15:0], vel_mm_s[15:0]}
//   19      {lateral_mm[15:0], disp_mm[15:0]}
//   20      {27'b0, track_valid, settled, coast[2:0]}
module vo_grab_results (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          frame_start,
    input  wire          frame_end,
    input  wire          frame_good,
    input  wire  [1:0]   sel,
    // FULL detect (beacon_vo's u_detect)
    input  wire          f_done,
    input  wire  [2:0]   f_valid,
    input  wire  [41:0]  f_x, f_y,
    input  wire  [56:0]  f_count,
    input  wire  [14:0]  f_flags,
    // KEPT detect
    input  wire          k_done,
    input  wire  [2:0]   k_valid,
    input  wire  [41:0]  k_x, k_y,
    input  wire  [56:0]  k_count,
    input  wire  [14:0]  k_flags,
    // geometry and tracker (beacon_vo, clocked by the same PCLK here)
    input  wire          vo_updated,
    input  wire  [2:0]   led_valid,
    input  wire  [47:0]  led_fwd, led_lat,
    input  wire          pair_valid, pair_ok,
    input  wire  [15:0]  heading_q12,
    input  wire          track_valid, settled,
    input  wire  [15:0]  range_mm, vel_mm_s, lateral_mm, disp_mm,
    input  wire  [2:0]   coast,
    output wire  [21*32-1:0] results
);
    logic [15:0] frame_no, fe_no, result_no;
    logic        fe_good, f_seen, k_seen;
    logic [32*12-1:0] det_w;               // words 2..13
    logic [32*7-1:0]  vo_w;                // words 14..20
    logic [1:0]  sel_q;

    function automatic logic [63:0] det_words(input logic v, input logic [13:0] x, input logic [13:0] y,
                                              input logic [18:0] n, input logic [4:0] f);
        return {x, y, 3'b0, v, f, 8'b0, n};
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_no <= '0; fe_no <= '0; result_no <= '0; fe_good <= 1'b0; f_seen <= 1'b0; k_seen <= 1'b0;
            det_w <= '0; vo_w <= '0; sel_q <= '0;
        end else begin
            if (frame_start) frame_no <= frame_no + 16'd1;
            if (frame_end) begin
                fe_no   <= frame_start ? frame_no + 16'd1 : frame_no;
                fe_good <= frame_good;
                f_seen  <= 1'b0;
                k_seen  <= 1'b0;
            end
            if (f_done) begin
                f_seen <= 1'b1;
                for (int k = 0; k < 3; k++)
                    det_w[32*(10 - 2*k) +: 64] <= det_words(f_valid[k], f_x[k*14 +: 14], f_y[k*14 +: 14],
                                                            f_count[k*19 +: 19], f_flags[k*5 +: 5]);
            end
            if (k_done) begin
                k_seen <= 1'b1;
                for (int k = 0; k < 3; k++)
                    det_w[32*(4 - 2*k) +: 64] <= det_words(k_valid[k], k_x[k*14 +: 14], k_y[k*14 +: 14],
                                                           k_count[k*19 +: 19], k_flags[k*5 +: 5]);
            end
            if (vo_updated) begin
                result_no <= fe_no;
                sel_q     <= sel;
                vo_w <= {led_fwd[15:0], led_lat[15:0], led_fwd[31:16], led_lat[31:16], led_fwd[47:32], led_lat[47:32],
                         11'b0, pair_ok, pair_valid, led_valid, heading_q12,
                         range_mm, vel_mm_s,
                         lateral_mm, disp_mm,
                         27'b0, track_valid, settled, coast};
            end
        end
    end

    assign results = {frame_no, result_no,
                      8'h26, 8'h02, 8'b0, sel_q, fe_good, f_seen && k_seen, 4'b0,
                      det_w, vo_w};
endmodule

`default_nettype wire
