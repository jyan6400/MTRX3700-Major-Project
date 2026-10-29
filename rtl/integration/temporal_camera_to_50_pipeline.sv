`default_nettype none
// V3-D10C Stage 1: safe isolated camera-stream -> CCL -> result CDC path.
// Camera stream must be clocked and checked in pclk already (camera_stream_adapter).
// No navigation/motor signals are driven by this module.
module temporal_camera_to_50_pipeline #(
    parameter integer RAW_W=640, RAW_H=480, MIN_PIXELS=5
)(
    input wire pclk, rst_n_pclk,
    input wire clk_50, rst_n_50,
    input wire invalidate, capture_off, capture_on,
    input wire frame_start, frame_end, frame_good, pixel_valid,
    input wire [15:0] pixel_rgb565,
    input wire [9:0] pixel_x, pixel_y,
    output wire off_ready, pair_complete, pair_error,
    output wire frame_ready, frame_error, frame_overrun,
    output wire ccl_busy, ccl_result_valid,
    output wire cdc_src_ready, cdc_src_overrun,
    output wire result_50_valid, target_50_valid,
    output wire [((RAW_W/2<=1)?1:$clog2(RAW_W/2))-1:0] target_50_x,
    output wire [((RAW_H/2<=1)?1:$clog2(RAW_H/2))-1:0] target_50_y,
    output wire [((RAW_W*RAW_H/4<=1)?1:$clog2(RAW_W*RAW_H/4+1))-1:0] blob_50_pixels
);
    localparam integer XW=(RAW_W/2<=1)?1:$clog2(RAW_W/2);
    localparam integer YW=(RAW_H/2<=1)?1:$clog2(RAW_H/2);
    localparam integer CW=(RAW_W*RAW_H/4<=1)?1:$clog2(RAW_W*RAW_H/4+1);
    wire ccl_target_valid;
    wire [XW-1:0] ccl_x;
    wire [YW-1:0] ccl_y;
    wire [CW-1:0] ccl_pixels;

    temporal_camera_pclk_pipeline #(.RAW_W(RAW_W), .RAW_H(RAW_H), .MIN_PIXELS(MIN_PIXELS)) vision (
        .pclk, .rst_n(rst_n_pclk), .invalidate,
        .capture_off, .capture_on, .frame_start, .frame_end, .frame_good,
        .pixel_valid, .pixel_rgb565, .pixel_x, .pixel_y,
        .off_ready, .pair_complete, .pair_error,
        .frame_ready, .frame_error, .overrun(frame_overrun),
        .busy(ccl_busy), .result_valid(ccl_result_valid),
        .target_valid(ccl_target_valid), .target_x(ccl_x), .target_y(ccl_y),
        .blob_pixels(ccl_pixels)
    );

    temporal_result_cdc #(.XW(XW), .YW(YW), .CW(CW)) result_mailbox (
        .src_clk(pclk), .src_rst_n(rst_n_pclk),
        .src_valid(ccl_result_valid), .src_target_valid(ccl_target_valid),
        .src_x(ccl_x), .src_y(ccl_y), .src_count(ccl_pixels),
        .src_ready(cdc_src_ready), .src_overrun(cdc_src_overrun),
        .dst_clk(clk_50), .dst_rst_n(rst_n_50),
        .dst_valid(result_50_valid), .dst_target_valid(target_50_valid),
        .dst_x(target_50_x), .dst_y(target_50_y), .dst_count(blob_50_pixels)
    );
endmodule
`default_nettype wire
