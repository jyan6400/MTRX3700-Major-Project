`default_nettype none
// V3-D10B Stage 2: existing checked camera stream -> 2x decimation ->
// temporal OFF/ON -> immutable hit buffer -> sequential CCL.
// Everything is clocked by camera PCLK.  NO PCLK/CLOCK_50 CDC here.
// Inputs are the registered outputs of camera_stream_adapter at RAW_W x RAW_H.
module temporal_camera_pclk_pipeline #(
    parameter integer RAW_W=640, RAW_H=480, MIN_PIXELS=5
) (
    input wire pclk, rst_n, invalidate,
    input wire capture_off, capture_on,
    input wire frame_start, frame_end, frame_good, pixel_valid,
    input wire [15:0] pixel_rgb565,
    input wire [9:0] pixel_x, pixel_y,
    output wire off_ready, pair_complete, pair_error,
    output wire frame_ready, frame_error, overrun,
    output wire busy, result_valid, target_valid,
    output wire [((RAW_W/2<=1)?1:$clog2(RAW_W/2))-1:0] target_x,
    output wire [((RAW_H/2<=1)?1:$clog2(RAW_H/2))-1:0] target_y,
    output wire [((RAW_W*RAW_H/4<=1)?1:$clog2(RAW_W*RAW_H/4+1))-1:0] blob_pixels
);
    localparam integer W=RAW_W/2, H=RAW_H/2;
    // Explicit invalidation of malformed frames. The upstream adapter's
    // empty close occurs when frame_end=1 and frame_good=0.
    wire bad_close = frame_end && !frame_good;
    wire local_invalidate = invalidate || bad_close;
    wire sample = pixel_valid && !pixel_x[0] && !pixel_y[0];
    wire sample_fs = sample && (pixel_x == 10'd0) && (pixel_y == 10'd0);
    wire sample_fe = sample && (pixel_x == 10'(RAW_W-2)) &&
                                     (pixel_y == 10'(RAW_H-2));
    // frame_start/frame_end from the adapter are deliberately NOT forwarded:
    // the temporal core requires boundaries on the decimated valid pixel.
    temporal_sequential_pipeline #(.WIDTH(W),.HEIGHT(H),.MIN_PIXELS(MIN_PIXELS)) pipe (
        .clk(pclk), .rst_n, .invalidate(local_invalidate),
        .capture_off, .capture_on,
        .pixel_valid(sample), .frame_start(sample_fs), .frame_end(sample_fe),
        .pixel_rgb565,
        .off_ready, .pair_complete, .pair_error,
        .frame_ready, .frame_error, .overrun,
        .busy, .result_valid, .target_valid, .target_x, .target_y, .blob_pixels
    );
endmodule
`default_nettype wire
