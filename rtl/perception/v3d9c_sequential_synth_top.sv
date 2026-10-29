`default_nettype none
// Synthesis-only top: exposes real data inputs and observable CCL outputs.
// No camera/chassis connection. WIDTH=320, HEIGHT=240.
module v3d9c_sequential_synth_top (
    input  wire        CLOCK_50,
    input  wire        rst_n,
    input  wire        invalidate,
    input  wire        hit_valid,
    input  wire        hit,
    input  wire        frame_start,
    input  wire        frame_end,
    input  wire [8:0]  hit_x,
    input  wire [7:0]  hit_y,
    output wire        frame_ready,
    output wire        frame_error,
    output wire        overrun,
    output wire        busy,
    output wire        result_valid,
    output wire        target_valid,
    output wire [8:0]  target_x,
    output wire [7:0]  target_y,
    output wire [16:0] blob_pixels
);
    wire read_en, read_valid, read_hit, release_frame;
    wire [16:0] read_addr;

    temporal_hit_frame_buffer #(.WIDTH(320), .HEIGHT(240)) frame_buffer (
        .clk(CLOCK_50), .rst_n(rst_n), .invalidate(invalidate),
        .hit_valid(hit_valid), .hit(hit),
        .frame_start(frame_start), .frame_end(frame_end),
        .hit_x(hit_x), .hit_y(hit_y),
        .release_frame(release_frame), .frame_ready(frame_ready),
        .frame_error(frame_error), .overrun(overrun),
        .read_en(read_en), .read_addr(read_addr),
        .read_valid(read_valid), .read_hit(read_hit)
    );

    temporal_ccl_sequential #(.WIDTH(320), .HEIGHT(240), .MIN_PIXELS(5)) ccl (
        .clk(CLOCK_50), .rst_n(rst_n), .invalidate(invalidate),
        .frame_ready(frame_ready), .frame_error(frame_error),
        .read_en(read_en), .read_addr(read_addr),
        .read_valid(read_valid), .read_hit(read_hit),
        .release_frame(release_frame), .busy(busy),
        .result_valid(result_valid), .target_valid(target_valid),
        .target_x(target_x), .target_y(target_y), .blob_pixels(blob_pixels)
    );
endmodule
`default_nettype wire
