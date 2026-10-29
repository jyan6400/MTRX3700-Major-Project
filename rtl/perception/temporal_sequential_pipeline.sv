`default_nettype none
// V3-D10B: OFF/ON RGB565 pixel core -> immutable hit frame -> sequential CCL.
// All signals MUST be synchronous to clk; this module does not implement CDC.
// The upstream stream must be a complete WIDTH*HEIGHT raster for each capture.
// Only start another OFF/ON capture after busy=0 and frame_ready=0.
module temporal_sequential_pipeline #(
    parameter integer WIDTH=320, HEIGHT=240, MIN_PIXELS=5,
    parameter integer RED_DELTA=35
)(
    input wire clk, rst_n, invalidate,
    input wire capture_off, capture_on,
    input wire pixel_valid, frame_start, frame_end,
    input wire [15:0] pixel_rgb565,
    output wire off_ready, pair_complete, pair_error,
    output wire frame_ready, frame_error, overrun,
    output wire busy, result_valid, target_valid,
    output wire [((WIDTH<=1)?1:$clog2(WIDTH))-1:0] target_x,
    output wire [((HEIGHT<=1)?1:$clog2(HEIGHT))-1:0] target_y,
    output wire [((WIDTH*HEIGHT<=1)?1:$clog2(WIDTH*HEIGHT+1))-1:0] blob_pixels
);
    localparam integer XW=(WIDTH<=1)?1:$clog2(WIDTH);
    localparam integer YW=(HEIGHT<=1)?1:$clog2(HEIGHT);
    localparam integer AW=(WIDTH*HEIGHT<=1)?1:$clog2(WIDTH*HEIGHT);
    wire hit_valid, hit;
    wire [XW-1:0] hit_x;
    wire [YW-1:0] hit_y;
    wire release_frame, read_en, read_valid, read_hit;
    wire [AW-1:0] read_addr;
    // Frame markers must follow the REGISTERED hit stream, not raw camera pins.
    wire hit_frame_start = hit_valid && (hit_x==XW'(0)) && (hit_y==YW'(0));
    wire hit_frame_end = hit_valid && (hit_x==XW'(WIDTH-1)) && (hit_y==YW'(HEIGHT-1));

    temporal_red_pixel_core #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.RED_DELTA(RED_DELTA)) pixel_core (
        .clk,.rst_n,.invalidate,.capture_off,.capture_on,.pixel_valid,
        .frame_start,.frame_end,.pixel_rgb565,
        .off_ready,.pair_complete,.pair_error,.hit_valid,.hit,.hit_x,.hit_y
    );
    temporal_hit_frame_buffer #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) frame_buffer (
        .clk,.rst_n,.invalidate,.hit_valid,.hit,
        .frame_start(hit_frame_start),.frame_end(hit_frame_end),
        .hit_x,.hit_y,.release_frame,.frame_ready,.frame_error,.overrun,
        .read_en,.read_addr,.read_valid,.read_hit
    );
    temporal_ccl_sequential #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.MIN_PIXELS(MIN_PIXELS)) ccl (
        .clk,.rst_n,.invalidate,.frame_ready,.frame_error,
        .read_en,.read_addr,.read_valid,.read_hit,.release_frame,
        .busy,.result_valid,.target_valid,.target_x,.target_y,.blob_pixels
    );
endmodule
`default_nettype wire
