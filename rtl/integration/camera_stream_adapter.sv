`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Major Project - Group 26
// camera_stream_adapter (V2): OV7670 capture stream -> checked vision stream.
//
// Clock domain: camera PCLK only. No clock-domain crossing in this module.
//
// Input (Ed ov7670_capture):
//   cam_frame_start  one PCLK when VSYNC falls (rows of a new frame follow)
//   cam_valid        one PCLK per complete RGB565 pixel
//   cam_pixel        RGB565
//   cam_x, cam_y     coordinates from the capture's own HREF/VSYNC counters
//
// Output (red_mask / led_centroid / later stages):
//   frame_start, frame_end, pixel_valid, pixel_rgb565, pixel_x, pixel_y
//   frame_good       qualifies frame_end: 1 = complete W x H frame,
//                                         0 = frame rejected
//
// Why this exists (V1 problem):
//   The V1 adapter was a pass-through. frame_end only fired on pixel
//   (639,479), so a frame cut short by an early VSYNC produced no frame_end
//   and no result at all, pixels captured mid-frame after reset were fed
//   into the first centroid, and malformed rows reached the centroid with
//   the wrong coordinates.
//
// What this module does:
//   1. Lock. After reset, every pixel is dropped until the first
//      frame_start, so a frame entered part-way is never processed.
//   2. Check every pixel. Its own counters (exp_x, exp_y) follow the raster
//      order 0..W-1, 0..H-1. A pixel whose (x,y) differs from them (a short
//      row, a long row, a missing row, x >= W) is a COORDINATE error.
//   3. Close every frame exactly once:
//        - good frame: frame_end and frame_good on its last pixel (W-1,H-1),
//          exactly as in V1, so led_centroid behaves as before;
//        - COORDINATE error: closed immediately, on the offending pixel;
//        - TRUNCATED (frame_start before the last pixel): closed in the same
//          cycle as the new frame_start.
//      A rejected frame is closed with an EMPTY CLOSE: frame_start = 1,
//      frame_end = 1, pixel_valid = 0, frame_good = 0, all in one cycle.
//      A consumer that clears its accumulators on frame_start (led_centroid
//      does) therefore reports an empty frame: frame_done with
//      target_valid = 0. A bad frame can never produce a target; it produces
//      an explicit "no target" result that navigation already treats safely.
//      Pixels of a frame after it is closed are dropped until the next
//      frame_start.
//   4. Diagnostics: good/bad frame counters, dropped-pixel counter, the
//      error flags of the last closed frame, and the frame period in PCLK
//      cycles (frame rate = f_PCLK / frame_period).
//
// Latency: every output is registered, so the stream is exactly one PCLK
// behind the input, with all outputs delayed together (alignment kept).
//
// Assumption: the camera leaves at least 32 PCLKs between a frame's last
// pixel and the next frame_start (the OV7670 leaves 10 blank lines plus
// 3 VSYNC lines), so led_centroid's 32-cycle divider never sees two
// frame_end pulses closer than that.
//
// Parameters W and H let testbenches run on small frames; the top level uses
// the defaults 640 x 480.

module camera_stream_adapter #(
    parameter int W        = 640,
    parameter int H        = 480,
    parameter int PERIOD_W = 24      // frame_period width; saturates
) (
    input  wire        clk,          // camera PCLK
    input  wire        rst_n,        // PCLK-synchronised active-low reset

    input  wire        cam_frame_start,
    input  wire        cam_valid,
    input  wire [15:0] cam_pixel,
    input  wire [9:0]  cam_x,
    input  wire [8:0]  cam_y,

    output logic        frame_start,
    output logic        frame_end,
    output logic        pixel_valid,
    output logic [15:0] pixel_rgb565,
    output logic [9:0]  pixel_x,
    output logic [9:0]  pixel_y,

    output logic        frame_good,      // valid with frame_end
    output logic        stream_ok,       // last closed frame was good
    output logic [1:0]  last_error,      // {TRUNCATED, COORDINATE} of last closed frame
    output logic [15:0] good_frames,     // wrap-around counters
    output logic [15:0] bad_frames,
    output logic [15:0] dropped_pixels,  // saturating
    output logic [PERIOD_W-1:0] frame_period
);

    // Expected coordinates of the next pixel.
    logic [9:0] exp_x;
    logic [8:0] exp_y;

    logic synced;        // a frame_start has been seen since reset
    logic open_frame;    // the current frame has not been closed yet

    logic [PERIOD_W-1:0] period_cnt;

    wire coord_ok   = (cam_x == exp_x) && (cam_y == exp_y);
    wire last_pixel = (exp_x == 10'(W-1)) && (exp_y == 9'(H-1));

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_start    <= 1'b0;
            frame_end      <= 1'b0;
            pixel_valid    <= 1'b0;
            pixel_rgb565   <= 16'd0;
            pixel_x        <= 10'd0;
            pixel_y        <= 10'd0;
            frame_good     <= 1'b0;

            stream_ok      <= 1'b0;
            last_error     <= 2'b00;
            good_frames    <= 16'd0;
            bad_frames     <= 16'd0;
            dropped_pixels <= 16'd0;
            frame_period   <= '0;

            exp_x          <= 10'd0;
            exp_y          <= 9'd0;
            synced         <= 1'b0;
            open_frame     <= 1'b0;
            period_cnt     <= '0;
        end else begin
            // Single-cycle outputs default low.
            frame_start <= 1'b0;
            frame_end   <= 1'b0;
            pixel_valid <= 1'b0;
            frame_good  <= 1'b0;

            // Frame period: PCLK cycles from one frame_start to the next.
            if (period_cnt != '1)
                period_cnt <= period_cnt + 1'b1;

            if (cam_frame_start) begin
                // ---------------------------------------------- new frame
                if (synced)
                    frame_period <= period_cnt;
                period_cnt  <= PERIOD_W'(1);

                frame_start <= 1'b1;

                if (open_frame) begin
                    // Previous frame never reached its last pixel.
                    frame_end  <= 1'b1;           // empty close
                    bad_frames <= bad_frames + 1'b1;
                    last_error <= 2'b10;          // TRUNCATED
                    stream_ok  <= 1'b0;
                end

                // A pixel cannot coincide with frame_start from
                // ov7670_capture (VSYNC is high for the pixel's row);
                // if one ever did, it is dropped.
                if (cam_valid && dropped_pixels != 16'hFFFF)
                    dropped_pixels <= dropped_pixels + 1'b1;

                synced     <= 1'b1;
                open_frame <= 1'b1;
                exp_x      <= 10'd0;
                exp_y      <= 9'd0;

            end else if (cam_valid) begin
                if (!open_frame) begin
                    // ------------------------------------ not in a frame
                    // Before the first frame_start, after a rejected
                    // frame, or extra pixels after a complete frame.
                    if (dropped_pixels != 16'hFFFF)
                        dropped_pixels <= dropped_pixels + 1'b1;

                end else if (!coord_ok) begin
                    // ------------------------------- coordinate error
                    frame_start <= 1'b1;          // empty close
                    frame_end   <= 1'b1;
                    bad_frames  <= bad_frames + 1'b1;
                    last_error  <= 2'b01;         // COORDINATE
                    stream_ok   <= 1'b0;
                    open_frame  <= 1'b0;
                    if (dropped_pixels != 16'hFFFF)
                        dropped_pixels <= dropped_pixels + 1'b1;

                end else begin
                    // ---------------------------------- good pixel
                    pixel_valid  <= 1'b1;
                    pixel_rgb565 <= cam_pixel;
                    pixel_x      <= cam_x;
                    pixel_y      <= {1'b0, cam_y};

                    if (exp_x == 10'(W-1)) begin
                        exp_x <= 10'd0;
                        exp_y <= exp_y + 1'b1;
                    end else begin
                        exp_x <= exp_x + 1'b1;
                    end

                    if (last_pixel) begin
                        frame_end   <= 1'b1;
                        frame_good  <= 1'b1;
                        good_frames <= good_frames + 1'b1;
                        last_error  <= 2'b00;
                        stream_ok   <= 1'b1;
                        open_frame  <= 1'b0;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
