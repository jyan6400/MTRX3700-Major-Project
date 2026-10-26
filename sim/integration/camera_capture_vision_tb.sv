`timescale 1ns/1ps
`default_nettype none

// MTRX3700 V1 - Real camera capture to vision integration
//
// Ed OV7670 model -> Ed capture -> Group 26 camera adapter
// -> red_mask -> led_centroid.
//
// Checks:
// - RGB565 pixel data against Ed camera golden model
// - Complete 640x480 frames
// - Correct raster coordinates
// - Red classification against an independent reference
// - Centroid X/Y against independently calculated averages
// - frame_done and target_valid at completed frames
//
// Two full frames are verified.
// +wave enables FST tracing (large file).

module camera_capture_vision_tb;

    // --------------------------------------------------
    // 1. Ed camera model
    // --------------------------------------------------

    logic clk_10 = 1'b0;
    always #50 clk_10 = ~clk_10;

    logic camera_reset_n = 1'b0;

    tri1 sio_d;
    logic sio_c = 1'b1;

    wire pclk;
    wire vsync;
    wire href;
    wire [7:0] camera_data;

    int camera_frames;
    int camera_writes;
    int camera_errors;

    int scene_x0 = 0;

    ov7670_model #(
        .VERBOSE(0)
    ) cam (
        .sio_c(sio_c),
        .sio_d(sio_d),
        .xclk(clk_10),
        .reset_n(camera_reset_n),
        .pwdn(1'b0),

        .pclk(pclk),
        .vsync(vsync),
        .href(href),
        .d(camera_data),

        .scene_mode(1),
        .scene_x0(scene_x0),
        .scene_y0(0),
        .scene_code(8'hA5),

        .frames_done(camera_frames),
        .write_count(camera_writes),
        .error_count(camera_errors)
    );

    // --------------------------------------------------
    // 2. University OV7670 capture RTL
    // --------------------------------------------------

    wire capture_frame_start;
    wire capture_valid;
    wire [15:0] capture_pixel;
    wire [9:0] capture_x;
    wire [8:0] capture_y;

    ov7670_capture u_capture (
        .pclk(pclk),
        .vsync(vsync),
        .href(href),
        .d(camera_data),

        .frame_start(capture_frame_start),
        .valid(capture_valid),
        .pixel(capture_pixel),
        .x(capture_x),
        .y(capture_y)
    );

    // --------------------------------------------------
    // 3. Group 26 camera stream adapter
    // --------------------------------------------------

    wire frame_start;
    wire frame_end;
    wire pixel_valid;

    wire [15:0] pixel_rgb565;
    wire [9:0] pixel_x;
    wire [9:0] pixel_y;

    wire        adapter_frame_good;
    wire        adapter_stream_ok;
    wire [1:0]  adapter_last_error;
    wire [15:0] adapter_good_frames;
    wire [15:0] adapter_bad_frames;
    wire [15:0] adapter_dropped_pixels;
    wire [23:0] adapter_frame_period;

    camera_stream_adapter u_adapter (
        .clk(pclk),
        .rst_n(camera_reset_n),

        .cam_frame_start(capture_frame_start),
        .cam_valid(capture_valid),
        .cam_pixel(capture_pixel),
        .cam_x(capture_x),
        .cam_y(capture_y),

        .frame_start(frame_start),
        .frame_end(frame_end),
        .pixel_valid(pixel_valid),
        .pixel_rgb565(pixel_rgb565),
        .pixel_x(pixel_x),
        .pixel_y(pixel_y),

        .frame_good(adapter_frame_good),
        .stream_ok(adapter_stream_ok),
        .last_error(adapter_last_error),
        .good_frames(adapter_good_frames),
        .bad_frames(adapter_bad_frames),
        .dropped_pixels(adapter_dropped_pixels),
        .frame_period(adapter_frame_period)
    );

    // --------------------------------------------------
    // 4. Group 26 RGB565 red segmentation
    // --------------------------------------------------

    wire mask_valid;
    wire is_red;
    wire [9:0] mask_x;
    wire [9:0] mask_y;

    red_mask u_red_mask (
        .pixel_rgb565(pixel_rgb565),
        .pixel_x(pixel_x),
        .pixel_y(pixel_y),
        .pixel_valid(pixel_valid),

        .mask_valid(mask_valid),
        .is_red(is_red),
        .mask_x(mask_x),
        .mask_y(mask_y)
    );

    // --------------------------------------------------
    // 5. Group 26 LED centroid
    // --------------------------------------------------

    wire frame_done;
    wire target_valid;
    wire [9:0] target_x;
    wire [9:0] target_y;
    wire [31:0] red_count;

    led_centroid #(
        .MIN_PIXELS(32'd20)
    ) u_centroid (
        .clk(pclk),
        .rst_n(camera_reset_n),

        .frame_start(frame_start),
        .frame_end(frame_end),

        .pixel_valid(mask_valid),
        .is_red(is_red),
        .pixel_x(mask_x),
        .pixel_y(mask_y),

        .frame_done(frame_done),
        .target_valid(target_valid),
        .target_x(target_x),
        .target_y(target_y),
        .red_count(red_count)
    );

    // --------------------------------------------------
    // 6. Independent golden-model reference
    // --------------------------------------------------

    int pixel_count = 0;
    int reference_count = 0;
    int reference_sum_x = 0;
    int reference_sum_y = 0;

    int previous_x = -1;
    int previous_y = -1;

    int starts_seen = 0;
    int completed_frames = 0;
    int total_pixel_checks = 0;

    logic [15:0] expected_pixel;

    // Independently classify RGB565 colours.
    // This function does not read the DUT's is_red output.

    function automatic bit reference_is_red(
        input logic [15:0] rgb
    );
        int r;
        int g;
        int b;

        r = int'(rgb[15:11]);
        g = int'(rgb[10:5]);
        b = int'(rgb[4:0]);

        return
            (r >= 20) &&
            (g <= 24) &&
            (b <= 12) &&
            (r >= ((g >> 1) + 8)) &&
            (r >= (b + 8));
    endfunction

    // Sample after the rising PCLK edge has updated
    // the capture and centroid registers.
    //
    // The Ed model may change its camera output pins
    // near the falling edge, but the capture module's
    // registered output belongs to the previous pixel.

    always @(negedge pclk) begin

        if (camera_reset_n) begin

            if (frame_start) begin
                starts_seen++;

                pixel_count = 0;
                reference_count = 0;
                reference_sum_x = 0;
                reference_sum_y = 0;

                previous_x = -1;
                previous_y = -1;
            end

            if (pixel_valid) begin

                if (starts_seen == 0)
                    $fatal(1,
                        "Pixel received before first frame_start");

                // Compare captured RGB565 values against the
                // Ed camera model's expected pixel.

                expected_pixel =
                    cam.expected_word(int'(pixel_x),
                                      int'(pixel_y));

                if (pixel_rgb565 !== expected_pixel)
                    $fatal(1,
                        "Camera pixel mismatch at (%0d,%0d): got %04h expected %04h",
                        pixel_x, pixel_y,
                        pixel_rgb565, expected_pixel);

                // Verify exact 640x480 raster sequence.

                if (pixel_count == 0) begin
                    if (pixel_x != 0 || pixel_y != 0)
                        $fatal(1,
                            "First pixel not at image origin");
                end else begin
                    if (pixel_count % 640 == 0) begin
                        if (pixel_x != 0 ||
                            int'(pixel_y) != previous_y + 1)
                            $fatal(1,
                                "Bad row transition at pixel %0d",
                                pixel_count);
                    end else begin
                        if (int'(pixel_x) != previous_x + 1 ||
                            int'(pixel_y) != previous_y)
                            $fatal(1,
                                "Bad horizontal raster sequence");
                    end
                end

                previous_x = int'(pixel_x);
                previous_y = int'(pixel_y);

                // Check the Group 26 classifier against
                // the independent colour threshold.

                if (is_red !==
                    reference_is_red(pixel_rgb565))
                    $fatal(1,
                        "Red mask disagreement at (%0d,%0d)",
                        pixel_x, pixel_y);

                if (!mask_valid ||
                    mask_x !== pixel_x ||
                    mask_y !== pixel_y)
                    $fatal(1,
                        "Mask stream coordinates/valid mismatch");

                if (reference_is_red(pixel_rgb565)) begin
                    reference_count++;
                    reference_sum_x += int'(pixel_x);
                    reference_sum_y += int'(pixel_y);
                end

                pixel_count++;
                total_pixel_checks++;

                // frame_end must coincide with the
                // final valid pixel.

                if (frame_end !==
                    ((pixel_x == 639) && (pixel_y == 479)))
                    $fatal(1,
                        "Incorrect frame_end at (%0d,%0d)",
                        pixel_x, pixel_y);
            end

            // Registered centroid outputs correspond to the
            // preceding frame's final pixel.

            if (frame_done) begin

                if (pixel_count != 640 * 480)
                    $fatal(1,
                        "Incomplete frame: received %0d pixels",
                        pixel_count);

                if (red_count !== 32'(reference_count))
                    $fatal(1,
                        "Red count mismatch: RTL=%0d reference=%0d",
                        red_count, reference_count);

                if (reference_count >= 20) begin

                    if (!target_valid)
                        $fatal(1,
                            "Valid target missing at frame completion");

                    if (target_x !==
                        10'(reference_sum_x / reference_count) ||
                        target_y !==
                        10'(reference_sum_y / reference_count))
                        $fatal(1,
                            "Centroid mismatch: RTL=(%0d,%0d) expected=(%0d,%0d)",
                            target_x, target_y,
                            reference_sum_x / reference_count,
                            reference_sum_y / reference_count);

                end else begin

                    if (target_valid ||
                        target_x !== 10'd0 ||
                        target_y !== 10'd0)
                        $fatal(1,
                            "False target detected");
                end

                // V2 frame-integrity stage: real camera frames
                // must all be accepted, none rejected.
                if (adapter_bad_frames != 16'd0 ||
                    adapter_good_frames != 16'(completed_frames + 1) ||
                    adapter_stream_ok !== 1'b1)
                    $fatal(1,
                        "Adapter integrity: good=%0d bad=%0d ok=%b err=%b",
                        adapter_good_frames, adapter_bad_frames,
                        adapter_stream_ok, adapter_last_error);

                completed_frames++;

                $display(
                    "PASS: frame %0d | pixels=%0d red=%0d target_valid=%b centroid=(%0d,%0d)",
                    completed_frames,
                    pixel_count,
                    reference_count,
                    target_valid,
                    target_x,
                    target_y
                );

                if (completed_frames == 2) begin
                    if (camera_errors != 0)
                        $fatal(1,
                            "Camera model reported %0d errors",
                            camera_errors);

                    $display("-------------------------------------------");
                    $display("PASS: camera_capture_vision_tb");
                    $display("2 complete 640x480 frames checked");
                    $display("Total RGB565 pixel checks: %0d",
                             total_pixel_checks);
                    $display("Camera model errors: %0d",
                             camera_errors);
                    $display("Adapter: %0d good / %0d bad frames, %0d pixels dropped before lock, frame period %0d PCLKs",
                             adapter_good_frames, adapter_bad_frames,
                             adapter_dropped_pixels, adapter_frame_period);
                    $display("-------------------------------------------");
                    $finish;
                end
            end
        end
    end

    // --------------------------------------------------
    // 7. Configure camera model
    // --------------------------------------------------

    initial begin

        if ($test$plusargs("wave")) begin
            $dumpfile("camera_capture_vision.fst");
            $dumpvars(0, camera_capture_vision_tb);
        end

        #10us;
        camera_reset_n = 1'b1;

        #1.5ms;

        // Match Ed ov7670_capture_tb configuration.

        cam.set_reg(8'h12, 8'h04);
        cam.set_reg(8'h40, 8'hD0);

        cam.set_reg(8'h17, 8'h14);
        cam.set_reg(8'h18, 8'h02);
        cam.set_reg(8'h32, 8'h80);

        cam.set_reg(8'h19, 8'h02);
        cam.set_reg(8'h1A, 8'h7A);
        cam.set_reg(8'h03, 8'h0F);

        cam.set_course_tuning();
    end

    // Timeout to detect a stalled capture/vision pipeline.

    initial begin
        #350ms;
        $fatal(1,
            "TIMEOUT: only %0d complete frames",
            completed_frames);
    end

endmodule

`default_nettype wire
