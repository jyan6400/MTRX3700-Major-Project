`timescale 1ns/1ps
/* verilator lint_off PROCASSINIT */
/* verilator lint_off BLKSEQ */
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off UNUSEDSIGNAL */
// (A testbench: never synthesised.)
/*
 *  vision_grab_replay_tb -- Group 26 (Wilson, M2): a photo grabbed from the robot, sent again through the camera
 *  model and the robot's RTL. Run it in a folder grab.py has written (scene.hex, frame.hex).
 *
 *      scene.hex --> ov7670_model (the repo's, vendor/camera/tb) --> ov7670_capture --> camera_stream_adapter
 *                    --> red_mask --> led_centroid (SW1) --> vision_grab_results  (the same RTL as vision_grab_top)
 *
 *  This is FrameGrabJtag's replay_tb with the robot's chain at YOUR MODULE, and the camera at the robot's 50 fps
 *  (CLKRC 0x80, 40 MHz PCLK; set +fps=25 for a grab taken at 25). The camera model sends the photo
 *  pixel-doubled: camera pixel (x, y) = frame.hex (x/2, y/2), checked for all 307,200 pixels of the frame. The
 *  result words for that frame are taken at the next frame's first pixel, as frame_snap takes them, and
 *  written to +out=<file> (8 hex digits a line, word 0 first). validate_grab.py compares them:
 *      KEPT words (7..14) with the board's results.json: must be equal (same pixels in, same RTL)
 *      FULL words (2..6) with the golden model on the pixel-doubled frame: must be equal
 *  The bench itself only checks the pixels and that the words belong to the replayed frame.
 *
 *  Plusargs: +out=<file> (default replay_words.hex), +sw1=0|1, +fps=50|25, +hstart=<n> +vstart=<n> (grab.py's)
 */
module vision_grab_replay_tb;
    localparam int W = 320, H = 240, N = 18;
    logic xclk = 1'b0; always #50 xclk = ~xclk;                     // 10 MHz XCLK
    logic reset_n = 1'b0;
    wire  sio_d; logic sio_c = 1'b1; pullup(sio_d);
    wire  pclk, vsync, href; wire [7:0] d;
    int   frames_done, write_count, error_count;
    ov7670_model #(.VERBOSE(0), .SCENE_FILE("scene.hex")) cam (
        .sio_c(sio_c), .sio_d(sio_d), .xclk(xclk), .reset_n(reset_n), .pwdn(1'b0),
        .pclk(pclk), .vsync(vsync), .href(href), .d(d),
        .scene_mode(1), .scene_x0(0), .scene_y0(0), .scene_code(8'hA5),
        .frames_done(frames_done), .write_count(write_count), .error_count(error_count));

    wire        cam_frame_start, cam_valid;
    wire [15:0] cam_pixel;
    wire [9:0]  cam_x;
    wire [8:0]  cam_y;
    ov7670_capture cap (.pclk(pclk), .vsync(vsync), .href(href), .d(d),
        .frame_start(cam_frame_start), .valid(cam_valid), .pixel(cam_pixel), .x(cam_x), .y(cam_y));

    // ---- the robot's chain, as in vision_grab_top / major_project_top ----
    logic rst_n = 1'b0;
    logic sw1 = 1'b0;
    wire        frame_start, frame_end, pixel_valid, frame_good, stream_ok;
    wire [15:0] pixel_rgb565, good_frames, bad_frames, dropped_pixels;
    wire [9:0]  pixel_x, pixel_y;
    wire [1:0]  last_error;
    wire [23:0] frame_period;
    camera_stream_adapter #(.W(640), .H(480)) u_adapter (
        .clk(pclk), .rst_n(rst_n),
        .cam_frame_start(cam_frame_start), .cam_valid(cam_valid), .cam_pixel(cam_pixel), .cam_x(cam_x), .cam_y(cam_y),
        .frame_start(frame_start), .frame_end(frame_end), .pixel_valid(pixel_valid),
        .pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y),
        .frame_good(frame_good), .stream_ok(stream_ok), .last_error(last_error),
        .good_frames(good_frames), .bad_frames(bad_frames), .dropped_pixels(dropped_pixels), .frame_period(frame_period));

    wire       mask_valid, is_red;
    wire [9:0] mask_x, mask_y;
    red_mask u_red_mask (.pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y), .pixel_valid(pixel_valid),
        .mask_valid(mask_valid), .is_red(is_red), .mask_x(mask_x), .mask_y(mask_y));

    logic [1:0] bypass_sync = 2'b00;
    always_ff @(posedge pclk) bypass_sync <= {bypass_sync[0], sw1};

    wire        c_done, c_valid, kept_done;
    wire [9:0]  c_x, c_y, c_bx0, c_bx1, c_by0, c_by1;
    wire [31:0] c_count;
    wire [7:0]  c_conf;
    wire [3:0]  c_flags;
    led_centroid u_centroid (.clk(pclk), .rst_n(rst_n), .gate_bypass(bypass_sync[1]),
        .frame_start(frame_start), .frame_end(frame_end),
        .pixel_valid(mask_valid), .is_red(is_red), .pixel_x(mask_x), .pixel_y(mask_y),
        .frame_done(c_done), .target_valid(c_valid), .target_x(c_x), .target_y(c_y),
        .red_count(c_count), .confidence(c_conf), .reject_flags(c_flags),
        .bbox_x_min(c_bx0), .bbox_x_max(c_bx1), .bbox_y_min(c_by0), .bbox_y_max(c_by1));

    wire [32*N-1:0] results;
    vision_grab_results #(.N_RESULTS(N)) u_results (
        .clk(pclk), .rst_n(rst_n), .gate_bypass(bypass_sync[1]),
        .frame_start(frame_start), .frame_end(frame_end), .frame_good(frame_good),
        .stream_ok(stream_ok), .last_error(last_error), .good_frames(good_frames), .bad_frames(bad_frames),
        .dropped_pixels(dropped_pixels), .frame_period(frame_period),
        .mask_valid(mask_valid), .is_red(is_red), .mask_x(mask_x), .mask_y(mask_y),
        .full_done(c_done), .full_valid(c_valid), .full_x(c_x), .full_y(c_y), .full_count(c_count),
        .full_conf(c_conf), .full_flags(c_flags), .full_bx0(c_bx0), .full_bx1(c_bx1), .full_by0(c_by0), .full_by1(c_by1),
        .results(results), .kept_done(kept_done));

    // ---- the pixel check (as replay_tb) ----
    logic [15:0] frame [W * H];
    initial $readmemh("frame.hex", frame);
    int hstart = 160, vstart = 11, fps = 50;
    function automatic bit not_picture(input int camera_row);
        return camera_row + vstart < 11;
    endfunction
    logic checking = 1'b0;
    int pixels = 0, bad = 0, max_x = -1, n_done_in_frame = 0;
    logic [15:0] want;
    always @(posedge pclk) if (checking && cam_valid) begin
        pixels++;
        if (int'(cam_x) > max_x) max_x = cam_x;
        want = not_picture(int'(cam_y)) ? cam.expected_word(int'(cam_x), int'(cam_y)) : frame[int'(cam_y[8:1]) * W + int'(cam_x[9:1])];
        if (cam_pixel !== want) begin
            bad++;
            if (bad <= 5) $display("FAIL: camera pixel (%0d,%0d) = %04h, frame.hex (%0d,%0d) = %04h", cam_x, cam_y, cam_pixel, cam_x[9:1], cam_y[8:1], want);
        end
    end

    int fails = 0, stop, vstop, fd, sw1_arg = 0;
    string out = "replay_words.hex";
    logic [31:0] w;
    initial begin
        if ($value$plusargs("hstart=%d", hstart)) ;
        if ($value$plusargs("vstart=%d", vstart)) ;
        if ($value$plusargs("fps=%d", fps)) ;
        if ($value$plusargs("sw1=%d", sw1_arg)) ;
        if ($value$plusargs("out=%s", out)) ;
        sw1 = (sw1_arg != 0);
        stop = (hstart + 640) % 784;
        vstop = (vstart + 480) % 510;
        fork begin #400ms; $display("FAILED: vision_grab_replay_tb: timed out"); $finish; end join_none
        #10us reset_n = 1'b1;
        #1.5ms;
        cam.set_reg(8'h3A, 8'h04); cam.set_reg(8'h11, (fps == 25) ? 8'h81 : 8'h80); cam.set_reg(8'h6B, 8'h4A);
        cam.set_reg(8'h12, 8'h04); cam.set_reg(8'h40, 8'hD0);                                 // RGB565 VGA, full range
        cam.set_reg(8'h17, 8'(hstart >> 3)); cam.set_reg(8'h18, 8'(stop >> 3));
        cam.set_reg(8'h32, 8'(8'h80 | ((stop & 7) << 3) | (hstart & 7)));
        cam.set_reg(8'h19, 8'(vstart >> 2)); cam.set_reg(8'h1A, 8'(vstop >> 2));
        cam.set_reg(8'h03, 8'(((vstop & 3) << 2) | (vstart & 3)));
        cam.set_reg(8'h1E, 8'h37);                                                            // mirror + flip
        cam.set_course_tuning();
        @(posedge cam_frame_start);                    // the frame the registers changed in is over
        @(negedge pclk) rst_n = 1'b1;                  // the chain starts clean on a whole frame (the board's has run for seconds)
        @(posedge cam_frame_start);
        checking = 1'b1;
        @(posedge cam_frame_start);                    // one whole frame checked
        checking = 1'b0;
        do @(posedge pclk); while (!cam_valid);        // frame_snap's moment: the next frame's first pixel
        #1;
        fd = $fopen(out, "w");
        for (int i = 0; i < N; i++) begin
            w = results[32 * (N - 1 - i) +: 32];
            $fdisplay(fd, "%08x", w);
        end
        $fclose(fd);
        $display("replay: window from column %0d, row %0d, %0d fps, SW1=%0d; one frame: %0d camera pixels, columns 0..%0d; %0d differ from frame.hex pixel-doubled",
                 hstart, vstart, fps, sw1, pixels, max_x, bad);
        $display("replay: FULL valid=%0d x=%0d y=%0d count=%0d; KEPT valid=%0d u=%0d v=%0d count=%0d; words -> %s",
                 results[32*(N-5)+31], results[32*(N-3)+16 +: 10], results[32*(N-3) +: 10], results[32*(N-4) +: 32],
                 results[32*(N-10)+31], results[32*(N-8)+16 +: 10], results[32*(N-8) +: 10], results[32*(N-9) +: 32], out);
        if (pixels != 640 * 480 || max_x != 639) begin
            fails++; $display("FAIL: %0d pixels in the frame up to column %0d, expected %0d up to 639", pixels, max_x, 640 * 480);
        end
        if (bad != 0) fails++;
        if ((results[32*N-1 -: 16] - results[32*N-17 -: 16]) != 16'd1) begin
            fails++; $display("FAIL: the words are not the replayed frame's (live %0d, result %0d)", results[32*N-1 -: 16], results[32*N-17 -: 16]);
        end
        if (results[32*(N-2)+24 +: 8] != 8'h26) begin
            fails++; $display("FAIL: no result words (status %08x)", results[32*(N-2) +: 32]);
        end
        if (fails == 0) $display("PASS: vision_grab_replay_tb: all %0d pixels are frame.hex pixel-doubled; the chain's words for that frame are in %s", 640 * 480, out);
        else            $display("FAILED: vision_grab_replay_tb: %0d problems", fails);
        $finish;
    end
endmodule
