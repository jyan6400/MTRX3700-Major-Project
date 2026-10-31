`timescale 1ns/1ps
`default_nettype none

// Group 26 - golden-model replay bench (Wilson, M2).
//
// Drives the REAL vision-chain RTL with a recorded or generated camera stream
// and writes every observable output in the record format of golden/vlog.py,
// so golden/compare.py can check it against the Python golden model and
// against a board capture.
//
//   camera stream (.stim) -> camera_stream_adapter -> red_mask -> led_centroid
//                                                  \-> tri_led_detect
//
// This is the same chain, with the same default parameters, as
// rtl/integration/major_project_top.sv. SW1 drives led_centroid.gate_bypass
// directly (the top's two-flop synchroniser only delays it).
//
// Plusargs:  +STIM=<file.stim>  +OUT=<file.rec>
// Compile-time: -GW=<width> -GH=<height> for small-frame fuzz runs.
//
// Cycle convention: the event read for cycle c is applied before PCLK edge c;
// outputs registered on edge c are logged as cycle c.

module vision_replay_tb #(
    parameter int W = 640,
    parameter int H = 480
);

    logic clk = 1'b0;
    always #12.5 clk = ~clk;           // 40 MHz, as the OV7670 PCLK

    logic        rst_n = 1'b0;
    logic        cam_fs = 1'b0, cam_valid = 1'b0, sw1 = 1'b0;
    logic [15:0] cam_pixel = '0;
    logic [9:0]  cam_x = '0;
    logic [8:0]  cam_y = '0;

    // ---------------- adapter ------------------------------------------
    wire        frame_start, frame_end, pixel_valid, frame_good, stream_ok;
    wire [15:0] pixel_rgb565;
    wire [9:0]  pixel_x, pixel_y;
    wire [1:0]  last_error;
    wire [15:0] good_frames, bad_frames, dropped_pixels;
    wire [23:0] frame_period;

    camera_stream_adapter #(.W(W), .H(H)) u_adapter (
        .clk(clk), .rst_n(rst_n),
        .cam_frame_start(cam_fs), .cam_valid(cam_valid), .cam_pixel(cam_pixel),
        .cam_x(cam_x), .cam_y(cam_y),
        .frame_start(frame_start), .frame_end(frame_end), .pixel_valid(pixel_valid),
        .pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y),
        .frame_good(frame_good), .stream_ok(stream_ok), .last_error(last_error),
        .good_frames(good_frames), .bad_frames(bad_frames),
        .dropped_pixels(dropped_pixels), .frame_period(frame_period)
    );

    // ---------------- red_mask + led_centroid (as in major_project_top) --
    wire       mask_valid, is_red;
    wire [9:0] mask_x, mask_y;

    red_mask u_red_mask (
        .pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y),
        .pixel_valid(pixel_valid),
        .mask_valid(mask_valid), .is_red(is_red), .mask_x(mask_x), .mask_y(mask_y)
    );

    wire        c_done, c_valid;
    wire [9:0]  c_x, c_y, c_bx0, c_bx1, c_by0, c_by1;
    wire [31:0] c_count;
    wire [7:0]  c_conf;
    wire [3:0]  c_flags;

    led_centroid u_centroid (
        .clk(clk), .rst_n(rst_n), .gate_bypass(sw1),
        .frame_start(frame_start), .frame_end(frame_end),
        .pixel_valid(mask_valid), .is_red(is_red), .pixel_x(mask_x), .pixel_y(mask_y),
        .frame_done(c_done), .target_valid(c_valid), .target_x(c_x), .target_y(c_y),
        .red_count(c_count), .confidence(c_conf), .reject_flags(c_flags),
        .bbox_x_min(c_bx0), .bbox_x_max(c_bx1), .bbox_y_min(c_by0), .bbox_y_max(c_by1)
    );

    // ---------------- tri_led_detect (Wilson's V3 candidate) ------------
    wire        t_done;
    wire        t_v [3];
    wire [9:0]  t_x [3], t_y [3];
    wire [31:0] t_n [3];
    wire [7:0]  t_c [3];
    wire [3:0]  t_f [3];
    wire        f_r, f_g, f_b;

    tri_led_detect u_tri (
        .clk(clk), .rst_n(rst_n), .gate_bypass(sw1),
        .frame_start(frame_start), .frame_end(frame_end), .pixel_valid(pixel_valid),
        .pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y),
        .dbg_is_red(f_r), .dbg_is_green(f_g), .dbg_is_blue(f_b),
        .frame_done(t_done),
        .red_valid(t_v[0]), .red_x(t_x[0]), .red_y(t_y[0]), .red_count(t_n[0]),
        .red_confidence(t_c[0]), .red_reject(t_f[0]),
        .green_valid(t_v[1]), .green_x(t_x[1]), .green_y(t_y[1]), .green_count(t_n[1]),
        .green_confidence(t_c[1]), .green_reject(t_f[1]),
        .blue_valid(t_v[2]), .blue_x(t_x[2]), .blue_y(t_y[2]), .blue_count(t_n[2]),
        .blue_confidence(t_c[2]), .blue_reject(t_f[2])
    );

    // bounding boxes of the three tri-colour centroids (not on its ports)
    wire [9:0] t_bx0 [3], t_bx1 [3], t_by0 [3], t_by1 [3];
    assign t_bx0[0] = u_tri.u_cent_red.bbox_x_min;   assign t_bx1[0] = u_tri.u_cent_red.bbox_x_max;
    assign t_by0[0] = u_tri.u_cent_red.bbox_y_min;   assign t_by1[0] = u_tri.u_cent_red.bbox_y_max;
    assign t_bx0[1] = u_tri.u_cent_green.bbox_x_min; assign t_bx1[1] = u_tri.u_cent_green.bbox_x_max;
    assign t_by0[1] = u_tri.u_cent_green.bbox_y_min; assign t_by1[1] = u_tri.u_cent_green.bbox_y_max;
    assign t_bx0[2] = u_tri.u_cent_blue.bbox_x_min;  assign t_bx1[2] = u_tri.u_cent_blue.bbox_x_max;
    assign t_by0[2] = u_tri.u_cent_blue.bbox_y_min;  assign t_by1[2] = u_tri.u_cent_blue.bbox_y_max;

    wire [2:0] lcm_bits  = {u_tri.m_r, u_tri.m_g, u_tri.m_b};
    wire [2:0] filt_bits = {f_r, f_g, f_b};
    wire [9:0] filt_x    = u_tri.f_x;

    // ---------------- vision_tap on the same chain (+TAP=<file.taplog>) ---
    // Lets the capture path (vision_tap -> capture.py) be tested on streams
    // with adapter faults, which the camera model never produces.
    wire [55:0] tap_word;
    wire        tap_store, tap_trigger;
    vision_tap u_tap (
        .clk(clk), .rst_n(rst_n),
        .cam_frame_start(cam_fs), .cam_valid(cam_valid), .cam_pixel(cam_pixel),
        .cam_x(cam_x), .cam_y(cam_y),
        .frame_start(frame_start), .frame_end(frame_end), .frame_good(frame_good),
        .pixel_valid(pixel_valid), .is_red(is_red),
        .result_done(c_done), .target_valid(c_valid), .target_x(c_x), .target_y(c_y),
        .red_count(c_count), .bypass(sw1),
        .tap_word(tap_word), .tap_store(tap_store), .tap_trigger(tap_trigger)
    );
    int  fd_tap = 0;
    bit  tap_armed = 0;
    always @(posedge clk) begin
        if (fd_tap != 0) begin
            if (tap_trigger) tap_armed = 1;
            if (tap_armed && tap_store) $fwrite(fd_tap, "%014h\n", tap_word);
        end
    end

    // ---------------- stimulus and logging -------------------------------
    int     fd_in, fd_out;
    longint cycle = 0;
    longint n_events = 0;
    string  stim_path, out_path, line;

    task automatic log_cycle();
        if (pixel_valid)
            $fwrite(fd_out, "P %0d %0d %0d %0d %0d %0d %0d %0d\n", cycle, pixel_x, pixel_y,
                    pixel_rgb565, is_red, lcm_bits, filt_bits, filt_x);
        if (frame_start)
            $fwrite(fd_out, "S %0d\n", cycle);
        if (frame_end)
            $fwrite(fd_out, "E %0d %0d\n", cycle, frame_good);
        if (frame_start || frame_end)
            $fwrite(fd_out, "D %0d %0d %0d %0d %0d %0d %0d\n", cycle, stream_ok, last_error,
                    good_frames, bad_frames, dropped_pixels, frame_period);
        if (c_done)
            $fwrite(fd_out, "R %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d\n", cycle, c_valid,
                    c_x, c_y, c_count, c_conf, c_flags, c_bx0, c_bx1, c_by0, c_by1);
        if (t_done)
            for (int k = 0; k < 3; k++)
                $fwrite(fd_out, "T %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d\n", cycle, k,
                        t_v[k], t_x[k], t_y[k], t_n[k], t_c[k], t_f[k],
                        t_bx0[k], t_bx1[k], t_by0[k], t_by1[k]);
    endtask

    // Apply one cycle's inputs (called just after a negedge), clock it, log.
    task automatic step(input logic fs, input logic valid, input logic [15:0] px,
                        input logic [9:0] x, input logic [8:0] y);
        cam_fs = fs; cam_valid = valid; cam_pixel = px; cam_x = x; cam_y = y;
        @(posedge clk);
        #1;
        log_cycle();
        cycle++;
        @(negedge clk);
    endtask

    initial begin
        logic [63:0] w;
        longint target;
        int r;

        if (!$value$plusargs("STIM=%s", stim_path)) $fatal(1, "vision_replay_tb: +STIM=<file> required");
        if (!$value$plusargs("OUT=%s", out_path))   $fatal(1, "vision_replay_tb: +OUT=<file> required");
        fd_in = $fopen(stim_path, "r");
        if (fd_in == 0) $fatal(1, "vision_replay_tb: cannot open %s", stim_path);
        fd_out = $fopen(out_path, "w");
        if (fd_out == 0) $fatal(1, "vision_replay_tb: cannot open %s", out_path);
        $fwrite(fd_out, "# vision_replay_tb W=%0d H=%0d stim=%s\n", W, H, stim_path);
        if ($value$plusargs("TAP=%s", line)) begin
            fd_tap = $fopen(line, "w");
            if (fd_tap == 0) $fatal(1, "vision_replay_tb: cannot open %s", line);
        end

        repeat (4) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        target = 0;
        while ($fgets(line, fd_in) > 0) begin
            if (line.len() < 16 || line.getc(0) == "#") continue;
            r = $sscanf(line, "%h", w);
            if (r != 1) continue;
            target += longint'(w[63:40]);
            while (cycle < target)
                step(1'b0, 1'b0, cam_pixel, cam_x, cam_y);       // idle: data held, as the capture does
            sw1 = w[39];
            step(w[38], w[37], w[36:21], w[20:11], w[10:2]);
            n_events++;
        end
        repeat (200) step(1'b0, 1'b0, cam_pixel, cam_x, cam_y);    // flush the divider
        $fwrite(fd_out, "Z %0d %0d %0d %0d %0d %0d %0d\n", cycle, stream_ok, last_error,
                good_frames, bad_frames, dropped_pixels, frame_period);
        $fclose(fd_out);
        if (fd_tap != 0) $fclose(fd_tap);
        $display("PASS: vision_replay_tb (%0d events, %0d cycles) -> %s", n_events, cycle, out_path);
        $finish;
    end

endmodule

`default_nettype wire
