`timescale 1ns/1ps
`default_nettype none

// Group 26 V2 (M2, Wilson) - tri_led_detect on a real, busy lab photo.
//
// Frames are 640x480 RGB565 made from sim/data/lab_photo.jpg by
// sim/tools/make_led_frames.py (sim/data/*.hex, one word per pixel):
//
//   photo_only  - the photo alone: every detection would be false.
//   photo_leds  - red LED on the pink phone (430,205), green LED on the
//                 purple jacaranda (165,300), blue LED on the blue sky
//                 (205,190): each on the worst background for its colour.
//
// Each frame is streamed twice (one pixel per two PCLKs); the second
// frame's result is checked, so nothing depends on reset state.
//
// A second instance with the run filter OFF (RUN_LEN = 1) sees the same
// frames. On photo_leds its blue LED must be REJECTED by led_centroid's
// fill gate: a few dozen bright window-frame edge pixels read as blue,
// stretch the blue bounding box across half the picture and drop the fill
// to almost nothing. With the filter on, the same LED is accepted.
//
// +DUMP_DIR=<dir> writes <dir>/<scene>.txt: per pixel the RGB565 word,
// the raw mask bits (R,G,B) and the filtered mask bits, for
// sim/tools/render_led_masks.py.

module tri_led_photo_tb;

    logic        clk = 0;
    logic        rst_n = 0;
    logic        frame_start = 0, frame_end = 0, pixel_valid = 0;
    logic [15:0] pixel_rgb565 = 0;
    logic [9:0]  pixel_x = 0, pixel_y = 0;

    wire dbg_r, dbg_g, dbg_b, frame_done;
    wire red_valid, green_valid, blue_valid;
    wire [9:0] red_x, red_y, green_x, green_y, blue_x, blue_y;
    wire [31:0] red_count, green_count, blue_count;
    wire [7:0]  red_confidence, green_confidence, blue_confidence;
    wire [3:0]  red_reject, green_reject, blue_reject;
    logic       gate_bypass = 1'b0;

    always #12.5 clk = ~clk;   // 40 MHz PCLK

    tri_led_detect dut (.*,
        .dbg_is_red(dbg_r), .dbg_is_green(dbg_g), .dbg_is_blue(dbg_b));

    wire nf_b; wire [31:0] nf_b_count; wire [7:0] nf_b_conf; wire [3:0] nf_b_rej;
    tri_led_detect #(.RUN_LEN(1)) nofilt (
        .clk(clk), .rst_n(rst_n), .gate_bypass(1'b0),
        .frame_start(frame_start), .frame_end(frame_end),
        .pixel_valid(pixel_valid), .pixel_rgb565(pixel_rgb565),
        .pixel_x(pixel_x), .pixel_y(pixel_y),
        .dbg_is_red(), .dbg_is_green(), .dbg_is_blue(), .frame_done(),
        .red_valid(), .red_x(), .red_y(), .red_count(), .red_confidence(), .red_reject(),
        .green_valid(), .green_x(), .green_y(), .green_count(), .green_confidence(), .green_reject(),
        .blue_valid(nf_b), .blue_x(), .blue_y(), .blue_count(nf_b_count),
        .blue_confidence(nf_b_conf), .blue_reject(nf_b_rej)
    );

    logic [15:0] img [0:640*480-1];
    string dump_dir = "";
    int fd = 0;
    bit dumping = 0;

    task automatic send_frame();
        @(negedge clk); frame_start = 1;
        @(negedge clk); frame_start = 0;
        for (int y = 0; y < 480; y++)
            for (int x = 0; x < 640; x++) begin
                pixel_valid = 0; frame_end = 0;
                @(negedge clk);
                pixel_valid  = 1;
                pixel_rgb565 = img[y*640 + x];
                pixel_x = 10'(x); pixel_y = 10'(y);
                frame_end = (x == 639 && y == 479);
                if (dumping) begin
                    #1;
                    $fwrite(fd, "%h %0d%0d%0d %0d%0d%0d\n", pixel_rgb565,
                            dut.m_r, dut.m_g, dut.m_b, dbg_r, dbg_g, dbg_b);
                end
                @(negedge clk);
            end
        pixel_valid = 0; frame_end = 0;
    endtask

    bit r_v, g_v, b_v, nf_b_v;
    task automatic wait_result();
        int guard = 0;
        while (!frame_done && guard < 200) begin @(negedge clk); guard++; end
        r_v = red_valid; g_v = green_valid; b_v = blue_valid; nf_b_v = nf_b;
        @(negedge clk);
    endtask

    int errors = 0, checks = 0;

    task automatic expect_bit(string what, bit got, bit exp);
        checks++;
        if (got !== exp) begin errors++; $display("  FAIL %s: got %0b expected %0b", what, got, exp); end
        else $display("  ok   %s = %0b", what, got);
    endtask

    task automatic expect_near(string what, int got, int exp, int tol);
        checks++;
        if (got < exp - tol || got > exp + tol) begin
            errors++; $display("  FAIL %s: got %0d expected %0d +/-%0d", what, got, exp, tol);
        end else $display("  ok   %s = %0d (true %0d)", what, got, exp);
    endtask

    task automatic run_scene(string scene);
        $readmemh({"sim/data/", scene, ".hex"}, img);
        send_frame(); wait_result();               // warm-up frame
        if (dump_dir != "") begin
            fd = $fopen({dump_dir, "/", scene, ".txt"}, "w");
            dumping = (fd != 0);
        end
        send_frame(); wait_result();
        if (dumping) begin $fclose(fd); dumping = 0; end
        $display("%s: R valid=%0b (%0d,%0d) n=%0d conf=%0d rej=%b | G valid=%0b (%0d,%0d) n=%0d conf=%0d rej=%b | B valid=%0b (%0d,%0d) n=%0d conf=%0d rej=%b",
                 scene,
                 r_v, red_x, red_y, red_count, red_confidence, red_reject,
                 g_v, green_x, green_y, green_count, green_confidence, green_reject,
                 b_v, blue_x, blue_y, blue_count, blue_confidence, blue_reject);
    endtask

    initial begin
        void'($value$plusargs("DUMP_DIR=%s", dump_dir));
        repeat (4) @(negedge clk);
        rst_n = 1;

        run_scene("photo_only");
        expect_bit("photo_only red valid",   r_v, 0);
        expect_bit("photo_only green valid", g_v, 0);
        expect_bit("photo_only blue valid",  b_v, 0);

        run_scene("photo_leds");
        expect_bit ("photo_leds red valid",   r_v, 1);
        expect_bit ("photo_leds green valid", g_v, 1);
        expect_bit ("photo_leds blue valid",  b_v, 1);
        expect_near("red x",   int'(red_x),   430, 2);
        expect_near("red y",   int'(red_y),   205, 2);
        expect_near("green x", int'(green_x), 165, 2);
        expect_near("green y", int'(green_y), 300, 2);
        expect_near("blue x",  int'(blue_x),  205, 2);
        expect_near("blue y",  int'(blue_y),  190, 2);

        $display("  filter OFF, blue: valid=%0b pixels=%0d confidence=%0d reject=%b",
                 nf_b_v, nf_b_count, nf_b_conf, nf_b_rej);
        expect_bit ("filter OFF: blue LED rejected",      nf_b_v, 0);
        expect_bit ("filter OFF: blue rejected on fill",  nf_b_rej[3], 1);

        if (errors != 0)
            $fatal(1, "FAIL: tri_led_photo_tb: %0d errors in %0d checks", errors, checks);
        $display("PASS: tri_led_photo_tb (%0d checks, lab photo with and without LEDs)", checks);
        $finish;
    end

endmodule

`default_nettype wire
