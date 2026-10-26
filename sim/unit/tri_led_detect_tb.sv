`timescale 1ns/1ps
`default_nettype none

// Self-checking frame-level test for tri_led_detect
// (led_colour_mask + colour_run_filter + 3 x led_centroid).
//
// Streams full 640x480 frames, one pixel every 2 PCLK cycles (as the
// OV7670 delivers RGB565: two bytes per pixel).
//
// Frame A: red, green and blue LED discs with saturated white cores,
//          dim background, ~300 isolated single-pixel colour speckles.
//          Expect all three valid, centroids within +/-1 px of centres.
// Frame B: speckles only, plus a 3-pixel red run that wraps from the
//          end of one row to the start of the next. Expect nothing valid.
// Frame C: only the red LED, moved. Expect red valid at new place,
//          green/blue invalid.
// Frame A also checks exact per-colour pixel counts against a model of
// the run filter (disc ring minus the first 2 pixels of every row run).
//
// A second instance with the run filter disabled (RUN_LEN = 1) sees the
// same frames. On Frame A it must REJECT all three LEDs: the speckles
// stretch each colour's bounding box, so led_centroid's V2 fill gate
// fails. This is the reason the filter sits in front of the centroids.

module tri_led_detect_tb;

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

    always #20 clk = ~clk;   // 25 MHz-ish PCLK, value irrelevant

    tri_led_detect dut (.*,
        .dbg_is_red(dbg_r), .dbg_is_green(dbg_g), .dbg_is_blue(dbg_b));

    // Same pipeline with the run filter turned off.
    wire nf_done, nf_r, nf_g, nf_b;
    wire [3:0] nf_r_rej, nf_g_rej, nf_b_rej;
    tri_led_detect #(.RUN_LEN(1)) nofilt (
        .clk(clk), .rst_n(rst_n), .gate_bypass(1'b0),
        .frame_start(frame_start), .frame_end(frame_end),
        .pixel_valid(pixel_valid), .pixel_rgb565(pixel_rgb565),
        .pixel_x(pixel_x), .pixel_y(pixel_y),
        .dbg_is_red(), .dbg_is_green(), .dbg_is_blue(),
        .frame_done(nf_done),
        .red_valid(nf_r), .red_x(), .red_y(), .red_count(),
        .red_confidence(), .red_reject(nf_r_rej),
        .green_valid(nf_g), .green_x(), .green_y(), .green_count(),
        .green_confidence(), .green_reject(nf_g_rej),
        .blue_valid(nf_b), .blue_x(), .blue_y(), .blue_count(),
        .blue_confidence(), .blue_reject(nf_b_rej)
    );

    // ---------------- scene description ---------------------------------
    bit          led_on[3];
    int          led_cx[3], led_cy[3], led_rad[3];
    logic [15:0] led_col[3];

    localparam logic [15:0] C_RED   = {5'd30, 6'd10, 5'd5};
    localparam logic [15:0] C_GREEN = {5'd5,  6'd58, 5'd14};
    localparam logic [15:0] C_BLUE  = {5'd6,  6'd34, 5'd30};
    localparam logic [15:0] C_WHITE = 16'hFFFF;
    localparam logic [15:0] C_BG    = {5'd6, 6'd12, 5'd6};
    localparam int          CORE_R  = 3;

    bit speckles;
    bit wrap_run;

    function automatic logic [15:0] scene(int x, int y);
        int dx, dy, d2;
        for (int i = 0; i < 3; i++) begin
            if (!led_on[i]) continue;
            dx = x - led_cx[i]; dy = y - led_cy[i]; d2 = dx*dx + dy*dy;
            if (d2 <= CORE_R*CORE_R)           return C_WHITE;
            if (d2 <= led_rad[i]*led_rad[i]) return led_col[i];
        end
        if (wrap_run && ((y == 10 && (x == 638 || x == 639)) || (y == 11 && x == 0)))
            return C_RED;
        if (speckles && (x % 37 == 5) && (y % 23 == 7)) begin
            case ((x + y) % 3)
                0: return C_RED;
                1: return C_GREEN;
                default: return C_BLUE;
            endcase
        end
        return C_BG;
    endfunction

    // ---------------- stream one frame ----------------------------------
    task automatic send_frame();
        @(negedge clk);
        frame_start = 1;
        @(negedge clk);
        frame_start = 0;
        for (int y = 0; y < 480; y++) begin
            for (int x = 0; x < 640; x++) begin
                // byte 1: nothing valid
                pixel_valid = 0;
                frame_end   = 0;
                @(negedge clk);
                // byte 2: pixel complete
                pixel_valid  = 1;
                pixel_rgb565 = scene(x, y);
                pixel_x      = 10'(x);
                pixel_y      = 10'(y);
                frame_end    = (x == 639 && y == 479);
                @(negedge clk);
            end
        end
        pixel_valid = 0;
        frame_end   = 0;
    endtask

    // ---------------- result capture ------------------------------------
    bit got_r, got_g, got_b;
    bit nf_got_r, nf_got_g, nf_got_b;

    task automatic wait_result();
        int guard = 0;
        while (!frame_done) begin
            @(negedge clk);
            guard++;
            if (guard > 200) begin
                $display("FAIL no frame_done after frame");
                errors++;
                return;
            end
        end
        got_r = red_valid; got_g = green_valid; got_b = blue_valid;
        nf_got_r = nf_r; nf_got_g = nf_g; nf_got_b = nf_b;
        @(negedge clk);
    endtask

    int errors = 0, checks = 0;

    task automatic set_led(int i, bit on, int cx, int cy, int rad, logic [15:0] col);
        led_on[i] = on; led_cx[i] = cx; led_cy[i] = cy; led_rad[i] = rad; led_col[i] = col;
    endtask

    task automatic expect_bit(string what, bit got, bit exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("FAIL %s: got %0b expected %0b", what, got, exp);
        end else $display("  ok  %s = %0b", what, got);
    endtask

    task automatic expect_near(string what, int got, int exp, int tol);
        checks++;
        if (got < exp - tol || got > exp + tol) begin
            errors++;
            $display("FAIL %s: got %0d expected %0d +/-%0d", what, got, exp, tol);
        end else $display("  ok  %s = %0d (expected %0d)", what, got, exp);
    endtask

    // Count of pixels the filtered mask should accept for one disc ring:
    // per row, the coloured pixels form runs; each run of length L >= 3
    // contributes L-2 pixels.
    function automatic int expected_count(int i);
        int total = 0, run, d2, dx, dy;
        for (int y = 0; y < 480; y++) begin
            run = 0;
            for (int x = 0; x <= 640; x++) begin
                bit c = 0;
                if (x < 640) begin
                    dx = x - led_cx[i]; dy = y - led_cy[i]; d2 = dx*dx + dy*dy;
                    c = (d2 > CORE_R*CORE_R) && (d2 <= led_rad[i]*led_rad[i]);
                end
                if (c) run++;
                else begin
                    if (run >= 3) total += run - 2;
                    run = 0;
                end
            end
        end
        return total;
    endfunction

    initial begin
        set_led(0, 1, 100, 200, 10, C_RED);
        set_led(1, 1, 320, 240, 8, C_GREEN);
        set_led(2, 1, 540, 100, 12, C_BLUE);

        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (4) @(posedge clk);

        // ---------- Frame A ----------
        $display("Frame A: three LEDs + speckle noise");
        speckles = 1; wrap_run = 0;
        send_frame(); wait_result();
        expect_bit ("A red valid",   got_r, 1);
        expect_bit ("A green valid", got_g, 1);
        expect_bit ("A blue valid",  got_b, 1);
        expect_near("A red x",   red_x,   100, 1);
        expect_near("A red y",   red_y,   200, 1);
        expect_near("A green x", green_x, 320, 1);
        expect_near("A green y", green_y, 240, 1);
        expect_near("A blue x",  blue_x,  540, 1);
        expect_near("A blue y",  blue_y,  100, 1);
        expect_near("A red count",   red_count,   expected_count(0), 0);
        expect_near("A green count", green_count, expected_count(1), 0);
        expect_near("A blue count",  blue_count,  expected_count(2), 0);
        $display("  confidence R/G/B = %0d / %0d / %0d (of 255)",
                 red_confidence, green_confidence, blue_confidence);
        expect_bit ("A no-filter red valid",   nf_got_r, 0);
        expect_bit ("A no-filter green valid", nf_got_g, 0);
        expect_bit ("A no-filter blue valid",  nf_got_b, 0);
        expect_bit ("A no-filter red fill-gate reject",   nf_r_rej[3], 1);
        expect_bit ("A no-filter green fill-gate reject", nf_g_rej[3], 1);
        expect_bit ("A no-filter blue fill-gate reject",  nf_b_rej[3], 1);

        // ---------- Frame B ----------
        $display("Frame B: noise only + row-wrapping red run");
        led_on[0] = 0; led_on[1] = 0; led_on[2] = 0;
        speckles = 1; wrap_run = 1;
        send_frame(); wait_result();
        expect_bit ("B red valid",   got_r, 0);
        expect_bit ("B green valid", got_g, 0);
        expect_bit ("B blue valid",  got_b, 0);
        expect_near("B red count",   red_count,   0, 0);
        expect_near("B green count", green_count, 0, 0);
        expect_near("B blue count",  blue_count,  0, 0);

        // ---------- Frame C ----------
        $display("Frame C: red LED only, moved");
        set_led(0, 1, 450, 380, 9, C_RED);
        speckles = 1; wrap_run = 0;
        send_frame(); wait_result();
        expect_bit ("C red valid",   got_r, 1);
        expect_bit ("C green valid", got_g, 0);
        expect_bit ("C blue valid",  got_b, 0);
        expect_near("C red x", red_x, 450, 1);
        expect_near("C red y", red_y, 380, 1);

        if (errors != 0)
            $fatal(1, "FAIL: tri_led_detect_tb: %0d errors in %0d checks", errors, checks);
        $display("PASS: tri_led_detect_tb (%0d checks, 3 full 640x480 frames)", checks);
        $finish;
    end

endmodule

`default_nettype wire
