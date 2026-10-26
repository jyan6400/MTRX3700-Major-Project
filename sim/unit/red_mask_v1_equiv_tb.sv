`timescale 1ns/1ps
`default_nettype none

// Self-checking equivalence test for the V2 red_mask wrapper (M2, Wilson).
//
// 1. V1 equivalence: for every RGB565 code (x valid 0/1), the V2
//    red_mask with default parameters must produce exactly the same
//    mask_valid / is_red / mask_x / mask_y as the V1 red_mask
//    (kept as sim/unit/red_mask_v1_ref.sv). This guarantees the static
//    major_project_top behaves identically on red.
// 2. The default is_red equals led_colour_mask's red output (the same
//    classifier feeds tri_led_detect).
// 3. TARGET_COLOUR = 1 / 2 routes green / blue onto is_red.
// Every instance uses the exact V1 port list, as major_project_top does.

module red_mask_v1_equiv_tb;

    logic [15:0] rgb;
    logic [9:0]  px, py;
    logic        pv;

    // DUT, default build (as instantiated by major_project_top)
    wire d_mv, d_r;
    wire [9:0] d_x, d_y;
    red_mask dut (
        .pixel_rgb565(rgb), .pixel_x(px), .pixel_y(py), .pixel_valid(pv),
        .mask_valid(d_mv), .is_red(d_r), .mask_x(d_x), .mask_y(d_y)
    );

    // V1 golden reference
    wire v_mv, v_r;
    wire [9:0] v_x, v_y;
    red_mask_v1_ref v1 (
        .pixel_rgb565(rgb), .pixel_x(px), .pixel_y(py), .pixel_valid(pv),
        .mask_valid(v_mv), .is_red(v_r), .mask_x(v_x), .mask_y(v_y)
    );

    // Tri-colour classifier reference
    wire c_mv, c_r, c_g, c_b;
    wire [9:0] c_x, c_y;
    led_colour_mask cls (
        .pixel_rgb565(rgb), .pixel_x(px), .pixel_y(py), .pixel_valid(pv),
        .mask_valid(c_mv), .is_red(c_r), .is_green(c_g), .is_blue(c_b),
        .mask_x(c_x), .mask_y(c_y)
    );

    // Target-colour variants
    wire g_mv, g_t, b_mv, b_t;
    wire [9:0] g_x, g_y, b_x, b_y;
    red_mask #(.TARGET_COLOUR(2'd1)) dut_green (
        .pixel_rgb565(rgb), .pixel_x(px), .pixel_y(py), .pixel_valid(pv),
        .mask_valid(g_mv), .is_red(g_t), .mask_x(g_x), .mask_y(g_y)
    );
    red_mask #(.TARGET_COLOUR(2'd2)) dut_blue (
        .pixel_rgb565(rgb), .pixel_x(px), .pixel_y(py), .pixel_valid(pv),
        .mask_valid(b_mv), .is_red(b_t), .mask_x(b_x), .mask_y(b_y)
    );

    int checks = 0, errors = 0;
    int n_red = 0;

    task automatic check(string what, logic [9:0] got, logic [9:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            if (errors <= 20)
                $display("FAIL %s rgb=%h pv=%0b got=%0h exp=%0h", what, rgb, pv, got, exp);
        end
    endtask

    initial begin
        for (int v = 0; v < 65536; v++) begin
            for (int p = 0; p < 2; p++) begin
                rgb = v[15:0];
                pv  = p[0];
                px  = 10'($urandom_range(0, 639));
                py  = 10'($urandom_range(0, 479));
                #1;
                // 1. V1 equivalence
                check("v1 mask_valid", 10'(d_mv), 10'(v_mv));
                check("v1 is_red",     10'(d_r),  10'(v_r));
                check("v1 mask_x",     d_x, v_x);
                check("v1 mask_y",     d_y, v_y);
                // 2. default build agrees with the tri-colour classifier
                check("is_red == classifier red", 10'(d_r), 10'(c_r));
                // 3. target select
                check("TARGET=green", 10'(g_t), 10'(c_g));
                check("TARGET=blue",  10'(b_t), 10'(c_b));
                check("TARGET=green x", g_x, px);
                check("TARGET=blue y",  b_y, py);
                if (pv) n_red += int'(d_r);
            end
        end

        $display("red_mask: %0d of 65536 RGB565 codes classified as red (identical to V1)", n_red);
        if (errors != 0)
            $fatal(1, "FAIL: red_mask_v1_equiv_tb: %0d errors in %0d checks", errors, checks);
        $display("PASS: red_mask_v1_equiv_tb (%0d checks, all 65536 RGB565 codes)", checks);
        $finish;
    end

endmodule

`default_nettype wire
