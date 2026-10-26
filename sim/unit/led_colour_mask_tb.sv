`timescale 1ns/1ps
`default_nettype none

// Self-checking unit test for led_colour_mask.
//  1. Exhaustive: all 65536 RGB565 values, valid = 1 and valid = 0.
//     - compared with an independent integer reference model
//     - at most one colour flag high per pixel
//     - valid = 0 forces all flags low
//  2. Named cases: pure/dim/mixed colours, white, black, secondaries.
//  3. Coordinate and valid pass-through.

module led_colour_mask_tb;

    logic [15:0] rgb;
    logic [9:0]  px, py;
    logic        pv;
    wire         mv, ir, ig, ib;
    wire  [9:0]  mx, my;

    led_colour_mask dut (
        .pixel_rgb565(rgb), .pixel_x(px), .pixel_y(py), .pixel_valid(pv),
        .mask_valid(mv), .is_red(ir), .is_green(ig), .is_blue(ib),
        .mask_x(mx), .mask_y(my)
    );

    int errors = 0;
    int checks = 0;
    int n_red = 0, n_green = 0, n_blue = 0;

    // Independent reference: plain integers, defaults copied from the RTL.
    function automatic bit ref_dom(int p, int a, int b2,
                                   int mn, int amax, int bmax, int m);
        return (p >= mn) && (a <= amax) && (b2 <= bmax) &&
               (p >= a + m) && (p >= b2 + m);
    endfunction

    // Green ceilings use the full 6-bit green value (V1 convention).
    function automatic bit ref_red(int r, int g6, int g5, int b);
        return (r >= 20) && (g6 <= 24) && (b <= 12) && (r >= g5 + 8) && (r >= b + 8);
    endfunction
    function automatic bit ref_blue(int r, int g6, int g5, int b);
        return (b >= 18) && (r <= 14) && (g6 <= 45) && (b >= r + 6) && (b >= g5 + 6);
    endfunction

    task automatic check(string what, bit got, bit exp);
        checks++;
        if (got !== exp) begin
            errors++;
            if (errors < 20)
                $display("FAIL %s rgb=%h got=%0b exp=%0b", what, rgb, got, exp);
        end
    endtask

    task automatic named(string name, int r5, int g6, int b5,
                         bit er, bit eg, bit eb);
        rgb = {r5[4:0], g6[5:0], b5[4:0]};
        pv  = 1'b1;
        #1;
        check({name, " red"},   ir, er);
        check({name, " green"}, ig, eg);
        check({name, " blue"},  ib, eb);
    endtask

    initial begin
        int r, g, b;
        bit er, eg, eb;
        px = 10'd123; py = 10'd456;

        // ---- 1. exhaustive -------------------------------------------
        for (int v = 0; v < 65536; v++) begin
            rgb = v[15:0];
            r = v[15:11]; g = v[10:6]; b = v[4:0];
            er = ref_red(r, v[10:5], g, b);
            eg = ref_dom(g, r, b, 18, 14, 20, 6);
            eb = ref_blue(r, v[10:5], g, b);

            pv = 1'b1; #1;
            check("exh red",   ir, er);
            check("exh green", ig, eg);
            check("exh blue",  ib, eb);
            checks++;
            if ((int'(ir) + int'(ig) + int'(ib)) > 1) begin
                errors++;
                $display("FAIL exclusivity rgb=%h", rgb);
            end
            n_red += ir; n_green += ig; n_blue += ib;

            pv = 1'b0; #1;
            check("gate red",   ir, 1'b0);
            check("gate green", ig, 1'b0);
            check("gate blue",  ib, 1'b0);
        end

        // ---- 2. named cases (r5, g6, b5) ------------------------------
        named("pure red",      31,  0,  0, 1, 0, 0);
        named("pure green",     0, 63,  0, 0, 1, 0);
        named("pure blue",      0,  0, 31, 0, 0, 1);
        named("red led halo",  28, 14,  6, 1, 0, 0);
        named("green w/ blue",  6, 56, 16, 0, 1, 0);
        named("blue w/ cyan",   8, 36, 30, 0, 0, 1);
        named("white",         31, 63, 31, 0, 0, 0);
        named("black",          0,  0,  0, 0, 0, 0);
        named("grey",          16, 32, 16, 0, 0, 0);
        named("yellow",        31, 63,  0, 0, 0, 0);
        named("cyan",           0, 63, 31, 0, 0, 0);
        named("magenta",       31,  0, 31, 0, 0, 0);
        named("dim red",       12,  0,  0, 0, 0, 0);
        named("orange",        31, 40,  0, 0, 0, 0);
        named("pink skin",     28, 36, 20, 0, 0, 0);

        // ---- 3. pass-through -----------------------------------------
        pv = 1'b1; px = 10'd639; py = 10'd479; #1;
        check("x passthru", mx == 10'd639, 1'b1);
        check("y passthru", my == 10'd479, 1'b1);
        check("valid passthru hi", mv, 1'b1);
        pv = 1'b0; #1;
        check("valid passthru lo", mv, 1'b0);

        $display("Classified codes: red=%0d green=%0d blue=%0d of 65536",
                 n_red, n_green, n_blue);
        if (errors != 0)
            $fatal(1, "FAIL: led_colour_mask_tb: %0d errors in %0d checks", errors, checks);
        $display("PASS: led_colour_mask_tb (%0d checks, all 65536 RGB565 codes)", checks);
        $finish;
    end

endmodule

`default_nettype wire
