`timescale 1ns/1ps
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off UNUSEDSIGNAL */
// Group 26 - beacon visual odometry (Wilson, M2): beacon_geometry -> beacon_tracker -> vo_standoff
// on thousands of random detections (golden/vo/vo_check.py writes them: every corner of the
// geometry, out-of-range and impossible LEDs, outliers, dropouts, bay changes, stand-offs).
//
// Vector file, one line a frame:  S <valid3> <x0> <y0> <x1> <y1> <x2> <y2> <sel> <en> <appr> <tgt>
// Output, one line a frame, as beacon_vo_scene_tb's V records.
// Plusargs: +VEC=<file> +OUT=<file>; calibration through -G parameters at build time.
module beacon_math_tb #(
    parameter int F_Q4 = 8960, parameter int CX_Q4 = 5112, parameter int CY_Q4 = 3832,
    parameter int SIN_Q14 = 3406, parameter int COS_Q14 = 16026, parameter int DH_MM = 150
);
    logic clk = 1'b0; always #10 clk = ~clk;
    logic rst_n = 1'b0;

    logic        start = 1'b0;
    logic [2:0]  dv = '0;
    logic [41:0] dx = '0, dy = '0;
    logic [1:0]  sel = '0;
    logic        en = 1'b0, appr = 1'b0;
    logic [15:0] tgt = '0;

    wire        g_done, g_busy, pair_valid, pair_ok;
    wire [2:0]  lv;
    wire [47:0] fwd, lat;
    wire [3:0]  pair;
    wire signed [15:0] heading;
    beacon_geometry #(.F_Q4(F_Q4), .CX_Q4(CX_Q4), .CY_Q4(CY_Q4), .SIN_Q14(SIN_Q14), .COS_Q14(COS_Q14),
                      .DH_MM(DH_MM)) u_geom (
        .clk(clk), .rst_n(rst_n), .start(start), .det_valid(dv), .det_x_q4(dx), .det_y_q4(dy),
        .done(g_done), .led_valid(lv), .forward_mm(fwd), .lateral_mm(lat),
        .pair_valid(pair_valid), .pair_ok(pair_ok), .pair(pair), .heading_q12(heading), .busy(g_busy));

    wire updated, tv, settled;
    wire signed [15:0] range_mm, vel, lat_o, disp;
    wire [2:0] coast;
    beacon_tracker u_track (
        .clk(clk), .rst_n(rst_n), .update(g_done), .sel(sel), .led_valid(lv), .forward_mm(fwd),
        .lateral_mm(lat), .updated(updated), .track_valid(tv), .settled(settled), .range_mm(range_mm),
        .vel_mm_s(vel), .lateral_out_mm(lat_o), .disp_mm(disp), .coast(coast));

    wire reached, lost;
    vo_standoff u_so (
        .clk(clk), .rst_n(rst_n), .enable(en), .approaching(appr), .target_mm(tgt), .updated(updated),
        .track_valid(tv), .range_mm(range_mm), .reached(reached), .lost(lost));

    function automatic int s16(input logic [47:0] bus, input int k);
        logic signed [15:0] t;
        t = bus[k*16 +: 16];
        return int'(t);
    endfunction
    int fd_in, fd_out, r, n = 0;
    string vec, outp, line, tag;
    int v3, x0, y0, x1, y1, x2, y2, s, e, a, t;
    initial begin
        if (!$value$plusargs("VEC=%s", vec)) $fatal(1, "+VEC=<file>");
        if (!$value$plusargs("OUT=%s", outp)) $fatal(1, "+OUT=<file>");
        fd_in = $fopen(vec, "r");
        fd_out = $fopen(outp, "w");
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        while ($fgets(line, fd_in) > 0) begin
            if (line.getc(0) != "S") continue;
            r = $sscanf(line, "%s %d %d %d %d %d %d %d %d %d %d %d", tag, v3, x0, y0, x1, y1, x2, y2, s, e, a, t);
            @(negedge clk);
            dv = 3'(v3);
            dx = {14'(x2), 14'(x1), 14'(x0)};
            dy = {14'(y2), 14'(y1), 14'(y0)};
            sel = 2'(s); en = e[0]; appr = a[0]; tgt = 16'(t);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            wait (updated);
            @(posedge clk);                  // vo_standoff takes `updated` on this edge
            @(negedge clk);
            $fwrite(fd_out, "V %0d", n);
            for (int k = 0; k < 3; k++)
                $fwrite(fd_out, " %0d %0d %0d", lv[k], fwd[k*16 +: 16], s16(lat, k));
            $fwrite(fd_out, " %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d\n",
                    pair_valid, pair_ok, heading, tv, settled, range_mm, vel, lat_o, disp, coast, reached, lost, sel);
            n++;
        end
        $fclose(fd_out);
        $display("PASS: beacon_math_tb (%0d frames) -> %s", n, outp);
        $finish;
    end
endmodule
