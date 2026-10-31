`timescale 1ns/1ps
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off BLKSEQ */
// Group 26 - beacon visual odometry (Wilson, M2): a drive past the beacons, frame by frame.
//
//   background.hex + scene file (golden/vo/vo_scene.py) -> frames drawn here, pixel for pixel as the
//   Python draws them -> camera_stream_adapter -> beacon_vo (detect, CDC, geometry, tracker, stand-off)
//
// Two clocks, as on the robot: PCLK 40 MHz (the camera side) and CLOCK_50. A frame whose scene line
// says good = 0 is cut short at row 200, so the adapter closes it empty at the next frame_start.
// The bench writes every detection (D, PCLK) and every VO update (V, CLOCK_50) to +OUT;
// golden/vo/vo_check.py compares them with the golden model and with the ground truth.
//
// Plusargs: +BG=<background.hex> +SCENE=<scene.txt> +OUT=<file.rec>
module beacon_vo_scene_tb;
    localparam int W = 640, H = 480;

    logic pclk = 1'b0;  always #12.5 pclk = ~pclk;      // 40 MHz
    logic clk  = 1'b0;  always #10   clk  = ~clk;       // 50 MHz
    logic rst_n_p = 1'b0, rst_n = 1'b0;

    // camera stream into the adapter
    logic        cam_fs = 1'b0, cam_valid = 1'b0;
    logic [15:0] cam_pixel = '0;
    logic [9:0]  cam_x = '0;
    logic [8:0]  cam_y = '0;
    wire         frame_start, frame_end, pixel_valid, frame_good, stream_ok;
    wire [15:0]  pixel_rgb565, good_frames, bad_frames, dropped_pixels;
    wire [9:0]   pixel_x, pixel_y;
    wire [1:0]   last_error;
    wire [23:0]  frame_period;
    camera_stream_adapter #(.W(W), .H(H)) u_adapter (
        .clk(pclk), .rst_n(rst_n_p),
        .cam_frame_start(cam_fs), .cam_valid(cam_valid), .cam_pixel(cam_pixel), .cam_x(cam_x), .cam_y(cam_y),
        .frame_start(frame_start), .frame_end(frame_end), .pixel_valid(pixel_valid),
        .pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y),
        .frame_good(frame_good), .stream_ok(stream_ok), .last_error(last_error),
        .good_frames(good_frames), .bad_frames(bad_frames), .dropped_pixels(dropped_pixels),
        .frame_period(frame_period));

    // the VO, with the inputs the scene gives each frame
    logic [1:0]  sel = '0;
    logic        so_en = 1'b0, so_appr = 1'b0;
    logic [15:0] so_tgt = '0;
    wire         vo_updated, track_valid, track_settled, so_reached, so_lost, pair_valid, pair_ok;
    wire signed [15:0] range_mm, vel_mm_s, lateral_mm, disp_mm, heading_q12;
    wire [2:0]   led_valid, dbg_mask;
    wire [47:0]  led_fwd, led_lat;
    wire         det_dropped, cdc_overrun;
    beacon_vo u_vo (
        .pclk(pclk), .rst_n_pclk(rst_n_p),
        .frame_start(frame_start), .frame_end(frame_end), .frame_good(frame_good),
        .pixel_valid(pixel_valid), .pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y),
        .dbg_mask(dbg_mask),
        .clk(clk), .rst_n(rst_n), .sel(sel), .standoff_enable(so_en), .standoff_approaching(so_appr),
        .standoff_target_mm(so_tgt),
        .vo_updated(vo_updated), .track_valid(track_valid), .track_settled(track_settled),
        .range_mm(range_mm), .vel_mm_s(vel_mm_s), .lateral_mm(lateral_mm), .disp_mm(disp_mm),
        .standoff_reached(so_reached), .standoff_lost(so_lost),
        .led_valid(led_valid), .led_forward_mm(led_fwd), .led_lateral_mm(led_lat),
        .pair_valid(pair_valid), .pair_ok(pair_ok), .heading_q12(heading_q12),
        .det_dropped(det_dropped), .cdc_overrun(cdc_overrun));

    // ---------------- records ----------------
    function automatic int s16(input logic [47:0] bus, input int k);
        logic signed [15:0] t;
        t = bus[k*16 +: 16];
        return int'(t);
    endfunction
    int fd_out;
    int n_det = 0, n_upd = 0, n_drop = 0, n_ovr = 0;
    always @(posedge pclk) begin
        if (u_vo.u_detect.done) begin
            $fwrite(fd_out, "D %0d %0d", n_det, u_vo.u_detect.res_good);
            for (int k = 0; k < 3; k++)
                $fwrite(fd_out, " %0d %0d %0d %0d %0d %0d %0d %0d %0d", u_vo.u_detect.valid[k],
                        u_vo.u_detect.x_q4[k*14 +: 14], u_vo.u_detect.y_q4[k*14 +: 14],
                        u_vo.u_detect.count[k*19 +: 19], u_vo.u_detect.flags[k*5 +: 5],
                        u_vo.u_detect.bbox[k*40+30 +: 10], u_vo.u_detect.bbox[k*40+20 +: 10],
                        u_vo.u_detect.bbox[k*40+10 +: 10], u_vo.u_detect.bbox[k*40 +: 10]);
            $fwrite(fd_out, "\n");
            n_det++;
        end
        if (det_dropped) n_drop++;
        if (cdc_overrun) n_ovr++;
    end
    // log one clock after vo_updated: the tracker's outputs are new at vo_updated, the stand-off's a clock later
    logic vo_updated_q = 1'b0;
    always @(posedge clk) vo_updated_q <= vo_updated;
    always @(posedge clk) if (vo_updated_q) begin
        $fwrite(fd_out, "V %0d", n_upd);
        for (int k = 0; k < 3; k++)
            $fwrite(fd_out, " %0d %0d %0d", led_valid[k], led_fwd[k*16 +: 16], s16(led_lat, k));
        $fwrite(fd_out, " %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d\n",
                pair_valid, pair_ok, heading_q12, track_valid, track_settled, range_mm, vel_mm_s,
                lateral_mm, disp_mm, u_vo.u_track.coast, so_reached, so_lost, sel);
        n_upd++;
    end

    // ---------------- frames ----------------
    logic [15:0] bg  [W * H];
    logic [15:0] fr  [W * H];
    int fd_sc, r, n_frames = 0, expect_upd = 0;
    string bg_path, sc_path, out_path, line, tag;
    int f_sel, f_good, f_en, f_appr, f_tgt, f_n, d_k, d_rgb, d_u, d_v, d_r;

    task automatic draw_disc(input int rgb, input int U, input int V, input int R);
        int xa, xb, ya, yb, dx, dy;
        xa = (U - R) / 16 - 2; xb = (U + R) / 16 + 2;
        ya = (V - R) / 16 - 2; yb = (V + R) / 16 + 2;
        if (xa < 0) xa = 0;
        if (ya < 0) ya = 0;
        if (xb > W - 1) xb = W - 1;
        if (yb > H - 1) yb = H - 1;
        for (int y = ya; y <= yb; y++)
            for (int x = xa; x <= xb; x++) begin
                dx = 16 * x - U;
                dy = 16 * y - V;
                if (longint'(dx) * dx + longint'(dy) * dy <= longint'(R) * R) fr[y * W + x] = 16'(rgb);
            end
    endtask

    // one frame: frame_start, rows of W pixels with a short gap, an end gap. good = 0 stops at row 200.
    task automatic stream(input int good);
        @(negedge pclk); cam_fs = 1'b1; @(negedge pclk); cam_fs = 1'b0;
        repeat (8) @(negedge pclk);
        for (int y = 0; y < (good ? H : 200); y++) begin
            for (int x = 0; x < W; x++) begin
                cam_valid = 1'b1; cam_x = 10'(x); cam_y = 9'(y); cam_pixel = fr[y * W + x];
                @(negedge pclk);
            end
            cam_valid = 1'b0;
            repeat (16) @(negedge pclk);
        end
        repeat (64) @(negedge pclk);
    endtask

    initial begin
        if (!$value$plusargs("BG=%s", bg_path))    $fatal(1, "+BG=<background.hex>");
        if (!$value$plusargs("SCENE=%s", sc_path)) $fatal(1, "+SCENE=<scene.txt>");
        if (!$value$plusargs("OUT=%s", out_path))  $fatal(1, "+OUT=<file.rec>");
        $readmemh(bg_path, bg);
        fd_sc = $fopen(sc_path, "r");
        if (fd_sc == 0) $fatal(1, "cannot open %s", sc_path);
        fd_out = $fopen(out_path, "w");
        repeat (4) @(posedge pclk);
        rst_n_p = 1'b1; rst_n = 1'b1;
        repeat (4) @(posedge pclk);

        while ($fgets(line, fd_sc) > 0) begin
            if (line.len() < 2 || line.getc(0) != "F") continue;
            r = $sscanf(line, "%s %d %d %d %d %d %d", tag, f_sel, f_good, f_en, f_appr, f_tgt, f_n);
            for (int i = 0; i < W * H; i++) fr[i] = bg[i];
            for (int i = 0; i < f_n; i++) begin
                r = $fgets(line, fd_sc);
                r = $sscanf(line, "%s %d %d %d %d %d", tag, d_k, d_rgb, d_u, d_v, d_r);
                draw_disc(d_rgb, d_u, d_v, d_r);
            end
            // this frame's inputs hold until its result is in (a cut-short frame's result comes
            // in at the next frame's start, with the next frame's inputs: vo_check.py knows)
            @(negedge clk);
            sel = 2'(f_sel); so_en = f_en[0]; so_appr = f_appr[0]; so_tgt = 16'(f_tgt);
            stream(f_good);
            n_frames++;
            if (f_good) begin
                expect_upd = n_frames;
                fork
                    begin wait (n_upd >= expect_upd); end
                    begin repeat (20000) @(posedge clk); end
                join_any
                disable fork;
            end
        end
        repeat (2000) @(posedge clk);
        $fclose(fd_out);
        if (n_upd == n_frames && n_det == n_frames && n_drop == 0 && n_ovr == 0)
            $display("PASS: beacon_vo_scene_tb (%0d frames, %0d detections, %0d VO updates) -> %s", n_frames, n_det, n_upd, out_path);
        else
            $display("FAIL: beacon_vo_scene_tb: %0d frames, %0d detections, %0d updates, %0d dropped, %0d CDC overruns",
                     n_frames, n_det, n_upd, n_drop, n_ovr);
        $finish;
    end
endmodule
