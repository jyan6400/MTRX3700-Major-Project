`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Major Project - Group 26
// Self-checking testbench for camera_stream_adapter (V2 frame integrity).
//
// Small frames (W = 8, H = 4) keep the test fast; the full 640 x 480 path is
// exercised with the Ed camera model in sim/integration/camera_capture_vision_tb.sv.
//
// Three layers of checking:
//   1. Reference model. A behavioural model in this file, written
//      independently of the RTL, predicts every output every cycle. The DUT
//      is compared against it on every clock (outputs, counters, flags).
//   2. Scenario checks. Each scenario states its expected counters, error
//      flags and frame result, and prints PASS with its name.
//   3. Downstream effect. red_mask and led_centroid are connected to the DUT
//      output exactly as in major_project_top. Every frame contains red
//      pixels, and the bench checks that a good frame gives the correct
//      centroid and that every rejected frame gives target_valid = 0.
//
// Pixels arrive one every two clocks with gaps between rows and frames, as
// from ov7670_capture.
//
// Run (Verilator 5.050), from the repository root:
//   $ verilator --binary --timing -Wno-CASEINCOMPLETE --top-module camera_stream_adapter_tb \
//     --Mdir build/unit/camera_stream_adapter \
//     rtl/integration/camera_stream_adapter.sv rtl/image_processing/red_mask.sv \
//     rtl/perception/led_centroid.sv sim/unit/camera_stream_adapter_tb.sv
//   ./build/unit/camera_stream_adapter/Vcamera_stream_adapter_tb

module camera_stream_adapter_tb;

    localparam int W = 8;
    localparam int H = 4;
    localparam int PW = 24;

    localparam logic [15:0] RED   = 16'hF800;
    localparam logic [15:0] GREY  = 16'h8410;

    // ------------------------------------------------------------------
    // Clock, reset, DUT
    // ------------------------------------------------------------------

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic        rst_n = 1'b0;
    logic        cam_frame_start = 1'b0;
    logic        cam_valid = 1'b0;
    logic [15:0] cam_pixel = 16'd0;
    logic [9:0]  cam_x = 10'd0;
    logic [8:0]  cam_y = 9'd0;

    wire        frame_start, frame_end, pixel_valid, frame_good, stream_ok;
    wire [15:0] pixel_rgb565;
    wire [9:0]  pixel_x, pixel_y;
    wire [1:0]  last_error;
    wire [15:0] good_frames, bad_frames, dropped_pixels;
    wire [PW-1:0] frame_period;

    camera_stream_adapter #(
        .W(W), .H(H), .PERIOD_W(PW)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .cam_frame_start(cam_frame_start),
        .cam_valid(cam_valid),
        .cam_pixel(cam_pixel),
        .cam_x(cam_x),
        .cam_y(cam_y),
        .frame_start(frame_start),
        .frame_end(frame_end),
        .pixel_valid(pixel_valid),
        .pixel_rgb565(pixel_rgb565),
        .pixel_x(pixel_x),
        .pixel_y(pixel_y),
        .frame_good(frame_good),
        .stream_ok(stream_ok),
        .last_error(last_error),
        .good_frames(good_frames),
        .bad_frames(bad_frames),
        .dropped_pixels(dropped_pixels),
        .frame_period(frame_period)
    );

    // Downstream, wired as in major_project_top.
    wire       mask_valid, is_red;
    wire [9:0] mask_x, mask_y;

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

    wire        c_done, c_valid;
    wire [9:0]  c_x, c_y;
    wire [31:0] c_count;

    led_centroid #(
        .MIN_PIXELS(32'd2)
    ) u_centroid (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start(frame_start),
        .frame_end(frame_end),
        .pixel_valid(mask_valid),
        .is_red(is_red),
        .pixel_x(mask_x),
        .pixel_y(mask_y),
        .frame_done(c_done),
        .target_valid(c_valid),
        .target_x(c_x),
        .target_y(c_y),
        .red_count(c_count)
    );

    // ------------------------------------------------------------------
    // Reference model: predicts the DUT's registered outputs.
    // Updated on each rising edge from the inputs the DUT sampled.
    // ------------------------------------------------------------------

    bit  m_fs, m_fe, m_pv, m_good, m_ok;
    int  m_px, m_py, m_pix;
    int  m_err;                 // 0 none, 1 coordinate, 2 truncated
    int  m_good_n, m_bad_n, m_drop;
    int  m_period, m_pcnt;
    bit  m_synced, m_open;
    int  m_ex, m_ey;

    localparam int PMAX = (1 << PW) - 1;

    task automatic model_reset();
        m_fs = 0; m_fe = 0; m_pv = 0; m_good = 0; m_ok = 0;
        m_px = 0; m_py = 0; m_pix = 0; m_err = 0;
        m_good_n = 0; m_bad_n = 0; m_drop = 0;
        m_period = 0; m_pcnt = 0;
        m_synced = 0; m_open = 0; m_ex = 0; m_ey = 0;
    endtask

    function automatic void drop_one();
        if (m_drop < 65535) m_drop++;
    endfunction

    task automatic model_step(
        input bit s, input bit v, input int pix, input int x, input int y
    );
        m_fs = 0; m_fe = 0; m_pv = 0; m_good = 0;

        if (s) begin
            if (m_synced) m_period = m_pcnt;
            m_pcnt = 1;
            m_fs = 1;
            if (m_open) begin
                m_fe = 1;
                m_bad_n = (m_bad_n + 1) % 65536;
                m_err = 2;
                m_ok = 0;
            end
            if (v) drop_one();
            m_synced = 1;
            m_open = 1;
            m_ex = 0;
            m_ey = 0;
        end else begin
            if (m_pcnt < PMAX) m_pcnt++;
            if (v) begin
                if (!m_open) begin
                    drop_one();
                end else if (x != m_ex || y != m_ey) begin
                    m_fs = 1; m_fe = 1;
                    m_bad_n = (m_bad_n + 1) % 65536;
                    m_err = 1;
                    m_ok = 0;
                    m_open = 0;
                    drop_one();
                end else begin
                    m_pv = 1; m_pix = pix; m_px = x; m_py = y;
                    if (x == W - 1 && y == H - 1) begin
                        m_fe = 1; m_good = 1;
                        m_good_n = (m_good_n + 1) % 65536;
                        m_err = 0;
                        m_ok = 1;
                        m_open = 0;
                    end
                    if (m_ex == W - 1) begin m_ex = 0; m_ey++; end
                    else m_ex++;
                end
            end
        end
    endtask

    // Note: m_pcnt increments before a start is seen in the RTL too; the
    // RTL increments every cycle and overwrites with 1 on a start, which is
    // what the model does.

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            model_reset();
        else
            model_step(cam_frame_start, cam_valid, int'(cam_pixel),
                       int'(cam_x), int'(cam_y));
    end

    // ------------------------------------------------------------------
    // Cycle-by-cycle comparison (on the falling edge, after the update)
    // ------------------------------------------------------------------

    int model_checks = 0;
    int checks = 0;

    always @(negedge clk) begin
        if (rst_n) begin
            if (frame_start !== m_fs || frame_end !== m_fe ||
                pixel_valid !== m_pv || frame_good !== m_good)
                $fatal(1, "FAIL model: strobes fs/fe/pv/good DUT=%b%b%b%b model=%b%b%b%b at %0t",
                       frame_start, frame_end, pixel_valid, frame_good,
                       m_fs, m_fe, m_pv, m_good, $time);
            if (m_pv && (int'(pixel_rgb565) != m_pix ||
                         int'(pixel_x) != m_px || int'(pixel_y) != m_py))
                $fatal(1, "FAIL model: pixel DUT=(%0d,%0d)%04h model=(%0d,%0d)%04h",
                       pixel_x, pixel_y, pixel_rgb565, m_px, m_py, m_pix);
            if (int'(good_frames) != m_good_n || int'(bad_frames) != m_bad_n ||
                int'(dropped_pixels) != m_drop)
                $fatal(1, "FAIL model: counters good/bad/drop DUT=%0d/%0d/%0d model=%0d/%0d/%0d",
                       good_frames, bad_frames, dropped_pixels, m_good_n, m_bad_n, m_drop);
            if (int'(last_error) != m_err || stream_ok !== m_ok)
                $fatal(1, "FAIL model: last_error/stream_ok DUT=%0d/%b model=%0d/%b",
                       last_error, stream_ok, m_err, m_ok);
            if (int'(frame_period) != m_period)
                $fatal(1, "FAIL model: frame_period DUT=%0d model=%0d",
                       frame_period, m_period);
            model_checks++;
        end
    end

    // ------------------------------------------------------------------
    // Downstream monitor: every closed frame's centroid result
    // ------------------------------------------------------------------

    int  out_red_n = 0, out_red_sx = 0, out_red_sy = 0;
    bit  pend = 0;
    bit  pend_good;
    int  pend_n, pend_sx, pend_sy;
    int  results = 0, good_results = 0, rejected_results = 0;

    always @(negedge clk) begin
        if (!rst_n) begin
            out_red_n = 0; out_red_sx = 0; out_red_sy = 0; pend = 0;
        end else begin
            if (frame_start) begin
                out_red_n = 0; out_red_sx = 0; out_red_sy = 0;
            end
            if (pixel_valid && pixel_rgb565 == RED) begin
                out_red_n++;
                out_red_sx += int'(pixel_x);
                out_red_sy += int'(pixel_y);
            end
            if (frame_end) begin
                if (pend)
                    $fatal(1, "FAIL downstream: frame_end while centroid busy");
                pend = 1;
                pend_good = frame_good;
                pend_n = out_red_n; pend_sx = out_red_sx; pend_sy = out_red_sy;
            end
            if (c_done) begin
                if (!pend)
                    $fatal(1, "FAIL downstream: frame_done without frame_end");
                pend = 0;
                results++;
                if (!pend_good) begin
                    rejected_results++;
                    if (c_valid !== 1'b0)
                        $fatal(1, "FAIL downstream: rejected frame produced a target at (%0d,%0d)",
                               c_x, c_y);
                end else begin
                    good_results++;
                    if (pend_n < 2) $fatal(1, "FAIL bench: good frame without red pixels");
                    if (c_valid !== 1'b1 ||
                        int'(c_x) != pend_sx / pend_n ||
                        int'(c_y) != pend_sy / pend_n)
                        $fatal(1, "FAIL downstream: centroid valid=%b (%0d,%0d) expected (%0d,%0d)",
                               c_valid, c_x, c_y, pend_sx / pend_n, pend_sy / pend_n);
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Stimulus helpers (drive on the falling edge)
    // ------------------------------------------------------------------

    task automatic idle(input int n);
        repeat (n) begin
            @(negedge clk);
            cam_frame_start = 0;
            cam_valid = 0;
        end
    endtask

    task automatic start();
        @(negedge clk);
        cam_frame_start = 1;
        cam_valid = 0;
        @(negedge clk);
        cam_frame_start = 0;
    endtask

    // Red pixels at x = 2..3 on every row: centroid x = 2, y = mean row.
    function automatic logic [15:0] colour(input int x, input int y);
        return (x == 2 || x == 3) ? RED : (GREY ^ 16'(y * 16 + x));
    endfunction

    task automatic pixel(input int x, input int y);
        @(negedge clk);
        cam_frame_start = 0;
        cam_valid = 1;
        cam_pixel = colour(x, y);
        cam_x = 10'(x);
        cam_y = 9'(y);
        @(negedge clk);         // second byte: no pixel
        cam_valid = 0;
    endtask

    task automatic row(input int y, input int len);
        for (int x = 0; x < len; x++) pixel(x, y);
        idle(6);                // horizontal blanking
    endtask

    task automatic rows(input int y0, input int y1);
        for (int y = y0; y <= y1; y++) row(y, W);
    endtask

    // A complete clean frame: start, blanking, H rows, blanking.
    task automatic clean_frame();
        start();
        idle(20);
        rows(0, H - 1);
        idle(40);
    endtask

    task automatic wait_result();
        idle(45);               // led_centroid takes 32 cycles to divide
    endtask

    task automatic pass(input string name);
        checks++;
        $display("PASS [%0d]: %s", checks, name);
    endtask

    task automatic expect_state(
        input string name, input int good_n, input int bad_n,
        input int err, input bit ok
    );
        @(negedge clk);
        if (int'(good_frames) != good_n || int'(bad_frames) != bad_n ||
            int'(last_error) != err || stream_ok !== ok)
            $fatal(1, "FAIL %s: good=%0d bad=%0d err=%0d ok=%b, expected %0d/%0d/%0d/%b",
                   name, good_frames, bad_frames, last_error, stream_ok,
                   good_n, bad_n, err, ok);
    endtask

    // ------------------------------------------------------------------
    // Scenarios
    // ------------------------------------------------------------------

    int drop_before, res_before;

    initial begin
        if ($test$plusargs("wave")) begin
            $dumpfile("camera_stream_adapter.fst");
            $dumpvars(0, camera_stream_adapter_tb);
        end

        idle(5);
        rst_n = 1;
        idle(3);

        // 1. Pixels before the first frame_start are not processed.
        row(1, 5);
        if (dropped_pixels != 16'd5 || good_frames != 0 || bad_frames != 0)
            $fatal(1, "FAIL unsynchronised: dropped=%0d", dropped_pixels);
        pass("pixels before first frame_start are dropped (5 of 5)");

        // 2. Clean frame: W x H pixels, frame_end + frame_good on (W-1,H-1).
        res_before = results;
        clean_frame();
        wait_result();
        expect_state("clean frame", 1, 0, 0, 1);
        if (results != res_before + 1 || c_valid !== 1'b0 /* pulse gone */ ||
            c_x != 10'd2 || c_y != 10'd1)
            $fatal(1, "FAIL clean frame: centroid (%0d,%0d)", c_x, c_y);
        pass("clean frame accepted; centroid (2,1) downstream");

        // 3. Second clean frame and frame period.
        @(negedge clk);
        clean_frame();
        wait_result();
        expect_state("second clean frame", 2, 0, 0, 1);
        pass("back-to-back clean frame accepted");

        // Frame period: measured between two frame_starts sent 1000 cycles apart.
        start();
        idle(998);
        start();
        if (frame_period != PW'(1000))
            $fatal(1, "FAIL frame period: %0d, expected 1000", frame_period);
        pass("frame_period measures 1000 PCLKs between frame_starts");
        // That frame_start closed the (empty) frame opened 1000 cycles earlier.
        wait_result();
        expect_state("empty frame between starts", 2, 1, 2, 0);
        pass("frame with no pixels closed as TRUNCATED");

        // 4. Truncated frame: VSYNC arrives after 2 of 4 rows.
        //    (the frame opened by the last start above)
        rows(0, 1);
        idle(10);
        res_before = results;
        start();                // closes the truncated frame, opens a new one
        wait_result();
        expect_state("truncated frame", 2, 2, 2, 0);
        if (results != res_before + 1)
            $fatal(1, "FAIL truncated: no downstream result");
        pass("truncated frame closed at next frame_start; target_valid=0 downstream");

        // The frame that start opened is completed normally.
        idle(10);
        rows(0, H - 1);
        idle(40);
        wait_result();
        expect_state("recovery after truncation", 3, 2, 0, 1);
        pass("next frame accepted after truncation");

        // 5. Short row: row 1 has W-2 pixels.
        start();
        idle(20);
        row(0, W);
        row(1, W - 2);
        drop_before = int'(dropped_pixels);
        rows(2, H - 1);
        idle(40);
        wait_result();
        expect_state("short row", 3, 3, 1, 0);
        if (dropped_pixels != 16'(drop_before + 2 * W))
            $fatal(1, "FAIL short row: dropped %0d, expected %0d",
                   int'(dropped_pixels) - drop_before, 2 * W);
        pass("short row rejected (COORDINATE); remaining rows dropped; target_valid=0");

        // 6. Long row: row 2 has W+1 pixels (x = W).
        start();
        idle(20);
        rows(0, 1);
        row(2, W + 1);
        row(3, W);
        idle(40);
        wait_result();
        expect_state("long row", 3, 4, 1, 0);
        pass("long row (x = W) rejected (COORDINATE)");

        // 7. Missing row: row 2 skipped.
        start();
        idle(20);
        rows(0, 1);
        row(3, W);
        idle(40);
        wait_result();
        expect_state("missing row", 3, 5, 1, 0);
        pass("missing row rejected (COORDINATE)");

        // 8. Extra pixels after a complete frame are dropped.
        start();
        idle(20);
        rows(0, H - 1);
        drop_before = int'(dropped_pixels);
        res_before = results;
        row(0, 3);
        idle(40);
        wait_result();
        expect_state("extra pixels", 4, 5, 0, 1);
        if (dropped_pixels != 16'(drop_before + 3) || results != res_before + 1)
            $fatal(1, "FAIL extra pixels: dropped %0d results %0d",
                   int'(dropped_pixels) - drop_before, results - res_before);
        pass("extra pixels after a complete frame dropped; one result only");

        // 9. Reset in the middle of a frame.
        start();
        idle(20);
        rows(0, 1);
        @(negedge clk) rst_n = 0;
        idle(3);
        @(negedge clk) rst_n = 1;
        rows(2, H - 1);         // tail of the interrupted frame
        idle(10);
        if (good_frames != 0 || bad_frames != 0 || dropped_pixels != 16'(2 * W) ||
            stream_ok !== 1'b0)
            $fatal(1, "FAIL reset mid-frame: good=%0d bad=%0d dropped=%0d",
                   good_frames, bad_frames, dropped_pixels);
        pass("reset mid-frame: counters cleared, frame tail dropped");
        clean_frame();
        wait_result();
        expect_state("clean frame after reset", 1, 0, 0, 1);
        pass("clean frame accepted after reset");

        // 10. Several frames: random defects against the reference model.
        for (int i = 0; i < 40; i++) begin
            int kind = $urandom_range(0, 4);
            start();
            idle($urandom_range(2, 30));
            for (int y = 0; y < H; y++) begin
                int len = W;
                if (kind == 1 && y == 1) len = $urandom_range(1, W - 1);
                if (kind == 2 && y == 2) len = W + 1;
                if (kind == 3 && y == 2) continue;
                if (kind == 4 && y == 2) break;
                row(y, len);
            end
            idle($urandom_range(40, 80));
        end
        wait_result();
        pass("40 frames with random defects match the reference model");

        if (rejected_results == 0 || good_results == 0)
            $fatal(1, "FAIL bench: coverage good=%0d rejected=%0d",
                   good_results, rejected_results);
        pass($sformatf("downstream: %0d good frames gave the right centroid, %0d rejected frames gave no target",
                       good_results, rejected_results));

        $display("-----------------------------------------------");
        $display("PASS: camera_stream_adapter_tb (%0d scenario checks, %0d cycle checks)",
                 checks, model_checks);
        $display("-----------------------------------------------");
        $finish;
    end

    initial begin
        #5ms;
        $fatal(1, "FAIL: timeout");
    end

endmodule

`default_nettype wire
