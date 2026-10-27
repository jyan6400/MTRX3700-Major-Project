`timescale 1ns/1ps
`default_nettype none

module led_centroid_tb;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic rst_n = 1'b0;

    logic frame_start = 1'b0;
    logic frame_end = 1'b0;
    logic pixel_valid = 1'b0;
    logic is_red = 1'b0;

    logic [9:0] pixel_x = 10'd0;
    logic [9:0] pixel_y = 10'd0;

    logic frame_done;
    logic target_valid;
    logic [9:0] target_x;
    logic [9:0] target_y;
    logic [31:0] red_count;
    logic [7:0]  confidence;
    logic [3:0]  reject_flags;
    logic [9:0]  bbox_x_min, bbox_x_max, bbox_y_min, bbox_y_max;

    // Part A (V1 regression) runs with the gates bypassed; Part B tests the V2 gates.
    logic gate_bypass = 1'b1;

    int checks = 0;

    // Use three pixels to make the unit tests compact.
    // Production configuration defaults to 20 pixels.
    // MAX_PIXELS is also scaled down so the "too large" case stays short.

    led_centroid #(
        .MIN_PIXELS(32'd3),
        .MAX_PIXELS(32'd200),
        .MAX_ASPECT(4),
        .MIN_CONFIDENCE(8'd64)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .gate_bypass(gate_bypass),

        .frame_start(frame_start),
        .frame_end(frame_end),

        .pixel_valid(pixel_valid),
        .is_red(is_red),
        .pixel_x(pixel_x),
        .pixel_y(pixel_y),

        .frame_done(frame_done),
        .target_valid(target_valid),
        .target_x(target_x),
        .target_y(target_y),
        .red_count(red_count),
        .confidence(confidence),
        .reject_flags(reject_flags),
        .bbox_x_min(bbox_x_min),
        .bbox_x_max(bbox_x_max),
        .bbox_y_min(bbox_y_min),
        .bbox_y_max(bbox_y_max)
    );

    task automatic send_pixel(
        input bit start,
        input bit finish,
        input bit valid,
        input bit red,
        input logic [9:0] x,
        input logic [9:0] y
    );
        @(negedge clk);

        frame_start = start;
        frame_end   = finish;
        pixel_valid = valid;
        is_red      = red;
        pixel_x     = x;
        pixel_y     = y;

        @(posedge clk);
        #1;
    endtask

    task automatic check_result(
        input string name,
        input bit expected_valid,
        input logic [9:0] expected_x,
        input logic [9:0] expected_y,
        input logic [31:0] expected_count
    );

        // frame_end was asserted for one clock.
        // Release frame inputs before awaiting the sequential result.
        @(negedge clk);
        frame_start = 1'b0;
        frame_end = 1'b0;
        pixel_valid = 1'b0;

        // The post-frame pipeline takes 34 PCLK cycles.
        begin : wait_for_centroid
            bit completed;
            completed = 1'b0;
            for (int n = 0; n < 40; n++) begin
                @(posedge clk);
                #1;
                if (frame_done) begin
                    completed = 1'b1;
                    break;
                end
            end
            if (!completed)
                $fatal(1, "Timeout waiting for centroid frame_done");
        end

        if (frame_done !== 1'b1)
            $fatal(1,
                "FAIL %s: frame_done must pulse at frame completion",
                name
            );

        if (target_valid !== expected_valid ||
            target_x !== expected_x ||
            target_y !== expected_y ||
            red_count !== expected_count)
            $fatal(1,
                "FAIL %s: valid=%b x=%0d y=%0d count=%0d; expected valid=%b x=%0d y=%0d count=%0d",
                name,
                target_valid, target_x, target_y, red_count,
                expected_valid, expected_x, expected_y, expected_count
            );

        checks++;
        $display("PASS [%0d]: %s", checks, name);
    endtask

    task automatic check_pulses_clear(
        input string name
    );
        send_pixel(0, 0, 0, 0, 0, 0);

        if (frame_done !== 1'b0 ||
            target_valid !== 1'b0)
            $fatal(1,
                "FAIL %s: frame_done=%b target_valid=%b",
                name, frame_done, target_valid
            );

        checks++;
        $display("PASS [%0d]: %s", checks, name);
    endtask

    // ---------------- V2 helpers ----------------

    task automatic begin_frame();
        send_pixel(1, 0, 0, 0, 0, 0);
    endtask

    // Red pixels over a w x h box at (x0, y0); step 2 marks every second column.
    task automatic send_rect(
        input int x0, input int y0, input int w, input int h, input int step
    );
        for (int y = y0; y < y0 + h; y++)
            for (int x = x0; x < x0 + w; x += step)
                send_pixel(0, 0, 1, 1, x[9:0], y[9:0]);
    endtask

    // Ends the frame and checks every V2 output. Also checks the 34-cycle latency.
    task automatic end_frame_check(
        input string name,
        input bit expected_valid,
        input int expected_x, input int expected_y,
        input int expected_count,
        input int expected_conf,
        input logic [3:0] expected_flags
    );
        int latency;
        send_pixel(0, 1, 0, 0, 0, 0);   // frame_end, sampled on this edge
        latency = 0;
        @(negedge clk);
        frame_end = 1'b0;
        begin : wait_v2
            for (int n = 1; n <= 40; n++) begin
                @(posedge clk);
                #1;
                if (frame_done) begin
                    latency = n;
                    disable wait_v2;
                end
            end
            $fatal(1, "FAIL %s: timeout waiting for frame_done", name);
        end

        if (latency != 34)
            $fatal(1, "FAIL %s: latency %0d cycles, expected 34", name, latency);

        if (target_valid !== expected_valid ||
            target_x !== expected_x[9:0] ||
            target_y !== expected_y[9:0] ||
            red_count !== expected_count ||
            confidence !== expected_conf[7:0] ||
            reject_flags !== expected_flags)
            $fatal(1,
                "FAIL %s: valid=%b x=%0d y=%0d n=%0d conf=%0d flags=%b; expected valid=%b x=%0d y=%0d n=%0d conf=%0d flags=%b",
                name, target_valid, target_x, target_y, red_count, confidence, reject_flags,
                expected_valid, expected_x, expected_y, expected_count, expected_conf, expected_flags);

        checks++;
        $display("PASS [%0d]: %s (conf=%0d, flags=%b)", checks, name, confidence, reject_flags);
    endtask

    task automatic check_bbox(
        input string name,
        input int x_min, input int x_max, input int y_min, input int y_max
    );
        if (bbox_x_min !== x_min[9:0] || bbox_x_max !== x_max[9:0] ||
            bbox_y_min !== y_min[9:0] || bbox_y_max !== y_max[9:0])
            $fatal(1, "FAIL %s: bbox x %0d..%0d y %0d..%0d; expected x %0d..%0d y %0d..%0d",
                name, bbox_x_min, bbox_x_max, bbox_y_min, bbox_y_max, x_min, x_max, y_min, y_max);
        checks++;
        $display("PASS [%0d]: %s", checks, name);
    endtask

    initial begin

        $dumpfile("led_centroid.vcd");
        $dumpvars(0, led_centroid_tb);

        repeat (3) @(negedge clk);
        rst_n = 1'b1;

        // ================= Part A: V1 regression (gate_bypass = 1) =================

        // Test 1: centroid calculation.
        send_pixel(1, 0, 1, 1, 10, 20);
        send_pixel(0, 0, 1, 1, 20, 30);
        send_pixel(0, 1, 1, 1, 30, 40);

        check_result(
            "three-pixel centroid",
            1, 20, 30, 3
        );

        // Test 2: frame_done and target_valid are pulses.
        check_pulses_clear("valid-frame pulses clear");

        // Test 3: reject a frame with two pixels.
        send_pixel(1, 0, 1, 1, 100, 100);
        send_pixel(0, 1, 1, 1, 200, 200);

        check_result(
            "two pixels rejected",
            0, 0, 0, 2
        );

        // Test 4: frame_done still pulses for rejected frames.
        check_pulses_clear("rejected-frame pulse clears");

        // Test 5: invalid and non-red pixels ignored.
        send_pixel(1, 0, 1, 0, 400, 400);
        send_pixel(0, 0, 0, 1, 400, 400);
        send_pixel(0, 1, 1, 0, 400, 400);

        check_result(
            "zero red pixels",
            0, 0, 0, 0
        );

        // Test 6: integer division rounds down.
        send_pixel(1, 0, 1, 1, 10, 20);
        send_pixel(0, 0, 1, 1, 11, 21);
        send_pixel(0, 1, 1, 1, 12, 21);

        check_result(
            "integer centroid rounding",
            1, 11, 20, 3
        );

        // Test 7: coordinates at image boundary.
        send_pixel(1, 0, 1, 1, 639, 479);
        send_pixel(0, 0, 1, 1, 639, 479);
        send_pixel(0, 1, 1, 1, 639, 479);

        check_result(
            "image boundary",
            1, 639, 479, 3
        );

        // Test 8: reset must clear all outputs.
        send_pixel(1, 0, 1, 1, 500, 400);
        send_pixel(0, 0, 1, 1, 500, 400);

        @(negedge clk);
        rst_n = 1'b0;
        #1;

        if (frame_done !== 1'b0 ||
            target_valid !== 1'b0 ||
            target_x !== 10'd0 ||
            target_y !== 10'd0 ||
            red_count !== 32'd0 ||
            confidence !== 8'd0 ||
            reject_flags !== 4'd0 ||
            bbox_x_max !== 10'd0)
            $fatal(1, "FAIL: reset did not clear outputs");

        checks++;
        $display("PASS [%0d]: reset clears outputs", checks);

        frame_start = 0;
        frame_end = 0;
        pixel_valid = 0;
        is_red = 0;

        @(negedge clk);
        rst_n = 1'b1;

        // Test 9: old partial pixels must not contribute.
        send_pixel(1, 0, 1, 1, 25, 35);
        send_pixel(0, 0, 1, 1, 25, 35);
        send_pixel(0, 1, 1, 1, 25, 35);

        check_result(
            "reset discards partial frame",
            1, 25, 35, 3
        );

        // Test 10: empty frame after a valid frame.
        send_pixel(1, 1, 0, 0, 0, 0);

        check_result(
            "empty frame clears target",
            0, 0, 0, 0
        );

        // Test 11: empty-frame pulse clears.
        check_pulses_clear("empty-frame pulse clears");


        // ================= Part B: V2 gates (gate_bypass = 0) =================
        gate_bypass = 1'b0;

        // B1: compact 6x6 LED blob is accepted; fill 36/36 saturates confidence.
        // Centroid x = 615/6 = 102, y = 1215/6 = 202.
        begin_frame();
        send_rect(100, 200, 6, 6, 1);
        end_frame_check("V2 compact blob accepted", 1, 102, 202, 36, 255, 4'b0000);
        check_bbox("V2 bounding box of blob", 100, 105, 200, 205);

        // B2: every second column of an 8x4 area: pixels at x = 300..306, so the box is
        // 7x4 = 28 and confidence = 256*16/28 = 146. Sparser than B1 but still accepted.
        begin_frame();
        send_rect(300, 100, 8, 4, 2);
        end_frame_check("V2 partial fill confidence", 1, 303, 101, 16, 146, 4'b0000);

        // B3: LED plus a distant red distractor. The box spans both blobs:
        // 72 / (206*206) -> conf 0, frame rejected instead of steering to the midpoint.
        begin_frame();
        send_rect(100, 200, 6, 6, 1);
        send_rect(300, 400, 6, 6, 1);
        end_frame_check("V2 distant distractor rejected", 0, 0, 0, 72, 0, 4'b1000);

        // B4: the same distractor frame with the bypass on behaves like V1:
        // accepted, and the centroid lands between the two objects (202, 302).
        gate_bypass = 1'b1;
        begin_frame();
        send_rect(100, 200, 6, 6, 1);
        send_rect(300, 400, 6, 6, 1);
        end_frame_check("bypass reproduces V1 midpoint error", 1, 202, 302, 72, 0, 4'b1000);
        gate_bypass = 1'b0;

        // B5: thin 40x2 stripe (e.g. reflection) fails the 4:1 aspect gate.
        begin_frame();
        send_rect(50, 50, 40, 2, 1);
        end_frame_check("V2 stripe rejected by aspect", 0, 0, 0, 80, 255, 4'b0100);

        // B6: 15x15 = 225 pixels exceeds MAX_PIXELS (200): large red background.
        begin_frame();
        send_rect(200, 200, 15, 15, 1);
        end_frame_check("V2 oversized region rejected", 0, 0, 0, 225, 255, 4'b0010);

        // B7: two pixels: too few (same rule as V1), plus low fill across 101x101.
        begin_frame();
        send_pixel(0, 0, 1, 1, 100, 100);
        send_pixel(0, 0, 1, 1, 200, 200);
        end_frame_check("V2 too few pixels rejected", 0, 0, 0, 2, 0, 4'b1001);

        // B8: empty frame reports zero box and zero confidence.
        begin_frame();
        end_frame_check("V2 empty frame", 0, 0, 0, 0, 0, 4'b1001);
        check_bbox("V2 empty frame bbox cleared", 0, 0, 0, 0);

        // B9: a valid frame after rejected ones recovers immediately (no stale state).
        begin_frame();
        send_rect(400, 300, 5, 5, 1);
        end_frame_check("V2 recovery after rejection", 1, 402, 302, 25, 255, 4'b0000);
        check_pulses_clear("V2 pulses clear");

        $display("---------------------------------------");
        $display("PASS: led_centroid_tb (%0d checks)", checks);
        $display("---------------------------------------");

        $finish;
    end

endmodule

`default_nettype wire
