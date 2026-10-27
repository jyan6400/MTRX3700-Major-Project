`timescale 1ns/1ps
`default_nettype none

module red_vision_pipeline_tb;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic rst_n = 1'b0;

    logic frame_start = 1'b0;
    logic frame_end = 1'b0;
    logic pixel_valid = 1'b0;

    logic [15:0] pixel_rgb565 = 16'd0;
    logic [9:0] pixel_x = 10'd0;
    logic [9:0] pixel_y = 10'd0;

    logic mask_valid;
    logic is_red;

    wire [9:0] mask_x;
    wire [9:0] mask_y;

    logic frame_done;
    logic target_valid;
    logic [9:0] target_x;
    logic [9:0] target_y;
    logic [31:0] red_count;

    int checks = 0;

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

    led_centroid #(
        .MIN_PIXELS(32'd3)
    ) u_centroid (
        .clk(clk),
        .rst_n(rst_n),
        .gate_bypass(1'b1), // Legacy V1 acceptance regression

        .frame_start(frame_start),
        .frame_end(frame_end),

        .pixel_valid(mask_valid),
        .is_red(is_red),
        .pixel_x(mask_x),
        .pixel_y(mask_y),

        .frame_done(frame_done),
        .target_valid(target_valid),
        .target_x(target_x),
        .target_y(target_y),
        .red_count(red_count)
    );

    task automatic send_pixel(
        input bit start_of_frame,
        input bit end_of_frame,
        input bit valid,
        input logic [15:0] rgb,
        input logic [9:0] x,
        input logic [9:0] y,
        input bit expected_red
    );

        @(negedge clk);

        frame_start  = start_of_frame;
        frame_end    = end_of_frame;
        pixel_valid  = valid;
        pixel_rgb565 = rgb;
        pixel_x      = x;
        pixel_y      = y;

        #1;

        if (is_red !== (valid && expected_red))
            $fatal(1,
                "Mask failure RGB=%h x=%0d y=%0d expected=%b actual=%b",
                rgb, x, y, (valid && expected_red), is_red
            );

        if (mask_valid !== valid ||
            mask_x !== x ||
            mask_y !== y)
            $fatal(1, "Mask stream pass-through failure");

        @(posedge clk);
        #1;
    endtask

    task automatic check_frame(
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

        // The iterative centroid calculation takes 32 PCLK cycles.
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
                "FAIL %s: frame_done missing",
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

    initial begin

        $dumpfile("red_vision_pipeline.vcd");
        $dumpvars(0, red_vision_pipeline_tb);

        repeat (3) @(negedge clk);
        rst_n = 1'b1;

        // Frame 1: red pixels mixed with non-red pixels.
        // Expected centroid: (110,210).

        send_pixel(1, 0, 1, 16'hF800, 100, 200, 1);
        send_pixel(0, 0, 1, 16'h07E0, 300, 100, 0);
        send_pixel(0, 0, 1, 16'hF800, 110, 210, 1);
        send_pixel(0, 0, 1, 16'h001F, 400, 300, 0);
        send_pixel(0, 1, 1, 16'hF800, 120, 220, 1);

        check_frame(
            "RGB565 stream to centroid",
            1, 110, 210, 3
        );

        // Frame 2: LED disappears.

        send_pixel(1, 0, 1, 16'h0000, 50, 50, 0);
        send_pixel(0, 0, 1, 16'hFFFF, 60, 60, 0);
        send_pixel(0, 1, 1, 16'h07E0, 70, 70, 0);

        check_frame(
            "LED disappearance",
            0, 0, 0, 0
        );

        // Frame 3: one red pixel is insufficient.

        send_pixel(1, 0, 1, 16'hF800, 500, 400, 1);
        send_pixel(0, 1, 1, 16'h0000, 100, 100, 0);

        check_frame(
            "isolated red noise rejected",
            0, 0, 0, 1
        );

        // Frame 4: invalid red pixel must not contribute.

        send_pixel(1, 0, 0, 16'hF800, 600, 400, 1);
        send_pixel(0, 0, 1, 16'hF800, 200, 100, 1);
        send_pixel(0, 0, 1, 16'hF800, 210, 110, 1);
        send_pixel(0, 1, 1, 16'hF800, 220, 120, 1);

        check_frame(
            "invalid pixel ignored",
            1, 210, 110, 3
        );

        // Frame 5: target at the image boundary.

        send_pixel(1, 0, 1, 16'hF800, 639, 479, 1);
        send_pixel(0, 0, 1, 16'hF800, 639, 479, 1);
        send_pixel(0, 1, 1, 16'hF800, 639, 479, 1);

        check_frame(
            "image boundary",
            1, 639, 479, 3
        );

        // Frame 6: empty frame.

        send_pixel(1, 1, 0, 16'h0000, 0, 0, 0);

        check_frame(
            "empty frame",
            0, 0, 0, 0
        );

        // Test 7: ensure pulses clear next cycle.

        send_pixel(0, 0, 0, 16'h0000, 0, 0, 0);

        if (frame_done !== 1'b0 ||
            target_valid !== 1'b0)
            $fatal(1,
                "FAIL: status pulses did not clear"
            );

        checks++;
        $display("PASS [%0d]: frame_done and target_valid pulses clear", checks);

        $display("-----------------------------------------");
        $display("PASS: red_vision_pipeline_tb (%0d checks)", checks);
        $display("-----------------------------------------");

        $finish;
    end

endmodule

`default_nettype wire
