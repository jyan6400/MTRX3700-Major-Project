`timescale 1ns/1ps
`default_nettype none

module steering_controller_tb;

    logic drive_enable = 1'b0;
    logic target_available = 1'b0;
    logic [9:0] target_x = 10'd320;

    logic signed [15:0] left_pwm;
    logic signed [15:0] right_pwm;

    logic signed [15:0] inverted_left;
    logic signed [15:0] inverted_right;

    int checks = 0;

    steering_controller dut (
        .drive_enable(drive_enable),
        .target_available(target_available),
        .target_x(target_x),
        .left_pwm(left_pwm),
        .right_pwm(right_pwm)
    );

    // Second instance tests configurable steering polarity.
    steering_controller #(
        .INVERT_STEERING(1'b1)
    ) dut_inverted (
        .drive_enable(drive_enable),
        .target_available(target_available),
        .target_x(target_x),
        .left_pwm(inverted_left),
        .right_pwm(inverted_right)
    );

    task automatic check_drive(
        input string name,
        input bit enable,
        input bit target_present,
        input logic [9:0] x,
        input int signed expected_left,
        input int signed expected_right
    );

        drive_enable = enable;
        target_available = target_present;
        target_x = x;

        #10;

        if (int'(left_pwm) != expected_left ||
            int'(right_pwm) != expected_right) begin
            $fatal(1,
                "FAIL %s: x=%0d enable=%b target=%b left=%0d right=%0d expected=%0d/%0d",
                name, x, enable, target_present,
                left_pwm, right_pwm,
                expected_left, expected_right
            );
        end

        checks++;
        $display("PASS [%0d]: %s", checks, name);
    endtask

    task automatic check_inverted(
        input string name,
        input int signed expected_left,
        input int signed expected_right
    );

        #1;

        if (int'(inverted_left) != expected_left ||
            int'(inverted_right) != expected_right) begin
            $fatal(1,
                "FAIL %s: inverted left=%0d right=%0d expected=%0d/%0d",
                name, inverted_left, inverted_right,
                expected_left, expected_right
            );
        end

        checks++;
        $display("PASS [%0d]: %s", checks, name);
    endtask

    initial begin
        $dumpfile("steering_controller.vcd");
        $dumpvars(0, steering_controller_tb);

        // Disabled and invalid-target conditions
        check_drive("disabled centred", 0, 1, 320, 0, 0);
        check_drive("target missing", 1, 0, 400, 0, 0);
        check_drive("disabled off-centre", 0, 1, 100, 0, 0);

        // Centre and deadband
        check_drive("target centred", 1, 1, 320, 80, 80);
        check_drive("left deadband edge", 1, 1, 300, 80, 80);
        check_drive("right deadband edge", 1, 1, 340, 80, 80);

        // Outside deadband, integer gain calculation
        check_drive("just outside right deadband",
                    1, 1, 341, 70, 60);
        check_drive("just outside left deadband",
                    1, 1, 299, 60, 70);

        // Normal proportional corrections
        check_drive("target right", 1, 1, 400, 85, 45);
        check_drive("target left", 1, 1, 240, 45, 85);

        // Steering polarity inversion
        check_inverted("inverted target left", 85, 45);

        check_drive("target right for inverse test",
                    1, 1, 400, 85, 45);
        check_inverted("inverted target right", 45, 85);

        // V2 speed-scheduling boundary tests
        check_drive("V2 error 20", 1, 1, 340, 80, 80);
        check_drive("V2 error 21", 1, 1, 341, 70, 60);
        check_drive("V2 error 80", 1, 1, 400, 85, 45);
        check_drive("V2 error 81", 1, 1, 401, 70, 30);

        check_drive("V2 left error 21", 1, 1, 299, 60, 70);
        check_drive("V2 left error 80", 1, 1, 240, 45, 85);
        check_drive("V2 left error 81", 1, 1, 239, 30, 70);
        // Image boundaries and PWM saturation
        check_drive("far right edge", 1, 1, 639, 120, 0);
        check_drive("far left edge", 1, 1, 0, 0, 120);

        // Disabled controller must override a large error
        check_drive("disabled at image edge", 0, 1, 639, 0, 0);

        // Missing target must override previous valid output
        check_drive("reacquired centre", 1, 1, 320, 80, 80);
        check_drive("lost target stops PWM", 1, 0, 320, 0, 0);

        $display("-------------------------------------------");
        $display("PASS: steering_controller_tb (%0d checks)",
                 checks);
        $display("-------------------------------------------");

        $finish;
    end

endmodule

`default_nettype wire
