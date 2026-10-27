`timescale 1ns/1ps
`default_nettype none

module motion_interface_tb;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic rst_n = 1'b0;
    logic arm = 1'b0;
    logic emergency_stop = 1'b0;

    logic frame_done = 1'b0;
    logic target_valid = 1'b0;
    logic [9:0] target_x = 10'd320;

    logic sonar_valid = 1'b0;
    logic [15:0] distance_mm = 16'd900;

    logic motor_enable;
    logic signed [15:0] left_pwm;
    logic signed [15:0] right_pwm;
    wire [2:0] state_debug;
    wire [9:0] target_x_debug;

    int checks = 0;

    motion_interface dut (
        .clk(clk),
        .rst_n(rst_n),
        .arm(arm),
        .emergency_stop(emergency_stop),
        .frame_done(frame_done),
        .target_valid(target_valid),
        .target_x(target_x),
        .sonar_valid(sonar_valid),
        .distance_mm(distance_mm),
        .motor_enable(motor_enable),
        .left_pwm(left_pwm),
        .right_pwm(right_pwm),
        .state_debug(state_debug),
        .target_x_debug(target_x_debug)
    );

    task automatic tick;
        @(posedge clk);
        #1;
    endtask

    task automatic send_frame(
        input bit found,
        input logic [9:0] x
    );
        @(negedge clk);
        frame_done = 1'b1;
        target_valid = found;
        target_x = x;
        tick();
        @(negedge clk);
        frame_done = 1'b0;
        target_valid = 1'b0;
    endtask

    task automatic check_motion(
        input string name,
        input bit expected_enable,
        input int expected_left,
        input int expected_right
    );
        #1;
        if (motor_enable !== expected_enable ||
            int'(left_pwm) !== expected_left ||
            int'(right_pwm) !== expected_right)
            $fatal(1,
                "FAIL %s: enable=%b left=%0d right=%0d; expected=%b %0d %0d",
                name, motor_enable, left_pwm, right_pwm,
                expected_enable, expected_left, expected_right);

        checks++;
        $display("PASS [%0d]: %s", checks, name);
    endtask

    initial begin
        $dumpfile("motion_interface.vcd");
        $dumpvars(0, motion_interface_tb);

        repeat (3) @(negedge clk);
        check_motion("reset disables motors", 0, 0, 0);

        rst_n = 1'b1;
        tick();
        check_motion("disarmed motors stopped", 0, 0, 0);

        @(negedge clk);
        arm = 1'b1;
        sonar_valid = 1'b1;
        tick();
        check_motion("no target acquired", 0, 0, 0);

        send_frame(1, 320);
        check_motion("centred target", 1, 80, 80);

        send_frame(1, 400);
        check_motion("target right", 1, 85, 45);

        send_frame(1, 240);
        check_motion("target left", 1, 45, 85);

        @(negedge clk);
        sonar_valid = 1'b0;
        check_motion("invalid sonar stops immediately", 0, 0, 0);

        @(negedge clk);
        sonar_valid = 1'b1;
        check_motion("fresh sonar restores driving", 1, 45, 85);

        send_frame(0, 0);
        check_motion("lost target stops", 0, 0, 0);

        send_frame(1, 320);
        check_motion("reacquired target", 1, 80, 80);

        @(negedge clk);
        emergency_stop = 1'b1;
        check_motion("emergency gates motor outputs", 0, 0, 0);

        tick();
        check_motion("emergency stopped state", 0, 0, 0);

        @(negedge clk);
        emergency_stop = 1'b0;
        tick();
        check_motion("emergency state remains latched", 0, 0, 0);

        @(negedge clk);
        arm = 1'b0;
        tick();
        check_motion("disarm clears stopped state", 0, 0, 0);

        @(negedge clk);
        arm = 1'b1;
        tick();
        send_frame(1, 320);
        check_motion("rearmed and reacquired", 1, 80, 80);

        @(negedge clk);
        distance_mm = 16'd250;
        check_motion("dock threshold stops motors", 0, 0, 0);

        tick();
        check_motion("docked state remains stopped", 0, 0, 0);

        @(negedge clk);
        distance_mm = 16'd900;
        tick();
        check_motion("docked cannot spontaneously restart", 0, 0, 0);

        $display("-----------------------------------------");
        $display("PASS: motion_interface_tb (%0d checks)", checks);
        $display("-----------------------------------------");
        $finish;
    end

endmodule

`default_nettype wire
