`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Major Project - V1
//
// Motion controller -> chassis_drive -> UGV02 UART receiver
//
// Checks the actual JSON commands decoded by the provided
// university UGV02 simulation model.
//
// Uses the real 50 MHz FPGA clock and 115200-baud UART.
// Command period shortened to 5 ms for simulation.
//
// Run with +wave to generate motion_uart.fst.

module motion_uart_tb;

    logic clk = 1'b0;
    always #10 clk = ~clk;  // 50 MHz

    logic rst_n = 1'b0;
    logic arm = 1'b0;
    logic emergency_stop = 1'b0;

    logic frame_done = 1'b0;
    logic target_valid = 1'b0;
    logic [9:0] target_x = 10'd320;

    logic sonar_valid = 1'b0;
    logic [15:0] distance_mm = 16'd900;

    wire motor_enable;
    wire signed [15:0] left_pwm;
    wire signed [15:0] right_pwm;

    wire [2:0] state_debug;
    wire [9:0] target_x_debug;

    wire uart_tx;
    wire uart_busy;

    // The Ed chassis driver expects signed 10-bit PWM values.
    wire signed [9:0] chassis_left;
    wire signed [9:0] chassis_right;

    assign chassis_left  = left_pwm[9:0];
    assign chassis_right = right_pwm[9:0];

    int checks = 0;

    // ----------------------------------------------------
    // 1. Our navigation and motor control system
    // ----------------------------------------------------

    motion_interface u_motion (
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

    // ----------------------------------------------------
    // 2. University-provided chassis UART driver
    // ----------------------------------------------------

    chassis_drive #(
        .CLK_HZ(50_000_000),
        .BAUD(115_200),
        .PERIOD_MS(5)
    ) u_chassis_drive (
        .clk(clk),
        .reset(!rst_n),

        .enable(motor_enable),
        .left(chassis_left),
        .right(chassis_right),

        .tx(uart_tx),
        .busy(uart_busy)
    );

    // ----------------------------------------------------
    // 3. University-provided UGV02 simulation model
    // ----------------------------------------------------

    real robot_x;
    real robot_y;
    real robot_heading;

    real velocity_left;
    real velocity_right;

    real command_left;
    real command_right;

    int commands_received;
    int protocol_problems;

    ugv02_model #(
        .NAME("MTRX3700-V1-UGV02")
    ) u_robot (
        .rx(uart_tx),
        .tx(),

        .x_m(robot_x),
        .y_m(robot_y),
        .heading_rad(robot_heading),

        .v_left(velocity_left),
        .v_right(velocity_right),

        .cmd_left(command_left),
        .cmd_right(command_right),

        .commands(commands_received),
        .problems(protocol_problems)
    );

    // ----------------------------------------------------
    // 4. Testbench utilities
    // ----------------------------------------------------

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

    // Wait for two complete UART lines.
    //
    // This avoids accidentally checking a command that began
    // transmission immediately before the inputs changed.
    //
    // Compare the line actually decoded by the UGV02 model.

    task automatic expect_uart(
        input string test_name,
        input string expected_json,
        input bit expected_enable,
        input int expected_left,
        input int expected_right
    );

        int start_lines;

        // Check the immediate internal motion command first.
        #1;

        if (motor_enable !== expected_enable ||
            int'(left_pwm) !== expected_left ||
            int'(right_pwm) !== expected_right) begin

            $fatal(1,
                "FAIL %s: internal motor outputs enable=%b left=%0d right=%0d, expected=%b/%0d/%0d",
                test_name,
                motor_enable,
                left_pwm,
                right_pwm,
                expected_enable,
                expected_left,
                expected_right
            );
        end

        start_lines = u_robot.lines;

        wait (u_robot.lines >= start_lines + 2);

        // Check the actual line of UART JSON received.
        if (u_robot.last_line != expected_json) begin

            $fatal(1,
                "FAIL %s: UGV02 received %s; expected %s",
                test_name,
                u_robot.last_line,
                expected_json
            );
        end

        if (protocol_problems != 0) begin
            $fatal(1,
                "FAIL %s: UGV02 reported %0d UART/protocol problems",
                test_name,
                protocol_problems
            );
        end

        checks++;

        $display(
            "PASS [%0d]: %s | UART received %s",
            checks, test_name, u_robot.last_line
        );

    endtask

    // Simulation watchdog: fail instead of hanging forever.
    initial begin
        #500_000_000;
        $fatal(1, "TIMEOUT: motion UART integration test");
    end

    // ----------------------------------------------------
    // 5. Integration scenarios
    // ----------------------------------------------------

    initial begin

        if ($test$plusargs("wave")) begin
            $dumpfile("motion_uart.fst");
            $dumpvars(0, motion_uart_tb);
        end

        // Reset the FPGA design and UART driver.
        repeat (5) @(negedge clk);

        rst_n = 1'b1;

        // Test 1: arm is off.
        expect_uart(
            "Disarmed robot",
            "{\"T\":11,\"L\":0,\"R\":0}",
            0, 0, 0
        );

        // Arm robot; no target available.
        @(negedge clk);
        arm = 1'b1;
        sonar_valid = 1'b1;

        tick();

        // Test 2: no LED detected.
        expect_uart(
            "Armed without target",
            "{\"T\":11,\"L\":0,\"R\":0}",
            0, 0, 0
        );

        // Simulate successful LED detection.
        send_frame(1, 320);

        // Test 3: centred LED, straight movement.
        expect_uart(
            "Centred LED",
            "{\"T\":11,\"L\":80,\"R\":80}",
            1, 80, 80
        );

        // Test 4: LED on right.
        send_frame(1, 400);

        expect_uart(
            "LED right steering",
            "{\"T\":11,\"L\":85,\"R\":45}",
            1, 85, 45
        );

        // Test 5: LED on left.
        send_frame(1, 240);

        expect_uart(
            "LED left steering",
            "{\"T\":11,\"L\":45,\"R\":85}",
            1, 45, 85
        );

        // Test 6: sonar becomes invalid.
        @(negedge clk);
        sonar_valid = 1'b0;

        expect_uart(
            "Sonar failure",
            "{\"T\":11,\"L\":0,\"R\":0}",
            0, 0, 0
        );

        // Restore sonar validity.
        @(negedge clk);
        sonar_valid = 1'b1;

        // Test 7: valid sonar restores movement.
        expect_uart(
            "Sonar restored",
            "{\"T\":11,\"L\":45,\"R\":85}",
            1, 45, 85
        );

        // Test 8: target disappears.
        send_frame(0, 0);

        expect_uart(
            "LED target lost",
            "{\"T\":11,\"L\":0,\"R\":0}",
            0, 0, 0
        );

        // Reacquire LED.
        send_frame(1, 320);

        expect_uart(
            "LED reacquired",
            "{\"T\":11,\"L\":80,\"R\":80}",
            1, 80, 80
        );

        // Test 10: emergency stop while moving.
        @(negedge clk);
        emergency_stop = 1'b1;

        expect_uart(
            "Emergency stop",
            "{\"T\":11,\"L\":0,\"R\":0}",
            0, 0, 0
        );

        // Advance the FSM into its latched stopped state.
        tick();

        // Release emergency input without disarming.
        @(negedge clk);
        emergency_stop = 1'b0;

        tick();

        // Test 11: must remain stopped.
        expect_uart(
            "Emergency state remains latched",
            "{\"T\":11,\"L\":0,\"R\":0}",
            0, 0, 0
        );

        // Explicitly disarm and rearm.
        @(negedge clk);
        arm = 1'b0;

        tick();

        @(negedge clk);
        arm = 1'b1;

        tick();

        send_frame(1, 320);

        // Test 12: rearming permits driving again.
        expect_uart(
            "Rearmed and reacquired",
            "{\"T\":11,\"L\":80,\"R\":80}",
            1, 80, 80
        );

        // Test 13: obstacle reaches docking distance.
        @(negedge clk);
        distance_mm = 16'd250;

        expect_uart(
            "Docking threshold reached",
            "{\"T\":11,\"L\":0,\"R\":0}",
            0, 0, 0
        );

        // Enter DOCKED.
        tick();

        // Obstacle moves away.
        @(negedge clk);
        distance_mm = 16'd900;

        tick();

        // Test 14: docking stays latched.
        expect_uart(
            "Docking state stays stopped",
            "{\"T\":11,\"L\":0,\"R\":0}",
            0, 0, 0
        );

        $display("--------------------------------------------");
        $display(
            "PASS: motion_uart_tb (%0d checks)",
            checks
        );
        $display("UGV02 UART protocol errors: %0d", protocol_problems);
        $display("--------------------------------------------");

        $finish;
    end

endmodule

`default_nettype wire
