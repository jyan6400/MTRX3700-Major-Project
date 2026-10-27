`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Group 26 - V1 top-level safety smoke test.
//
// This test instantiates the real major_project_top,
// including PLL models, sensing modules, navigation
// modules and the actual UGV02 chassis_drive.
//
// Controlled internal signals are forced to exercise the
// final safety gate without simulating full camera frames.
//
// This is a top-level control integration test, NOT a
// complete sensor-to-UART protocol regression.
//
// Only the TB uses force. Nothing is forced in the
// synthesizable FPGA design.

module major_project_top_tb;

    logic CLOCK_50 = 1'b0;
    always #10 CLOCK_50 = ~CLOCK_50;

    // Supply an independent camera pixel clock.
    logic cam_pclk = 1'b0;
    always #12.5 cam_pclk = ~cam_pclk;

    logic [3:0] KEY = 4'b1110;
    logic [9:0] SW = 10'd0;
    logic IRDA_RXD = 1'b1;

    tri [35:0] GPIO_1;

    // Simulated camera inputs. No full image is supplied
    // in this quick control-path test.
    assign GPIO_1[21]    = cam_pclk;
    assign GPIO_1[22]    = 1'b0;
    assign GPIO_1[23]    = 1'b1;
    assign GPIO_1[19:12] = 8'd0;

    // The sonar echo input idles low.
    assign GPIO_1[29] = 1'b0;

    // SCCB data remains undriven externally in this
    // smoke test; camera configuration is not checked.
    assign GPIO_1[24] = 1'bz;

    wire [9:0] LEDR;

    major_project_top #(
        .CAMERA_FPS(50),
        .UART_PERIOD_MS(1)
    ) dut (
        .CLOCK_50(CLOCK_50),
        .KEY(KEY),
        .SW(SW),
        .IRDA_RXD(IRDA_RXD),
        .GPIO_1(GPIO_1),
        .LEDR(LEDR)
    );

    int checks = 0;

    task automatic verify_drive(
        input string description,
        input bit expected_enable,
        input logic signed [9:0] expected_left,
        input logic signed [9:0] expected_right
    );
        repeat (4) @(posedge CLOCK_50);
        @(negedge CLOCK_50);

        if (dut.safe_motor_enable !== expected_enable)
            $fatal(1,
                "FAIL %s: safe_motor_enable=%b expected=%b",
                description,
                dut.safe_motor_enable,
                expected_enable
            );

        if (dut.chassis_left !== expected_left ||
            dut.chassis_right !== expected_right)
            $fatal(1,
                "FAIL %s: PWM=(%0d,%0d) expected=(%0d,%0d)",
                description,
                $signed(dut.chassis_left),
                $signed(dut.chassis_right),
                $signed(expected_left),
                $signed(expected_right)
            );

        // Check that the real Ed chassis_drive receives
        // the same gated values.
        if (dut.u_chassis.enable !== expected_enable ||
            dut.u_chassis.left !== expected_left ||
            dut.u_chassis.right !== expected_right)
            $fatal(1,
                "FAIL %s: chassis_drive input mismatch",
                description
            );

        checks++;
        $display("PASS [%0d]: %s", checks, description);
    endtask

    initial begin
        // Allow all clocks and PLL stand-ins to start.
        repeat (20) @(negedge CLOCK_50);

        verify_drive(
            "reset inhibits motor commands",
            0, 0, 0
        );

        KEY[0] = 1'b1;

        repeat (30) @(negedge CLOCK_50);

        // Use force only to control subsystem boundaries.
        // The top-level safety logic and chassis_drive
        // remain active and are tested normally.

        force dut.arm                 = 1'b0;
        force dut.motion_permitted    = 1'b0;
        force dut.emergency_stop      = 1'b1;
        force dut.camera_target_fresh = 1'b0;
        force dut.sonar_valid         = 1'b0;
        force dut.distance_mm         = 16'd0;
        force dut.sonar_forward_clear = 1'b0;
        force dut.motor_enable        = 1'b1;
        force dut.left_pwm            = 16'sd80;
        force dut.right_pwm           = 16'sd80;

        verify_drive(
            "disarmed robot remains stopped",
            0, 0, 0
        );

        // Arm only.
        force dut.arm = 1'b1;

        verify_drive(
            "IR stop still latched",
            0, 0, 0
        );

        // Release IR but leave sensors invalid.
        force dut.motion_permitted = 1'b1;
        force dut.emergency_stop = 1'b0;

        verify_drive(
            "missing camera and sonar inhibit motion",
            0, 0, 0
        );

        // Valid target but no ultrasonic measurement.
        force dut.camera_target_fresh = 1'b1;

        verify_drive(
            "missing sonar inhibits motion",
            0, 0, 0
        );

        // A fresh sonar measurement above stopping distance.
        force dut.sonar_valid = 1'b1;
        force dut.distance_mm = 16'd1000;
        force dut.sonar_forward_clear = 1'b1;

        verify_drive(
            "all safety conditions allow forward PWM",
            1, 80, 80
        );

        // Confirm that differential PWM is preserved.
        force dut.left_pwm = 16'sd100;
        force dut.right_pwm = 16'sd60;

        verify_drive(
            "left/right steering PWM reaches chassis",
            1, 100, 60
        );

        // Docking threshold.
        force dut.distance_mm = 16'd250;
        force dut.sonar_forward_clear = 1'b0;

        verify_drive(
            "250 mm docking threshold inhibits motion",
            0, 0, 0
        );

        // Below threshold must also remain stopped.
        force dut.distance_mm = 16'd200;
        force dut.sonar_forward_clear = 1'b0;

        verify_drive(
            "obstacle below threshold inhibits motion",
            0, 0, 0
        );

        // Restore distance, then simulate camera timeout.
        force dut.distance_mm = 16'd1000;
        force dut.sonar_forward_clear = 1'b1;
        force dut.camera_target_fresh = 1'b0;

        verify_drive(
            "expired camera frame inhibits motion",
            0, 0, 0
        );

        // Camera recovers.
        force dut.camera_target_fresh = 1'b1;

        verify_drive(
            "fresh camera target permits motion again",
            1, 100, 60
        );

        // Emergency stop during approach.
        force dut.emergency_stop = 1'b1;
        force dut.motion_permitted = 1'b0;

        verify_drive(
            "emergency stop overrides movement",
            0, 0, 0
        );

        // Return to a valid, permitted state.
        // The separate IR latch unit test verifies that
        // release requires the correct remote command.
        force dut.emergency_stop = 1'b0;
        force dut.motion_permitted = 1'b1;

        verify_drive(
            "permitted control state restores drive",
            1, 100, 60
        );

        // Simulate motion controller disabling movement.
        force dut.motor_enable = 1'b0;

        verify_drive(
            "navigation FSM stop request inhibits motion",
            0, 0, 0
        );

        // Restore the motor request, disarm physically.
        force dut.motor_enable = 1'b1;
        force dut.arm = 1'b0;

        verify_drive(
            "disarm overrides movement",
            0, 0, 0
        );

        $display("-------------------------------------------");
        $display(
            "PASS: major_project_top_tb (%0d checks)",
            checks
        );
        $display(
            "Top-level motor safety gates verified."
        );
        $display(
            "NOTE: camera/sonar/IR stimulus was injected."
        );
        $display(
            "NOTE: UART serial payload not yet decoded."
        );
        $display("-------------------------------------------");

        $finish;
    end

    initial begin
        #10ms;
        $fatal(1, "TIMEOUT: top-level safety test");
    end

endmodule

`default_nettype wire
