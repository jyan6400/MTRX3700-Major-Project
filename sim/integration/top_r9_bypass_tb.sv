`timescale 1ns/1ps
`default_nettype none

module top_r9_bypass_tb;

    logic CLOCK_50 = 0;
    always #10 CLOCK_50 = ~CLOCK_50;

    logic cam_pclk = 0;
    always #12.5 cam_pclk = ~cam_pclk;

    logic [3:0] KEY = 4'b1110;
    logic [9:0] SW = 10'd0;
    logic IRDA_RXD = 1'b1;

    tri [35:0] GPIO_1;

    assign GPIO_1[21] = cam_pclk;
    assign GPIO_1[22] = 1'b0;
    assign GPIO_1[23] = 1'b1;
    assign GPIO_1[19:12] = 8'd0;
    assign GPIO_1[29] = 1'b0;
    assign GPIO_1[24] = 1'bz;

    wire [9:0] LEDR;
    int checks = 0;

    major_project_top #(
        .UART_PERIOD_MS(1)
    ) dut (
        .CLOCK_50(CLOCK_50),
        .KEY(KEY),
        .SW(SW),
        .IRDA_RXD(IRDA_RXD),
        .GPIO_1(GPIO_1),
        .LEDR(LEDR)
    );

    task automatic settle;
        repeat (8) @(negedge CLOCK_50);
    endtask

    task automatic check(
        input string description,
        input bit condition
    );
        if (!condition)
            $fatal(1, "FAIL: %s", description);

        checks++;
        $display("PASS [%0d]: %s", checks, description);
    endtask

    // Inject a sonar reading at the CDC-to-filter boundary.
    // The actual sonar_filter remains operational.
    task automatic sonar_reading(input logic [15:0] mm);
        @(negedge CLOCK_50);
        force dut.sonar_measurement_mm = mm;
        force dut.sonar_measure_valid = 1'b1;

        // Exactly one 50 MHz clock cycle per sonar reading.
        @(negedge CLOCK_50);
        force dut.sonar_measure_valid = 1'b0;

        settle();
    endtask

    initial begin
        // Reset and PLL startup.
        repeat (20) @(negedge CLOCK_50);
        KEY[0] = 1'b1;
        repeat (100) @(negedge CLOCK_50);

        check("system reset released", dut.rst_n_50);

        // Control unrelated subsystem boundaries.
        // Do NOT force sonar_valid, distance_mm,
        // sonar_forward_clear or either bypass signal.
        force dut.motion_permitted = 1'b1;
        force dut.emergency_stop = 1'b0;
        force dut.camera_target_fresh = 1'b1;
        force dut.motor_enable = 1'b1;
        force dut.left_pwm = 16'sd80;
        force dut.right_pwm = 16'sd80;

        force dut.sonar_measure_valid = 1'b0;
        force dut.sonar_measurement_mm = 16'd0;

        // ------------------------------------------------
        // Test SW1: target-validation bypass
        // ------------------------------------------------
        SW[1] = 1'b0;
        repeat (8) @(negedge cam_pclk);

        check("SW1=0 selects V2 validation",
              dut.target_bypass_sync[1] === 1'b0);

        SW[1] = 1'b1;
        repeat (8) @(negedge cam_pclk);

        check("SW1=1 selects V1 target bypass",
              dut.target_bypass_sync[1] === 1'b1);

        SW[1] = 1'b0;
        repeat (8) @(negedge cam_pclk);

        check("SW1 returns to V2 validation",
              dut.target_bypass_sync[1] === 1'b0);

        // ------------------------------------------------
        // Test SW2: real sonar filtering
        // ------------------------------------------------
        SW[2] = 1'b0;
        settle();

        check("SW2=0 selects V2 sonar",
              dut.u_sonar_filter.bypass_active === 1'b0);

        sonar_reading(16'd600);
        check("V2 warm-up requires more readings",
              dut.sonar_valid === 1'b0);

        sonar_reading(16'd570);
        check("V2 still warming up",
              dut.sonar_valid === 1'b0);

        sonar_reading(16'd540);
        check("V2 warm-up completed",
              dut.sonar_valid === 1'b1 &&
              dut.distance_mm == 16'd540 &&
              dut.sonar_forward_clear === 1'b1);

        // Arm with verified clearance.
        SW[0] = 1'b1;
        settle();

        check("armed robot permitted to move",
              dut.safe_motor_enable === 1'b1);

        // First unexpected close echo.
        sonar_reading(16'd150);

        check("V2 rejects false docking distance",
              dut.distance_mm > 16'd250);

        check("V2 immediately inhibits forward motion",
              dut.sonar_forward_clear === 1'b0 &&
              dut.safe_motor_enable === 1'b0);

        // Same filtered path, bypass enabled.
        SW[2] = 1'b1;
        settle();

        check("SW2=1 activates V1 sonar bypass",
              dut.u_sonar_filter.bypass_active === 1'b1);

        check("V1 accepts the single 150 mm echo",
              dut.sonar_valid === 1'b1 &&
              dut.distance_mm == 16'd150);

        check("V1 threshold still prevents forward drive",
              dut.safe_motor_enable === 1'b0);

        // V1 immediately accepts the next reading.
        sonar_reading(16'd500);

        check("V1 restores clearance after valid echo",
              dut.distance_mm == 16'd500 &&
              dut.sonar_forward_clear === 1'b1);

        // Disarming must override sonar clearance.
        SW[0] = 1'b0;
        settle();

        check("SW0 disarm overrides V1 bypass",
              dut.safe_motor_enable === 1'b0);

        // E-stop must override a bypass-enabled state.
        SW[0] = 1'b1;
        settle();
        force dut.emergency_stop = 1'b1;

        settle();
        check("E-stop overrides V1 bypass",
              dut.safe_motor_enable === 1'b0);

        // Return to V2 and verify E-stop remains active.
        SW[2] = 1'b0;
        settle();

        check("SW2 returns to V2 mode",
              dut.u_sonar_filter.bypass_active === 1'b0);

        check("E-stop also overrides V2",
              dut.safe_motor_enable === 1'b0);

        $display("----------------------------------------");
        $display("PASS: top_r9_bypass_tb (%0d checks)", checks);
        $display("----------------------------------------");
        $finish;
    end

    initial begin
        #20ms;
        $fatal(1, "TIMEOUT: top_r9_bypass_tb");
    end

endmodule

`default_nettype wire
