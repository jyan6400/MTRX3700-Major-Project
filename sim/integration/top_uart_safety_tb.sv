`timescale 1ns/1ps
`default_nettype none

module top_uart_safety_tb;

    localparam int BIT_NS = 8681; // 115200 baud

    logic CLOCK_50 = 1'b0;
    always #10 CLOCK_50 = ~CLOCK_50;

    logic cam_pclk = 1'b0;
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

    wire uart_tx = GPIO_1[31];

    string last_command = "";
    int command_count = 0;
    int checks = 0;
    realtime last_command_time = 0;

    // Decode one complete UART 8N1 character.
    // Called after detecting a falling edge at the
    // beginning of a UART start bit.
    task automatic uart_receive_byte(
        output byte unsigned data
    );
        data = 8'd0;

        // Verify the middle of the start bit.
        #(BIT_NS / 2);

        if (uart_tx !== 1'b0)
            $fatal(1, "UART invalid start bit");

        // Sample eight data bits at their centres.
        for (int i = 0; i < 8; i++) begin
            #BIT_NS;
            data[i] = uart_tx;
        end

        // Sample stop bit at its centre.
        #BIT_NS;

        if (uart_tx !== 1'b1)
            $fatal(1, "UART invalid stop bit");

        // Move to the end of the stop bit.
        // This prevents detecting transitions inside
        // the previous character as a new start bit.
        #(BIT_NS / 2);
    endtask

    initial begin : uart_monitor
        byte unsigned ch;
        string assembling;

        assembling = "";

        forever begin
            @(negedge uart_tx);
            uart_receive_byte(ch);

            // Vendor UART includes NUL padding and
            // newline-delimited JSON commands.
            // Neither belongs in the JSON string.
            if (ch == 8'h00 ||
                ch == 8'h0A ||
                ch == 8'h0D)
                continue;

            // New message begins at an opening brace.
            if (ch == 8'h7B)
                assembling = "";

            // Reject bytes that do not belong to a
            // recognised JSON command.
            if ((ch < 8'h20) || (ch > 8'h7E)) begin
                $fatal(1,
                    "Unexpected UART byte 0x%02h",
                    ch
                );
            end

            assembling = {
                assembling,
                $sformatf("%c", ch)
            };

            if (ch == 8'h7D) begin
                last_command = assembling;
                last_command_time = $realtime;
                command_count++;

                $display(
                    "[UART @ %0t] %s",
                    $time,
                    last_command
                );

                assembling = "";
            end

            if (assembling.len() > 100)
                $fatal(1, "UART message too long");
        end
    end
    // Wait for a complete expected UART JSON message.
    task automatic expect_uart(
        input string description,
        input string expected
    );
        bit found;

        found = 0;

        // Eight milliseconds accommodates multiple UART
        // command periods and a command already in progress.
        for (int k = 0; k < 80; k++) begin
            #100us;

            if (last_command == expected) begin
                found = 1;
                break;
            end
        end

        if (!found)
            $fatal(1,
                "FAIL %s: expected %s, last received %s",
                description,
                expected,
                last_command
            );

        checks++;
        $display(
            "PASS [%0d]: %s",
            checks,
            description
        );
    endtask

    // Change a test stimulus between 50 MHz clock edges.
    task automatic settle;
        repeat (5) @(negedge CLOCK_50);
    endtask

    initial begin
        // Reset.
        repeat (20) @(negedge CLOCK_50);
        KEY[0] = 1'b1;

        repeat (30) @(negedge CLOCK_50);

        // Controlled subsystem boundary stimulus.
        force dut.arm = 1'b0;
        force dut.motion_permitted = 1'b0;
        force dut.emergency_stop = 1'b1;
        force dut.camera_target_fresh = 1'b0;
        force dut.sonar_valid = 1'b0;
        force dut.distance_mm = 16'd0;
        force dut.sonar_forward_clear = 1'b0;

        force dut.motor_enable = 1'b1;
        force dut.left_pwm = 16'sd80;
        force dut.right_pwm = 16'sd80;

        expect_uart(
            "disarmed startup sends stop",
            "{\"T\":11,\"L\":0,\"R\":0}"
        );

        force dut.arm = 1'b1;
        force dut.motion_permitted = 1'b1;
        force dut.emergency_stop = 1'b0;
        force dut.camera_target_fresh = 1'b1;
        force dut.sonar_valid = 1'b1;
        force dut.distance_mm = 16'd1000;
        force dut.sonar_forward_clear = 1'b1;

        settle();

        if (!dut.safe_motor_enable)
            $fatal(1, "Forward movement unexpectedly gated");

        expect_uart(
            "forward movement reaches UART",
            "{\"T\":11,\"L\":80,\"R\":80}"
        );

        force dut.left_pwm = 16'sd100;
        force dut.right_pwm = 16'sd60;

        expect_uart(
            "differential steering reaches UART",
            "{\"T\":11,\"L\":100,\"R\":60}"
        );

        // Trigger a docking stop.
        force dut.distance_mm = 16'd250;
        force dut.sonar_forward_clear = 1'b0;

        settle();

        if (dut.safe_motor_enable)
            $fatal(1, "Docking threshold did not gate motors");

        expect_uart(
            "docking threshold sends stop",
            "{\"T\":11,\"L\":0,\"R\":0}"
        );

        // Restore approach, then simulate loss of camera data.
        force dut.distance_mm = 16'd1000;
        force dut.sonar_forward_clear = 1'b1;
        force dut.camera_target_fresh = 1'b1;

        expect_uart(
            "approach resumes with fresh camera",
            "{\"T\":11,\"L\":100,\"R\":60}"
        );

        force dut.camera_target_fresh = 1'b0;

        expect_uart(
            "camera timeout sends stop",
            "{\"T\":11,\"L\":0,\"R\":0}"
        );

        // Restore approach, then trigger emergency stop.
        force dut.camera_target_fresh = 1'b1;

        expect_uart(
            "approach resumes before emergency stop",
            "{\"T\":11,\"L\":100,\"R\":60}"
        );

        force dut.emergency_stop = 1'b1;
        force dut.motion_permitted = 1'b0;

        expect_uart(
            "emergency stop reaches UART",
            "{\"T\":11,\"L\":0,\"R\":0}"
        );

        $display("---------------------------------------");
        $display("PASS: top_uart_safety_tb (%0d checks)", checks);
        $display("UART commands received: %0d", command_count);
        $display("---------------------------------------");

        $finish;
    end

    initial begin
        #100ms;
        $fatal(1, "TIMEOUT: UART safety integration");
    end

endmodule

`default_nettype wire


