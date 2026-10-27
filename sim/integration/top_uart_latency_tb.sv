`timescale 1ns/1ps
`default_nettype none

module top_uart_latency_tb;

    localparam int BIT_NS = 8681;
    localparam real MAX_STOP_MS = 20.0;

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

    // IMPORTANT: production UART period = 10 ms.
    major_project_top #(
        .UART_PERIOD_MS(10)
    ) dut (
        .CLOCK_50(CLOCK_50),
        .KEY(KEY),
        .SW(SW),
        .IRDA_RXD(IRDA_RXD),
        .GPIO_1(GPIO_1),
        .LEDR(LEDR)
    );

    wire uart_tx = GPIO_1[31];

    localparam string STOP_CMD =
        "{\"T\":11,\"L\":0,\"R\":0}";

    localparam string FORWARD_CMD =
        "{\"T\":11,\"L\":80,\"R\":80}";

    string last_command = "";
    int command_count = 0;
    realtime last_command_time = 0;

    int checks = 0;
    real worst_stop_ms = 0.0;

    task automatic uart_receive_byte(
        output byte unsigned data
    );
        data = 8'd0;

        #(BIT_NS / 2);

        if (uart_tx !== 1'b0)
            $fatal(1, "Invalid UART start bit");

        for (int i = 0; i < 8; i++) begin
            #BIT_NS;
            data[i] = uart_tx;
        end

        #BIT_NS;

        if (uart_tx !== 1'b1)
            $fatal(1, "Invalid UART stop bit");

        #(BIT_NS / 2);
    endtask

    // Decode real UART wire into completed JSON commands.
    initial begin : uart_monitor
        byte unsigned ch;
        string assembling;

        assembling = "";

        forever begin
            @(negedge uart_tx);
            uart_receive_byte(ch);

            // Vendor command padding/delimiters.
            if (ch == 8'h00 ||
                ch == 8'h0A ||
                ch == 8'h0D)
                continue;

            if (ch < 8'h20 || ch > 8'h7E)
                $fatal(1,
                    "Unexpected UART byte: 0x%02h", ch);

            if (ch == 8'h7B)
                assembling = "";

            assembling = {
                assembling,
                $sformatf("%c", ch)
            };

            if (ch == 8'h7D) begin
                last_command = assembling;
                last_command_time = $realtime;
                command_count++;

                $display(
                    "[UART @ %0.3f ms] %s",
                    $realtime / 1_000_000.0,
                    last_command
                );

                assembling = "";
            end

            if (assembling.len() > 100)
                $fatal(1, "UART command too long");
        end
    end

    // Wait for a specific command AFTER the given
    // command index, avoiding previously received
    // messages being mistaken for new ones.
    task automatic wait_command(
        input string expected,
        input int previous_count
    );
        int observed;

        observed = previous_count;

        forever begin
            wait (command_count > observed);

            if (last_command == expected)
                break;

            observed = command_count;
        end
    endtask

    task automatic verify_stop_latency(
        input string description,
        input realtime event_time,
        input int previous_count
    );
        real latency_ms;

        wait_command(STOP_CMD, previous_count);

        latency_ms =
            (last_command_time - event_time) /
            1_000_000.0;

        $display(
            "STOP LATENCY [%s] = %0.3f ms",
            description,
            latency_ms
        );

        if (latency_ms < 0.0)
            $fatal(1, "Stop arrived before event");

        if (latency_ms > MAX_STOP_MS)
            $fatal(1,
                "FAIL %s: latency %0.3f ms > %0.3f ms",
                description, latency_ms, MAX_STOP_MS);

        if (latency_ms > worst_stop_ms)
            worst_stop_ms = latency_ms;

        checks++;

        $display(
            "PASS [%0d]: %s",
            checks, description
        );
    endtask

    initial begin : test_sequence
        realtime event_time;
        int previous_count;

        repeat (20) @(negedge CLOCK_50);
        KEY[0] = 1'b1;
        repeat (30) @(negedge CLOCK_50);

        // Inject known-safe perception/navigation states.
        // The actual top-level safety gate and UART
        // transmitter operate without being forced.
        force dut.arm = 1'b1;
        force dut.motion_permitted = 1'b1;
        force dut.emergency_stop = 1'b0;
        force dut.camera_target_fresh = 1'b1;
        force dut.sonar_valid = 1'b1;
        force dut.distance_mm = 16'd1000;
        force dut.sonar_forward_clear = 1'b1;
        force dut.motor_enable = 1'b1;
        force dut.left_pwm = 16'sd80;
        force dut.right_pwm = 16'sd80;

        // --------------------------------------------
        // Test 1: IR emergency stop during forward drive
        // --------------------------------------------

        wait_command(FORWARD_CMD, 0);

        checks++;
        $display("PASS [%0d]: forward command received", checks);

        // Trigger shortly after the completed forward
        // command. This tests an unfavourable phase
        // of the 100 ms periodic command schedule.
        #1ms;

        @(negedge CLOCK_50);
        previous_count = command_count;
        event_time = $realtime;

        force dut.emergency_stop = 1'b1;
        force dut.motion_permitted = 1'b0;

        #1;

        if (dut.safe_motor_enable !== 1'b0)
            $fatal(1,
                "Emergency stop did not disable motor gate");

        verify_stop_latency(
            "IR emergency stop",
            event_time,
            previous_count
        );

        // --------------------------------------------
        // Test 2: obstacle at docking threshold
        // --------------------------------------------

        @(negedge CLOCK_50);

        force dut.emergency_stop = 1'b0;
        force dut.motion_permitted = 1'b1;
        force dut.distance_mm = 16'd1000;
        force dut.sonar_forward_clear = 1'b1;

        previous_count = command_count;

        wait_command(FORWARD_CMD, previous_count);

        checks++;
        $display("PASS [%0d]: approach resumed", checks);

        #1ms;

        @(negedge CLOCK_50);
        previous_count = command_count;
        event_time = $realtime;

        force dut.distance_mm = 16'd250;
        force dut.sonar_forward_clear = 1'b0;

        #1;

        if (dut.safe_motor_enable !== 1'b0)
            $fatal(1,
                "Docking threshold did not disable motor gate");

        verify_stop_latency(
            "250 mm docking stop",
            event_time,
            previous_count
        );

        $display("----------------------------------------");
        $display("PASS: top_uart_latency_tb");
        $display("Checks passed: %0d", checks);
        $display("Worst observed UART stop latency: %0.3f ms",
                 worst_stop_ms);
        $display("Production UART command period: 10 ms");
        $display("----------------------------------------");

        $finish;
    end

    initial begin
        #750ms;
        $fatal(1, "TIMEOUT: UART latency simulation");
    end

endmodule

`default_nettype wire

