`timescale 1ns/1ps
`default_nettype none
/* verilator lint_off ZERODLY */   // wait_ms uses a run-time delay (testbench only)

// ============================================================================
// MTRX3700 Major Project - Group 26 - V3
// Top-level reverse safety: major_project_top with motion_supervisor
// integrated, production 10 ms UART schedule, decoded from the real chassis
// TX pin (owner: Advay, M5; requires the V3 integration patch).
//
// The V3 navigation FSM is not present yet, so its reverse channel
// (dut.nav_rev_request / dut.nav_rev_*_pwm) is forced here, as the other top
// benches force the navigation and sensor wires. SW3 is driven through the
// real switch pin (synchronised inside the supervisor).
//
//   1. reverse requested with SW3 off: zero packets only
//   2. SW3 on, docked at 240 mm: -50/-50 packets (250 mm forward stop exempt)
//   3. IR stop, SW0 disarm and SW3 off during reverse: zero packet within
//      12 ms, no stale nonzero command, no resume without a new request
//   4. reverse -> forward: >= 100 ms of zero packets, then forward 80/80
// ============================================================================

module top_reverse_safety_tb;

    localparam int  BIT_NS      = 8681;
    localparam real MAX_STOP_MS = 12.0;

    logic CLOCK_50 = 1'b0;
    always #10 CLOCK_50 = ~CLOCK_50;

    logic cam_pclk = 1'b0;
    always #12.5 cam_pclk = ~cam_pclk;

    logic [3:0] KEY = 4'b1110;
    logic [9:0] SW = 10'd0;
    logic IRDA_RXD = 1'b1;

    tri [35:0] GPIO_1;
    assign GPIO_1[21]    = cam_pclk;
    assign GPIO_1[22]    = 1'b0;
    assign GPIO_1[23]    = 1'b1;
    assign GPIO_1[19:12] = 8'd0;
    assign GPIO_1[29]    = 1'b0;
    assign GPIO_1[24]    = 1'bz;

    wire [9:0] LEDR;

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

    // ---------------- UART decoder: (start, end, L, R) per line ----------------

    realtime pkt_start[$];
    realtime pkt_end[$];
    int      pkt_l[$];
    int      pkt_r[$];

    function automatic int find_str(input string s, input string key);
        for (int i = 0; i + key.len() <= s.len(); i++)
            if (s.substr(i, i + key.len() - 1) == key) return i;
        return -1;
    endfunction

    function automatic int field(input string s, input string key);
        int i, v, sign;
        i = find_str(s, key);
        if (i < 0) return 9999;
        i += key.len();
        sign = 1;
        if (s[i] == 8'h2D) begin sign = -1; i++; end
        v = 0;
        while (i < s.len() && s[i] >= 8'h30 && s[i] <= 8'h39) begin
            v = v * 10 + (int'(s[i]) - 48);
            i++;
        end
        return sign * v;
    endfunction

    initial begin : uart_decoder
        byte unsigned ch;
        string   line;
        realtime t_byte, t_line;
        line = "";
        t_line = 0;
        forever begin
            @(negedge uart_tx);
            t_byte = $realtime;
            ch = 8'd0;
            #(BIT_NS / 2);
            if (uart_tx !== 1'b0) $fatal(1, "FAIL: bad UART start bit");
            for (int i = 0; i < 8; i++) begin #BIT_NS; ch[i] = uart_tx; end
            #BIT_NS;
            if (uart_tx !== 1'b1) $fatal(1, "FAIL: bad UART stop bit");
            if (ch == 8'h7B) begin line = ""; t_line = t_byte; end
            if (ch != 8'h0A && ch != 8'h0D) line = {line, $sformatf("%c", ch)};
            if (ch == 8'h7D) begin
                pkt_start.push_back(t_line);
                pkt_end.push_back($realtime);
                pkt_l.push_back(field(line, "\"L\":"));
                pkt_r.push_back(field(line, "\"R\":"));
            end
        end
    end

    // ---------------- Helpers ----------------

    int  checks = 0;
    real worst_stop_ms = 0.0;

    task automatic pass(input string name);
        checks++;
        $display("PASS [%0d]: %s", checks, name);
    endtask

    task automatic wait_ms(input real ms);
        realtime d;
        d = ms * 1.0ms;
        if (d > 0) #(d);
        @(negedge CLOCK_50);
    endtask

    function automatic int count_pkts(input realtime t0, input realtime t1, input int l, input int r);
        int c;
        c = 0;
        for (int i = 0; i < pkt_start.size(); i++)
            if (pkt_start[i] >= t0 && pkt_start[i] < t1 && pkt_l[i] == l && pkt_r[i] == r) c++;
        return c;
    endfunction

    function automatic int count_all(input realtime t0, input realtime t1);
        int c;
        c = 0;
        for (int i = 0; i < pkt_start.size(); i++)
            if (pkt_start[i] >= t0 && pkt_start[i] < t1) c++;
        return c;
    endfunction

    task automatic check_stop(input string name, input realtime t_stop);
        realtime t_zero_end;
        bit found;
        found = 0;
        t_zero_end = 0;
        for (int i = 0; i < pkt_start.size(); i++) begin
            if (pkt_start[i] > t_stop + 200ns) begin
                if (pkt_l[i] != 0 || pkt_r[i] != 0)
                    $fatal(1, "FAIL %s: nonzero command L=%0d R=%0d started after the stop",
                           name, pkt_l[i], pkt_r[i]);
                if (!found) begin found = 1; t_zero_end = pkt_end[i]; end
            end
        end
        if (!found) $fatal(1, "FAIL %s: no zero-speed packet after the stop", name);
        if ((t_zero_end - t_stop) / 1.0ms > MAX_STOP_MS)
            $fatal(1, "FAIL %s: zero packet %0.3f ms after the stop", name, (t_zero_end - t_stop) / 1.0ms);
        if ((t_zero_end - t_stop) / 1.0ms > worst_stop_ms) worst_stop_ms = (t_zero_end - t_stop) / 1.0ms;
        pass($sformatf("%s: zero-speed packet complete %0.3f ms after the stop", name,
                       (t_zero_end - t_stop) / 1.0ms));
    endtask

    task automatic request_reverse(input int pwm);
        @(negedge CLOCK_50);
        force dut.nav_rev_left_pwm  = 16'(-pwm);
        force dut.nav_rev_right_pwm = 16'(-pwm);
        force dut.nav_rev_request   = 1'b1;
    endtask

    task automatic withdraw_reverse();
        @(negedge CLOCK_50);
        force dut.nav_rev_request   = 1'b0;
        force dut.nav_rev_left_pwm  = 16'sd0;
        force dut.nav_rev_right_pwm = 16'sd0;
    endtask

    // ---------------- Scenario ----------------

    realtime t0, t_stop;
    realtime last_rev_start;

    initial begin
        // Docked: forward navigation idle, 240 mm, forward stop active.
        force dut.arm                 = 1'b1;
        force dut.motion_permitted    = 1'b1;
        force dut.emergency_stop      = 1'b0;
        force dut.camera_target_fresh = 1'b0;
        force dut.sonar_valid         = 1'b1;
        force dut.distance_mm         = 16'd240;
        force dut.sonar_forward_clear = 1'b0;
        force dut.motor_enable        = 1'b0;
        force dut.left_pwm            = 16'sd0;
        force dut.right_pwm           = 16'sd0;
        withdraw_reverse();

        wait_ms(0.1);
        KEY[0] = 1'b1;
        wait_ms(25);

        // ---- 1. SW3 off ----
        t0 = $realtime;
        request_reverse(50);
        wait_ms(40);
        if (count_all(t0, $realtime) < 3 || count_all(t0, $realtime) != count_pkts(t0, $realtime, 0, 0))
            $fatal(1, "FAIL: reverse with SW3 off produced nonzero packets");
        if (dut.reverse_status != 3'd3)
            $fatal(1, "FAIL: status %0d, expected NOT_ENABLED", dut.reverse_status);
        pass("SW3 off: reverse request refused, zero packets only (NOT_ENABLED)");
        withdraw_reverse();

        // ---- 2. SW3 on: supervised reverse from the dock ----
        SW[3] = 1'b1;
        wait_ms(1);
        t0 = $realtime;
        request_reverse(50);
        wait_ms(40);
        if (count_pkts(t0, $realtime, -50, -50) < 3)
            $fatal(1, "FAIL: reverse packets not received through the top level");
        if (!dut.reverse_active)
            $fatal(1, "FAIL: reverse_active not set");
        pass("SW3 on, docked at 240 mm: {\"T\":11,\"L\":-50,\"R\":-50} reaches the chassis pin");

        // ---- 3a. IR stop during reverse ----
        t_stop = $realtime;
        force dut.emergency_stop   = 1'b1;
        force dut.motion_permitted = 1'b0;
        wait_ms(15);
        check_stop("IR stop during reverse (top level)", t_stop);
        force dut.emergency_stop   = 1'b0;
        force dut.motion_permitted = 1'b1;
        t0 = $realtime;
        wait_ms(25);
        if (count_all(t0, $realtime) != count_pkts(t0, $realtime, 0, 0))
            $fatal(1, "FAIL: reverse resumed after IR release without a new request");
        pass("IR released: no reverse until a new request");
        withdraw_reverse();

        // ---- 3b. SW0 disarm during reverse ----
        request_reverse(50);
        wait_ms(33.3);
        t_stop = $realtime;
        force dut.arm = 1'b0;
        wait_ms(15);
        check_stop("SW0 disarm during reverse (top level)", t_stop);
        force dut.arm = 1'b1;
        withdraw_reverse();

        // ---- 3c. Operator switch SW3 off during reverse ----
        request_reverse(50);
        wait_ms(37.7);
        t_stop = $realtime;
        SW[3] = 1'b0;
        wait_ms(15);
        check_stop("SW3 switched off during reverse (top level)", t_stop);
        withdraw_reverse();

        // ---- 4. reverse -> forward needs the 100 ms dwell ----
        SW[3] = 1'b1;
        wait_ms(1);
        request_reverse(50);
        wait_ms(25);
        last_rev_start = $realtime;
        withdraw_reverse();
        force dut.camera_target_fresh = 1'b1;
        force dut.distance_mm         = 16'd1000;
        force dut.sonar_forward_clear = 1'b1;
        force dut.motor_enable        = 1'b1;
        force dut.left_pwm            = 16'sd80;
        force dut.right_pwm           = 16'sd80;
        t0 = $realtime;
        wait_ms(140);
        begin
            realtime first_fwd;
            first_fwd = 0;
            for (int i = 0; i < pkt_start.size(); i++)
                if (pkt_start[i] >= t0 && pkt_l[i] == 80 && pkt_r[i] == 80) begin
                    first_fwd = pkt_start[i];
                    break;
                end
            if (first_fwd == 0)
                $fatal(1, "FAIL: forward did not resume after the dwell");
            if ((first_fwd - t0) / 1.0ms < 99.0)
                $fatal(1, "FAIL: forward resumed %0.1f ms after reverse (< 100 ms dwell)", (first_fwd - t0) / 1.0ms);
            if (count_pkts(t0 + 12ms, first_fwd, 0, 0) < 8)
                $fatal(1, "FAIL: expected zero packets throughout the dwell");
            pass($sformatf("reverse -> forward: %0d zero packets, forward 80/80 after %0.1f ms",
                           count_pkts(t0, first_fwd, 0, 0), (first_fwd - t0) / 1.0ms));
        end

        $display("---------------------------------------");
        $display("PASS: top_reverse_safety_tb (%0d checks)", checks);
        $display("Worst stop -> zero packet complete: %0.3f ms", worst_stop_ms);
        $display("---------------------------------------");
        $finish;
    end

    initial begin
        #2s;
        $fatal(1, "TIMEOUT: top_reverse_safety_tb");
    end

endmodule

`default_nettype wire
