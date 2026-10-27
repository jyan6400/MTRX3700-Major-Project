`timescale 1ns/1ps
`default_nettype none
/* verilator lint_off ZERODLY */   // wait_ms uses a run-time delay (testbench only)

// ============================================================================
// MTRX3700 Major Project - Group 26 - V3
// motion_supervisor -> chassis_drive (Ed) -> UGV02 model (Ed), at the real
// 50 MHz clock, 115200 baud and 10 ms command schedule (owner: Advay, M5).
//
// Every UART line is decoded here into its L and R values, with its start
// and end time, and checked:
//   1. forward -> dock -> supervised timed reverse -> stop, decoded values
//      and the chassis model actually moving backwards;
//   2. IR stop during reverse at five phases of the 10 ms schedule: no
//      nonzero command starts after the stop, and the first zero-speed packet
//      completes within one UART period + one packet;
//   3. SW0 disarm and invalid sonar during reverse: the same;
//   4. reverse timeout: commanded reverse is cut after REV_MAX_MS, then zero
//      packets;
//   5. rapid forward/reverse toggling: every change of direction in the
//      packet stream has a zero-speed packet in between;
//   6. unauthorised reverse (negative forward demand): only zero packets.
// The chassis model must report 0 protocol problems.
//
// Supervisor parameters shortened for simulation time: REV_MAX_MS = 100,
// DIR_DWELL_MS = 25 (still > one 10 ms UART period). Clock and UART timing
// are the production values.
// ============================================================================

module motion_supervisor_uart_tb;

    localparam int    REV_MS      = 100;
    localparam int    DWELL_MS    = 25;
    localparam int    BIT_NS      = 8680;            // 434 clocks at 50 MHz
    localparam real   MAX_STOP_MS = 12.0;            // 10 ms period + one ~1.9 ms packet

    logic clk = 1'b0;
    always #10 clk = ~clk;                           // 50 MHz

    logic               rst_n = 1'b0;
    logic               arm = 1'b0;
    logic               emergency_stop = 1'b0;
    logic               motion_permitted = 1'b1;
    logic               camera_target_fresh = 1'b0;
    logic               sonar_valid = 1'b0;
    logic               sonar_forward_clear = 1'b0;
    logic        [15:0] distance_mm = 16'd0;
    logic               fwd_enable = 1'b0;
    logic signed [15:0] fwd_left_pwm = 16'sd0;
    logic signed [15:0] fwd_right_pwm = 16'sd0;
    logic               rev_request = 1'b0;
    logic signed [15:0] rev_left_pwm = 16'sd0;
    logic signed [15:0] rev_right_pwm = 16'sd0;
    logic               reverse_enable = 1'b0;

    wire                drive_enable;
    wire signed [9:0]   left_cmd;
    wire signed [9:0]   right_cmd;
    wire [2:0]          mode;
    wire                stop_active;
    wire                reverse_permitted;
    wire                reverse_active;
    wire                reverse_timeout;
    wire                reverse_fault;
    wire [2:0]          reverse_status;
    wire [15:0]         reverse_ms;

    motion_supervisor #(
        .CLK_HZ(50_000_000),
        .REV_MAX_PWM(60),
        .REV_MAX_MS(REV_MS),
        .DIR_DWELL_MS(DWELL_MS)
    ) u_sup (
        .clk(clk), .rst_n(rst_n),
        .arm(arm), .emergency_stop(emergency_stop), .motion_permitted(motion_permitted),
        .camera_target_fresh(camera_target_fresh), .sonar_valid(sonar_valid),
        .sonar_forward_clear(sonar_forward_clear), .distance_mm(distance_mm),
        .fwd_enable(fwd_enable), .fwd_left_pwm(fwd_left_pwm), .fwd_right_pwm(fwd_right_pwm),
        .rev_request(rev_request), .rev_left_pwm(rev_left_pwm), .rev_right_pwm(rev_right_pwm),
        .reverse_enable(reverse_enable),
        .drive_enable(drive_enable), .left_cmd(left_cmd), .right_cmd(right_cmd),
        .mode(mode), .stop_active(stop_active),
        .reverse_permitted(reverse_permitted), .reverse_active(reverse_active),
        .reverse_timeout(reverse_timeout), .reverse_fault(reverse_fault),
        .reverse_status(reverse_status), .reverse_ms(reverse_ms)
    );

    wire uart_tx;
    wire chassis_busy;

    chassis_drive #(
        .CLK_HZ(50_000_000),
        .BAUD(115_200),
        .PERIOD_MS(10)
    ) u_chassis (
        .clk(clk),
        .reset(!rst_n),
        .enable(drive_enable),
        .left(left_cmd),
        .right(right_cmd),
        .tx(uart_tx),
        .busy(chassis_busy)
    );

    real robot_x, robot_y, robot_heading, v_l, v_r, c_l, c_r;
    int  model_commands, model_problems;

    ugv02_model #(
        .NAME("UGV02"),
        .VERBOSE(0)
    ) u_robot (
        .rx(uart_tx), .tx(),
        .x_m(robot_x), .y_m(robot_y), .heading_rad(robot_heading),
        .v_left(v_l), .v_right(v_r), .cmd_left(c_l), .cmd_right(c_r),
        .commands(model_commands), .problems(model_problems)
    );

    // ------------------------------------------------------------------
    // UART decoder: every line -> (start time, end time, L, R)
    // ------------------------------------------------------------------

    realtime pkt_start[$];
    realtime pkt_end[$];
    int      pkt_l[$];
    int      pkt_r[$];

    function automatic int find_str(input string s, input string key);
        for (int i = 0; i + key.len() <= s.len(); i++)
            if (s.substr(i, i + key.len() - 1) == key)
                return i;
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
        realtime t_byte;
        realtime t_line;
        line = "";
        t_line = 0;
        forever begin
            @(negedge uart_tx);
            t_byte = $realtime;
            ch = 8'd0;
            #(BIT_NS / 2);
            if (uart_tx !== 1'b0) $fatal(1, "FAIL: bad UART start bit at %0t", $time);
            for (int i = 0; i < 8; i++) begin
                #BIT_NS;
                ch[i] = uart_tx;
            end
            #BIT_NS;
            if (uart_tx !== 1'b1) $fatal(1, "FAIL: bad UART stop bit at %0t", $time);
            if (ch == 8'h7B) begin line = ""; t_line = t_byte; end
            if (ch != 8'h0A && ch != 8'h0D) line = {line, $sformatf("%c", ch)};
            if (ch == 8'h7D) begin
                if (find_str(line, "{\"T\":11,\"L\":") != 0)
                    $fatal(1, "FAIL: unexpected UART line '%s'", line);
                pkt_start.push_back(t_line);
                pkt_end.push_back($realtime);
                pkt_l.push_back(field(line, "\"L\":"));
                pkt_r.push_back(field(line, "\"R\":"));
            end
        end
    end

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

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
        @(negedge clk);
    endtask

    // After a stop event at t_stop: no nonzero command may START after it,
    // and the first zero-speed packet must complete within MAX_STOP_MS.
    task automatic check_stop(input string name, input realtime t_stop);
        realtime t_zero_end;
        bit      found;
        found = 0;
        t_zero_end = 0;
        for (int i = 0; i < pkt_start.size(); i++) begin
            if (pkt_start[i] > t_stop + 200ns) begin
                if (pkt_l[i] != 0 || pkt_r[i] != 0)
                    $fatal(1, "FAIL %s: nonzero command L=%0d R=%0d started %0.3f ms after the stop",
                           name, pkt_l[i], pkt_r[i], (pkt_start[i] - t_stop) / 1.0ms);
                if (!found) begin
                    found = 1;
                    t_zero_end = pkt_end[i];
                end
            end
        end
        if (!found)
            $fatal(1, "FAIL %s: no zero-speed packet after the stop", name);
        if ((t_zero_end - t_stop) / 1.0ms > MAX_STOP_MS)
            $fatal(1, "FAIL %s: zero packet completed %0.3f ms after the stop (> %0.1f ms)",
                   name, (t_zero_end - t_stop) / 1.0ms, MAX_STOP_MS);
        if ((t_zero_end - t_stop) / 1.0ms > worst_stop_ms)
            worst_stop_ms = (t_zero_end - t_stop) / 1.0ms;
        pass($sformatf("%s: zero-speed packet complete %0.3f ms after the stop, no stale nonzero command",
                       name, (t_zero_end - t_stop) / 1.0ms));
    endtask

    // Packets started in [t0, t1): count those equal to (l, r).
    function automatic int count_pkts(input realtime t0, input realtime t1, input int l, input int r);
        int c;
        c = 0;
        for (int i = 0; i < pkt_start.size(); i++)
            if (pkt_start[i] >= t0 && pkt_start[i] < t1 && pkt_l[i] == l && pkt_r[i] == r)
                c++;
        return c;
    endfunction

    function automatic int count_all(input realtime t0, input realtime t1);
        int c;
        c = 0;
        for (int i = 0; i < pkt_start.size(); i++)
            if (pkt_start[i] >= t0 && pkt_start[i] < t1)
                c++;
        return c;
    endfunction

    task automatic healthy_forward(input int pwm);
        @(negedge clk);
        arm = 1; emergency_stop = 0; motion_permitted = 1;
        camera_target_fresh = 1; sonar_valid = 1; sonar_forward_clear = 1; distance_mm = 16'd800;
        rev_request = 0; rev_left_pwm = 0; rev_right_pwm = 0;
        fwd_enable = 1; fwd_left_pwm = 16'(pwm); fwd_right_pwm = 16'(pwm);
    endtask

    task automatic docked();
        @(negedge clk);
        fwd_enable = 0; fwd_left_pwm = 0; fwd_right_pwm = 0;
        camera_target_fresh = 0; sonar_valid = 1; sonar_forward_clear = 0; distance_mm = 16'd240;
    endtask

    task automatic start_reverse(input int pwm);
        @(negedge clk);
        rev_left_pwm = 16'(-pwm); rev_right_pwm = 16'(-pwm);
        rev_request = 1;
    endtask

    task automatic end_reverse();
        @(negedge clk);
        rev_request = 0; rev_left_pwm = 0; rev_right_pwm = 0;
    endtask

    // ------------------------------------------------------------------
    // Reverse duration monitor (supervisor output, clock-exact)
    // ------------------------------------------------------------------

    realtime rev_on_t = 0;
    realtime rev_off_t = 0;
    bit      rev_was = 0;

    always @(posedge clk) begin
        if ((left_cmd < 0) && !rev_was) rev_on_t = $realtime;
        if (!(left_cmd < 0) && rev_was) rev_off_t = $realtime;
        rev_was = (left_cmd < 0);
    end

    // ------------------------------------------------------------------
    // Scenario
    // ------------------------------------------------------------------

    realtime t0, t1, t_stop;
    real     x_before;
    int      n_sign_changes;

    initial begin
        if ($test$plusargs("wave")) begin
            $dumpfile("motion_supervisor_uart.fst");
            $dumpvars(0, motion_supervisor_uart_tb);
        end

        wait_ms(0.1);
        rst_n = 1;
        @(negedge clk);
        reverse_enable = 1;                          // operator: rear corridor cleared

        // ---- 1. forward -> dock -> supervised reverse -> stop ----
        t0 = $realtime;
        healthy_forward(80);
        wait_ms(40);
        if (count_pkts(t0, $realtime, 80, 80) < 3)
            $fatal(1, "FAIL: forward 80/80 packets not received");
        pass("forward: {\"T\":11,\"L\":80,\"R\":80} packets every 10 ms");

        docked();
        t1 = $realtime;
        wait_ms(40);
        if (count_pkts(t1 + 12ms, $realtime, 0, 0) < 2 || count_pkts(t1 + 12ms, $realtime, 80, 80) != 0)
            $fatal(1, "FAIL: docked: expected only zero packets");
        pass("docked at 240 mm: zero-speed packets only");

        u_robot.update();
        x_before = v_l;                              // left wheel speed before reversing
        t0 = $realtime;
        start_reverse(50);
        for (int i = 0; i < 6; i++) begin           // range grows as it backs away
            wait_ms(10);
            distance_mm = distance_mm + 16'd8;
        end
        u_robot.update();
        if (count_pkts(t0, $realtime, -50, -50) < 4)
            $fatal(1, "FAIL: reverse -50/-50 packets not received");
        // The model's wheels lag (TAU 0.15 s), so check what it was told and
        // that the wheels are slowing towards reverse, not net displacement.
        if (!(c_l < 0.0 && c_r < 0.0 && v_l < x_before - 0.02))
            $fatal(1, "FAIL: chassis model not driven backwards (cmd %0.3f/%0.3f m/s, wheel %0.3f -> %0.3f m/s)",
                   c_l, c_r, x_before, v_l);
        pass($sformatf("supervised reverse: -50/-50 packets; model wheels commanded %0.3f m/s, slowing %0.3f -> %0.3f m/s",
                       c_l, x_before, v_l));
        t_stop = $realtime;
        end_reverse();
        wait_ms(20);
        check_stop("navigation ends the retreat", t_stop);
        wait_ms(DWELL_MS + 5);

        // ---- 2. IR stop during reverse, five phases of the UART schedule ----
        for (int k = 0; k < 5; k++) begin
            docked();
            start_reverse(50);
            wait_ms(20.0 + 2.1 * k);
            t_stop = $realtime;
            emergency_stop = 1; motion_permitted = 0;
            wait_ms(15);
            check_stop($sformatf("IR stop during reverse (phase %0d)", k), t_stop);
            if (reverse_status != 3'd2)
                $fatal(1, "FAIL: status after IR stop = %0d, expected STOPPED", reverse_status);
            emergency_stop = 0; motion_permitted = 1;
            t1 = $realtime;
            wait_ms(15);
            if (count_all(t1 + 200ns, $realtime) != count_pkts(t1 + 200ns, $realtime, 0, 0))
                $fatal(1, "FAIL: reverse resumed after IR release without a new request");
            end_reverse();
            wait_ms(1);
        end
        pass("after every IR release: no reverse until a new request");

        // ---- 3. SW0 disarm and invalid sonar during reverse ----
        docked();
        start_reverse(50);
        wait_ms(23.3);
        t_stop = $realtime;
        arm = 0;
        wait_ms(15);
        check_stop("SW0 disarm during reverse", t_stop);
        arm = 1;
        end_reverse();
        wait_ms(1);

        docked();
        start_reverse(50);
        wait_ms(27.7);
        t_stop = $realtime;
        sonar_valid = 0;
        wait_ms(15);
        check_stop("sonar invalid/stale during reverse", t_stop);
        if (reverse_status != 3'd4)
            $fatal(1, "FAIL: status after sonar loss = %0d, expected SONAR", reverse_status);
        sonar_valid = 1;
        end_reverse();
        wait_ms(1);

        // ---- 4. Timeout: request held, reverse cut after REV_MAX_MS ----
        docked();
        start_reverse(50);
        wait_ms(REV_MS + 30);
        if (!reverse_timeout)
            $fatal(1, "FAIL: reverse_timeout not raised");
        if (((rev_off_t - rev_on_t) / 1.0ms) < REV_MS - 0.001 || ((rev_off_t - rev_on_t) / 1.0ms) > REV_MS + 0.001)
            $fatal(1, "FAIL: reverse lasted %0.4f ms, expected %0d ms", (rev_off_t - rev_on_t) / 1.0ms, REV_MS);
        pass($sformatf("timeout: commanded reverse lasted %0.4f ms (limit %0d ms)",
                       (rev_off_t - rev_on_t) / 1.0ms, REV_MS));
        check_stop("reverse timeout", rev_off_t);
        end_reverse();
        wait_ms(1);

        // ---- 5. Rapid direction changes ----
        t0 = $realtime;
        for (int i = 0; i < 40; i++) begin
            if (i % 2 == 0) healthy_forward(70);
            else begin
                @(negedge clk);
                fwd_enable = 0; fwd_left_pwm = 0; fwd_right_pwm = 0;
                start_reverse(40);
            end
            wait_ms(3.0 + 1.7 * (i % 7) + ((i % 5 == 0) ? 30.0 : 0.0));
            if (i % 2 != 0) end_reverse();
        end
        @(negedge clk);
        fwd_enable = 0;
        wait_ms(20);
        n_sign_changes = 0;
        begin
            int last_sign;
            last_sign = 0;
            for (int i = 0; i < pkt_start.size(); i++) begin
                int s;
                if (pkt_start[i] < t0) continue;
                s = (pkt_l[i] > 0 || pkt_r[i] > 0) ? 1 : ((pkt_l[i] < 0 || pkt_r[i] < 0) ? -1 : 0);
                if ((pkt_l[i] > 0 && pkt_r[i] < 0) || (pkt_l[i] < 0 && pkt_r[i] > 0))
                    $fatal(1, "FAIL: mixed-direction packet L=%0d R=%0d", pkt_l[i], pkt_r[i]);
                if (s != 0 && last_sign != 0 && s != last_sign)
                    $fatal(1, "FAIL: packet %0d changes direction without a zero packet between", i);
                if (s != 0 && last_sign == 0) n_sign_changes++;
                last_sign = s;
            end
        end
        if (count_pkts(t0, $realtime, -40, -40) == 0 || count_pkts(t0, $realtime, 70, 70) == 0)
            $fatal(1, "FAIL: rapid toggling did not exercise both directions");
        pass($sformatf("rapid toggling: %0d packets, every direction change has a zero packet between",
                       count_all(t0, $realtime)));

        // ---- 6. Unauthorised reverse on the forward channel ----
        healthy_forward(80);
        wait_ms(DWELL_MS + 15);
        @(negedge clk);
        fwd_left_pwm = -16'sd40; fwd_right_pwm = -16'sd40;
        t_stop = $realtime;
        wait_ms(30);
        check_stop("unauthorised negative forward demand", t_stop);
        if (reverse_status != 3'd7)
            $fatal(1, "FAIL: status = %0d, expected UNAUTHORISED", reverse_status);
        @(negedge clk);
        fwd_enable = 0;
        wait_ms(15);

        // ---- Summary ----
        u_robot.update();
        if (model_problems != 0)
            $fatal(1, "FAIL: UGV02 model reported %0d problems", model_problems);
        pass($sformatf("UGV02 model: %0d commands accepted, 0 protocol problems", model_commands));
        pass($sformatf("worst stop-to-zero-packet latency: %0.3f ms (bound %0.1f ms)",
                       worst_stop_ms, MAX_STOP_MS));

        $display("---------------------------------------");
        $display("PASS: motion_supervisor_uart_tb (%0d checks)", checks);
        $display("UART packets decoded: %0d", pkt_start.size());
        $display("Worst stop -> zero packet complete: %0.3f ms", worst_stop_ms);
        $display("---------------------------------------");
        $finish;
    end

    initial begin
        #3s;
        $fatal(1, "TIMEOUT: motion_supervisor_uart_tb");
    end

endmodule

`default_nettype wire
