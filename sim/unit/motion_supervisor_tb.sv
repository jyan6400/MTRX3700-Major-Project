`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// MTRX3700 Major Project - Group 26 - V3
// Unit test for motion_supervisor.sv (owner: Advay Hassan, M5)
//
// Part A  directed: V2 forward rule, stop priority in every direction,
//         unauthorised and illegal reverse, operator enable, speed clamp,
//         exact timer boundary, pause, IR/disarm/reset during reverse,
//         invalid/stale sonar, wrong-way, direction-change dwell, re-arm.
// Part B  random stress with no reverse: outputs equal the V2 gate exactly.
// Part C  random stress with reverse requests, stops and direction changes.
//
// Safety invariants checked on EVERY clock of all three parts:
//   I1 stop inputs (arm=0, IR stop, !motion_permitted) => zero, enable 0
//   I2 reverse output never beyond -REV_MAX_PWM; forward never above 255;
//      never one wheel forward and the other reverse
//   I3 reverse output only with rev_request, operator enable and valid sonar
//   I4 forward output only with the full V2 permission and no rev_request
//   I5 every forward<->reverse change is separated by >= DWELL clocks of zero
//   I6 no session commands more than REV_MAX_MS of reverse
//
// Fast parameters: CLK_HZ = 100 kHz (100 clocks per ms), REV_MAX_MS = 20,
// DIR_DWELL_MS = 3. Logic is identical at 50 MHz / 1500 ms / 100 ms.
// Ends with "PASS: motion_supervisor_tb" or stops at the first "FAIL".
// ============================================================================

module motion_supervisor_tb;

    localparam int CLK_HZ     = 100_000;
    localparam int CYC_PER_MS = CLK_HZ / 1000;
    localparam int REV_MS     = 20;
    localparam int DWELL_MS   = 3;
    localparam int REV_PWM    = 60;
    localparam int WRONG_MM   = 60;
    localparam int REV_CYC    = REV_MS * CYC_PER_MS;     // 2000
    localparam int DWELL_CYC  = DWELL_MS * CYC_PER_MS;   // 300

    localparam logic [2:0] MODE_IDLE = 3'd0, MODE_FORWARD = 3'd1, MODE_REVERSE = 3'd2,
                           MODE_DWELL = 3'd3, MODE_STOP = 3'd4;
    localparam logic [2:0] ST_NONE = 3'd0, ST_TIMEOUT = 3'd1, ST_STOPPED = 3'd2,
                           ST_NOT_ENABLED = 3'd3, ST_SONAR = 3'd4, ST_WRONG_WAY = 3'd5,
                           ST_ILLEGAL = 3'd6, ST_UNAUTHORISED = 3'd7;

    logic clk = 1'b0;
    always #5 clk = ~clk;

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

    logic               drive_enable;
    logic signed [9:0]  left_cmd;
    logic signed [9:0]  right_cmd;
    logic [2:0]         mode;
    logic               stop_active;
    logic               reverse_permitted;
    logic               reverse_active;
    logic               reverse_timeout;
    logic               reverse_fault;
    logic [2:0]         reverse_status;
    logic [15:0]        reverse_ms;

    motion_supervisor #(
        .CLK_HZ(CLK_HZ),
        .FWD_MAX_PWM(255),
        .REV_MAX_PWM(REV_PWM),
        .REV_MAX_MS(REV_MS),
        .DIR_DWELL_MS(DWELL_MS),
        .WRONG_WAY_MM(16'(WRONG_MM))
    ) dut (
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

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    int checks = 0;

    task automatic pass(input string name);
        checks++;
        $display("PASS [%0d]: %s", checks, name);
    endtask

    // Advance n clocks; inputs are changed only between edges (after #1).
    task automatic tick(input int n = 1);
        repeat (n) @(posedge clk);
        #1;
    endtask

    task automatic expect_cmd(input string name, input bit en, input int l, input int r);
        #1;
        if (drive_enable !== en || left_cmd !== 10'(l) || right_cmd !== 10'(r))
            $fatal(1, "FAIL %s: enable=%b L=%0d R=%0d, expected %b/%0d/%0d",
                   name, drive_enable, left_cmd, right_cmd, en, l, r);
        pass(name);
    endtask

    task automatic expect_val(input string name, input int actual, input int expected);
        if (actual != expected)
            $fatal(1, "FAIL %s: got %0d, expected %0d", name, actual, expected);
        pass(name);
    endtask

    // A healthy forward approach, as V2 provides it.
    task automatic set_forward_ok(input int pwm = 80);
        arm = 1'b1; emergency_stop = 1'b0; motion_permitted = 1'b1;
        camera_target_fresh = 1'b1; sonar_valid = 1'b1; sonar_forward_clear = 1'b1;
        distance_mm = 16'd800;
        fwd_enable = 1'b1; fwd_left_pwm = 16'(pwm); fwd_right_pwm = 16'(pwm);
    endtask

    task automatic forward_idle();
        fwd_enable = 1'b0; fwd_left_pwm = 16'sd0; fwd_right_pwm = 16'sd0;
    endtask

    // At the dock: 240 mm, forward blocked by V2, reverse wanted.
    task automatic at_dock();
        forward_idle();
        camera_target_fresh = 1'b0;     // target may be out of view when backing off
        sonar_valid = 1'b1; sonar_forward_clear = 1'b0; distance_mm = 16'd240;
    endtask

    task automatic set_rev(input int l, input int r);
        rev_left_pwm = 16'(l); rev_right_pwm = 16'(r);
    endtask

    task automatic drop_request();
        rev_request = 1'b0; set_rev(0, 0);
        tick(2);
    endtask

    // Start a session and wait the 1-clock start latency.
    task automatic start_session(input int l, input int r);
        set_rev(l, r);
        rev_request = 1'b1;
        tick(1);
    endtask

    // ------------------------------------------------------------------
    // Continuous safety invariants (sampled at every rising edge)
    // ------------------------------------------------------------------

    int  inv_cycles = 0;
    int  last_dir = 0;        // 0 none, 1 fwd, 2 rev
    int  zero_run = 0;
    int  session_rev_cyc = 0;
    bit  rev_req_prev = 1'b0;
    int  dir_changes = 0;

    always @(posedge clk) begin
        if (rst_n) begin
            inv_cycles++;

            // I1
            if ((!arm || emergency_stop || !motion_permitted) &&
                (drive_enable || left_cmd != 0 || right_cmd != 0))
                $fatal(1, "FAIL I1 @%0t: motion while a stop input is active", $time);

            // I2
            if (int'(left_cmd) < -REV_PWM || int'(right_cmd) < -REV_PWM ||
                int'(left_cmd) > 255 || int'(right_cmd) > 255)
                $fatal(1, "FAIL I2 @%0t: command out of range L=%0d R=%0d", $time, left_cmd, right_cmd);
            if ((left_cmd > 0 && right_cmd < 0) || (left_cmd < 0 && right_cmd > 0))
                $fatal(1, "FAIL I2 @%0t: mixed-direction command L=%0d R=%0d", $time, left_cmd, right_cmd);

            // I3
            if ((left_cmd < 0 || right_cmd < 0) &&
                !(rev_request && dut.rev_en_sync && sonar_valid))
                $fatal(1, "FAIL I3 @%0t: reverse without authorisation, enable and valid sonar", $time);

            // I4
            if ((left_cmd > 0 || right_cmd > 0) &&
                !(fwd_enable && camera_target_fresh && sonar_valid && sonar_forward_clear && !rev_request))
                $fatal(1, "FAIL I4 @%0t: forward without the V2 permission", $time);

            // I5
            if (left_cmd != 0 || right_cmd != 0) begin
                int d;
                d = (left_cmd < 0 || right_cmd < 0) ? 2 : 1;
                if (last_dir != 0 && d != last_dir) begin
                    dir_changes++;
                    if (zero_run < DWELL_CYC)
                        $fatal(1, "FAIL I5 @%0t: direction change after only %0d zero clocks", $time, zero_run);
                end
                last_dir = d;
                zero_run = 0;
            end else begin
                zero_run++;
            end

            // I6
            if (rev_request && !rev_req_prev)
                session_rev_cyc = 0;
            if (left_cmd < 0 || right_cmd < 0)
                session_rev_cyc++;
            if (session_rev_cyc > REV_CYC)
                $fatal(1, "FAIL I6 @%0t: %0d clocks of reverse in one session", $time, session_rev_cyc);
            rev_req_prev = rev_request;
        end else begin
            last_dir = 0;
            zero_run = 0;
            rev_req_prev = 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // V2 reference gate (the inline top-level logic this module replaces)
    // ------------------------------------------------------------------

    function automatic bit v2_enable();
        return rst_n && arm && motion_permitted && !emergency_stop &&
               camera_target_fresh && sonar_valid && sonar_forward_clear && fwd_enable;
    endfunction

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------

    int n;
    int gap;
    int c0;

    initial begin
        if ($test$plusargs("wave")) begin
            $dumpfile("motion_supervisor.fst");
            $dumpvars(0, motion_supervisor_tb);
        end

        // ======================= Part A: directed =======================

        tick(3);
        expect_cmd("reset: no motion", 0, 0, 0);
        expect_val("reset: mode STOP", int'(mode), int'(MODE_STOP));
        rst_n = 1'b1;
        tick(2);
        expect_cmd("disarmed after reset: no motion", 0, 0, 0);

        // ---- Forward: the V2 rule, unchanged ----
        set_forward_ok(80);
        tick(1);
        expect_cmd("forward 80/80 with full V2 permission", 1, 80, 80);
        expect_val("mode FORWARD", int'(mode), int'(MODE_FORWARD));

        camera_target_fresh = 1'b0; expect_cmd("forward blocked: camera not fresh", 0, 0, 0);
        camera_target_fresh = 1'b1;
        sonar_valid = 1'b0;         expect_cmd("forward blocked: sonar invalid/stale", 0, 0, 0);
        sonar_valid = 1'b1;
        sonar_forward_clear = 1'b0; distance_mm = 16'd250;
                                    expect_cmd("forward blocked: 250 mm stop (forward_clear low)", 0, 0, 0);
        sonar_forward_clear = 1'b1; distance_mm = 16'd800;
        fwd_enable = 1'b0;          expect_cmd("forward blocked: navigation not enabling", 0, 0, 0);
        fwd_enable = 1'b1;
        fwd_left_pwm = 16'sd300; fwd_right_pwm = 16'sd100;
        tick(1);
        expect_cmd("forward PWM saturated at 255", 1, 255, 100);
        fwd_left_pwm = 16'sd80; fwd_right_pwm = 16'sd80;

        // Stop priority while driving forward (combinational: no clock edge).
        arm = 1'b0;               expect_cmd("SW0 disarm stops forward at once", 0, 0, 0);
        expect_val("mode STOP on disarm", int'(mode), int'(MODE_STOP));
        arm = 1'b1;
        emergency_stop = 1'b1; motion_permitted = 1'b0;
                                  expect_cmd("IR stop stops forward at once", 0, 0, 0);
        emergency_stop = 1'b0; motion_permitted = 1'b1;
        tick(1);
        expect_cmd("forward resumes when the stop clears (V2 behaviour)", 1, 80, 80);

        // ---- Unauthorised reverse on the forward channel ----
        fwd_left_pwm = -16'sd50; fwd_right_pwm = -16'sd50;
        expect_cmd("negative forward demand refused (no authorisation)", 0, 0, 0);
        tick(1);
        expect_val("status UNAUTHORISED", int'(reverse_status), int'(ST_UNAUTHORISED));
        expect_val("reverse_fault raised", int'(reverse_fault), 1);
        fwd_left_pwm = 16'sd80; fwd_right_pwm = -16'sd20;
        expect_cmd("spin demand (one wheel negative) refused", 0, 0, 0);
        fwd_left_pwm = 16'sd80; fwd_right_pwm = 16'sd80;
        tick(1);

        // ---- Reverse demand without a request is ignored ----
        forward_idle();
        set_rev(-50, -50);
        tick(5);
        expect_cmd("reverse demand without rev_request ignored", 0, 0, 0);
        set_rev(0, 0);

        // ---- Request with the operator switch off ----
        at_dock();
        tick(DWELL_CYC);
        start_session(-50, -50);
        expect_cmd("reverse refused: operator switch off", 0, 0, 0);
        tick(1);
        expect_val("status NOT_ENABLED", int'(reverse_status), int'(ST_NOT_ENABLED));
        reverse_enable = 1'b1;
        tick(5);
        expect_cmd("switch on later: ended session does not restart", 0, 0, 0);
        drop_request();

        // ---- Normal supervised reverse from the dock ----
        start_session(-50, -50);
        expect_cmd("reverse -50/-50 from the dock (250 mm forward stop exempt)", 1, -50, -50);
        expect_val("mode REVERSE", int'(mode), int'(MODE_REVERSE));
        expect_val("reverse_permitted", int'(reverse_permitted), 1);
        expect_val("reverse_active", int'(reverse_active), 1);
        expect_val("status cleared to NONE at session start", int'(reverse_status), int'(ST_NONE));
        set_rev(-200, -45);
        tick(1);
        expect_cmd("reverse speed clamped to -REV_MAX_PWM per wheel", 1, -60, -45);
        set_rev(-50, -50);
        tick(5 * CYC_PER_MS);
        expect_val("reverse_ms counts commanded reverse time", int'(reverse_ms), 5);
        drop_request();
        expect_cmd("request withdrawn: zero at once", 0, 0, 0);

        // ---- Exact timer boundary, with a pause in the middle ----
        n = 0;
        start_session(-50, -50);
        repeat (500) begin if (left_cmd < 0) n++; tick(1); end
        set_rev(0, 0);                          // paused: no reverse time used
        tick(1000);
        expect_val("pause does not use reverse time", int'(reverse_ms), 5);
        set_rev(-50, -50);
        #1;
        while (left_cmd < 0 || n == 500) begin
            if (left_cmd < 0) n++;
            tick(1);
            if (n > REV_CYC + 10) break;
        end
        expect_val("reverse cut after exactly REV_MAX_MS of commanded reverse", n, REV_CYC);
        expect_cmd("expiry produces zero output", 0, 0, 0);
        tick(1);
        expect_val("status TIMEOUT", int'(reverse_status), int'(ST_TIMEOUT));
        expect_val("reverse_timeout raised", int'(reverse_timeout), 1);
        expect_val("reverse_ms = REV_MAX_MS", int'(reverse_ms), REV_MS);
        tick(200);
        expect_cmd("request held high after timeout: stays stopped", 0, 0, 0);
        drop_request();
        start_session(-50, -50);
        expect_cmd("new rising edge: new session reverses again", 1, -50, -50);
        expect_val("new session clears timeout", int'(reverse_timeout), 0);
        expect_val("new session restarts reverse_ms", int'(reverse_ms), 0);

        // ---- IR stop during reverse ----
        tick(50);
        emergency_stop = 1'b1; motion_permitted = 1'b0;
        expect_cmd("IR stop during reverse: zero at once (no clock edge)", 0, 0, 0);
        tick(1);
        expect_val("status STOPPED", int'(reverse_status), int'(ST_STOPPED));
        emergency_stop = 1'b0; motion_permitted = 1'b1;
        tick(10);
        expect_cmd("IR released: reverse does NOT resume without a new request", 0, 0, 0);
        drop_request();

        // ---- Disarm during reverse ----
        start_session(-50, -50);
        tick(50);
        arm = 1'b0;
        expect_cmd("SW0 disarm during reverse: zero at once", 0, 0, 0);
        tick(1);
        expect_val("status STOPPED after disarm", int'(reverse_status), int'(ST_STOPPED));
        arm = 1'b1;
        tick(10);
        expect_cmd("re-armed: no reverse without a new request", 0, 0, 0);
        drop_request();

        // ---- Request raised while stopped ----
        emergency_stop = 1'b1; motion_permitted = 1'b0;
        start_session(-50, -50);
        tick(2);
        emergency_stop = 1'b0; motion_permitted = 1'b1;
        tick(5);
        expect_cmd("request raised during IR stop never reverses", 0, 0, 0);
        expect_val("status STOPPED", int'(reverse_status), int'(ST_STOPPED));
        drop_request();

        // ---- Reset during reverse ----
        start_session(-50, -50);
        tick(20);
        rst_n = 1'b0;
        expect_cmd("reset during reverse: zero at once", 0, 0, 0);
        tick(2);
        rst_n = 1'b1;
        tick(5);
        expect_cmd("after reset: no reverse without a new request", 0, 0, 0);
        expect_val("reset clears status", int'(reverse_status), int'(ST_NONE));
        drop_request();
        reverse_enable = 1'b1;
        tick(3);

        // ---- Invalid / stale sonar during reverse ----
        start_session(-50, -50);
        tick(20);
        sonar_valid = 1'b0;
        expect_cmd("sonar invalid/stale during reverse: zero at once", 0, 0, 0);
        tick(1);
        expect_val("status SONAR", int'(reverse_status), int'(ST_SONAR));
        sonar_valid = 1'b1;
        tick(5);
        expect_cmd("sonar valid again: no automatic resume", 0, 0, 0);
        drop_request();
        sonar_valid = 1'b0;
        start_session(-50, -50);
        expect_cmd("request with sonar already invalid: no reverse", 0, 0, 0);
        sonar_valid = 1'b1;
        drop_request();

        // ---- Wrong-way plausibility (front range should grow) ----
        distance_mm = 16'd300;
        start_session(-50, -50);
        distance_mm = 16'd400;  tick(2);   // moving away: peak 400
        distance_mm = 16'd340;  tick(2);   // 340 + 60 = 400: not below the peak
        expect_cmd("range 60 mm below peak: still reversing", 1, -50, -50);
        distance_mm = 16'd339;
        expect_cmd("range 61 mm below peak: wrong way, zero at once", 0, 0, 0);
        tick(1);
        expect_val("status WRONG_WAY", int'(reverse_status), int'(ST_WRONG_WAY));
        drop_request();
        distance_mm = 16'd240;

        // ---- Illegal demands during a session ----
        start_session(-50, 10);
        expect_cmd("positive wheel in the reverse channel: illegal, zero", 0, 0, 0);
        tick(1);
        expect_val("status ILLEGAL (positive reverse wheel)", int'(reverse_status), int'(ST_ILLEGAL));
        drop_request();
        start_session(-50, -50);
        fwd_enable = 1'b1; fwd_left_pwm = 16'sd80; fwd_right_pwm = 16'sd80;
        camera_target_fresh = 1'b1; sonar_forward_clear = 1'b1; distance_mm = 16'd800;
        expect_cmd("forward demand during reverse session: illegal, zero", 0, 0, 0);
        tick(1);
        expect_val("status ILLEGAL (forward during reverse)", int'(reverse_status), int'(ST_ILLEGAL));
        tick(5);
        expect_cmd("forward refused while rev_request is still high", 0, 0, 0);
        drop_request();

        // ---- Direction-change dwell: forward -> reverse ----
        set_forward_ok(80);
        tick(DWELL_CYC + 5);
        expect_cmd("forward again (after a full dwell)", 1, 80, 80);
        tick(10);
        at_dock();
        c0 = 0;
        set_rev(-50, -50);
        rev_request = 1'b1;
        gap = 0;
        tick(1);
        expect_val("mode DWELL while waiting to reverse", int'(mode), int'(MODE_DWELL));
        while (left_cmd == 0 && gap < DWELL_CYC + 50) begin gap++; tick(1); end
        if (gap < DWELL_CYC - 1 || gap > DWELL_CYC + 1)
            $fatal(1, "FAIL: forward->reverse zero dwell %0d clocks, expected about %0d", gap, DWELL_CYC);
        pass($sformatf("forward->reverse: %0d zero clocks before reversing (dwell %0d)", gap + 1, DWELL_CYC));
        expect_cmd("reverse after the dwell", 1, -50, -50);
        tick(20);                               // reverse for 20 clocks

        // ---- reverse -> forward ----
        rev_request = 1'b0; set_rev(0, 0);
        set_forward_ok(80);
        #1;
        expect_cmd("reverse withdrawn: zero at once", 0, 0, 0);
        gap = 1;
        tick(1);                                // session closes at this edge
        expect_val("mode DWELL while waiting to drive forward", int'(mode), int'(MODE_DWELL));
        while (left_cmd == 0 && gap < DWELL_CYC + 50) begin gap++; tick(1); end
        if (gap < DWELL_CYC - 1 || gap > DWELL_CYC + 1)
            $fatal(1, "FAIL: reverse->forward zero dwell %0d clocks, expected about %0d", gap, DWELL_CYC);
        pass($sformatf("reverse->forward: %0d zero clocks before driving forward (dwell %0d)", gap, DWELL_CYC));
        expect_cmd("forward after the dwell", 1, 80, 80);

        // ---- Same direction resumes without a dwell ----
        fwd_enable = 1'b0;
        tick(10);
        fwd_enable = 1'b1;
        #1;
        expect_cmd("forward pause and resume: no dwell needed", 1, 80, 80);

        // ======================= Part B: random, no reverse =======================
        // Outputs must equal the V2 inline gate exactly on every clock.
        void'($urandom(32'd33));
        for (int i = 0; i < 100_000; i++) begin
            arm                 = ($urandom_range(0, 99) < 95);
            emergency_stop      = ($urandom_range(0, 99) < 3);
            motion_permitted    = !emergency_stop;
            camera_target_fresh = ($urandom_range(0, 99) < 85);
            sonar_valid         = ($urandom_range(0, 99) < 90);
            sonar_forward_clear = sonar_valid && ($urandom_range(0, 99) < 85);
            distance_mm         = 16'($urandom_range(20, 3500));
            fwd_enable          = ($urandom_range(0, 99) < 80);
            fwd_left_pwm        = 16'($urandom_range(0, 255));
            fwd_right_pwm       = 16'($urandom_range(0, 255));
            rev_request         = 1'b0;
            set_rev(int'($urandom_range(0, 80)) - 80, int'($urandom_range(0, 80)) - 80);
            #1;
            if (drive_enable !== v2_enable() ||
                left_cmd  !== (v2_enable() ? fwd_left_pwm[9:0]  : 10'sd0) ||
                right_cmd !== (v2_enable() ? fwd_right_pwm[9:0] : 10'sd0))
                $fatal(1, "FAIL random V2 equivalence at step %0d: en %b L %0d R %0d vs V2 %b",
                       i, drive_enable, left_cmd, right_cmd, v2_enable());
            tick(1);
        end
        pass("random 100000 clocks without reverse: identical to the V2 gate");

        // ======================= Part C: random with reverse =======================
        set_forward_ok(80);
        reverse_enable = 1'b1;
        for (int i = 0; i < 200_000; i++) begin
            r_step();
            tick(1);
        end
        pass($sformatf("random 200000 clocks with reverse: invariants held, %0d direction changes", dir_changes));

        expect_val("invariants checked on every clock", (inv_cycles > 300_000) ? 1 : 0, 1);
        pass($sformatf("I1-I6 checked on %0d clocks", inv_cycles));

        $display("-----------------------------------------");
        $display("PASS: motion_supervisor_tb (%0d checks)", checks);
        $display("-----------------------------------------");
        $finish;
    end

    // One random step of Part C: mostly hold, sometimes change one thing.
    task automatic r_step();
        int r;
        r = int'($urandom_range(0, 999));
        if (r < 4)        begin emergency_stop = 1'b1; motion_permitted = 1'b0; end
        else if (r < 30)  begin emergency_stop = 1'b0; motion_permitted = 1'b1; end
        else if (r < 33)  arm = 1'b0;
        else if (r < 60)  arm = 1'b1;
        else if (r < 70)  rev_request = ~rev_request;
        else if (r < 80)  set_rev(-int'($urandom_range(0, 120)), -int'($urandom_range(0, 120)));
        else if (r < 82)  set_rev(int'($urandom_range(0, 40)), -int'($urandom_range(0, 40)));
        else if (r < 92)  begin fwd_enable = ~fwd_enable;
                                fwd_left_pwm = 16'($urandom_range(0, 200));
                                fwd_right_pwm = 16'($urandom_range(0, 200)); end
        else if (r < 94)  fwd_left_pwm = -16'($urandom_range(0, 50));
        else if (r < 98)  sonar_valid = ~sonar_valid;
        else if (r < 104) sonar_forward_clear = ~sonar_forward_clear;
        else if (r < 108) camera_target_fresh = ~camera_target_fresh;
        else if (r < 125) distance_mm = 16'($urandom_range(100, 1500));
        else if (r < 127) reverse_enable = ~reverse_enable;
    endtask

    initial begin
        #100ms;
        $fatal(1, "TIMEOUT: motion_supervisor_tb");
    end

endmodule

`default_nettype wire
