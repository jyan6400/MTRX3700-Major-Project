`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// MTRX3700 Major Project - Group 26 - V2
// Unit test for sonar_filter.sv (owner: Advay Hassan, M5)
//
// Part A: directed checks of every rule in sonar_filter's header
//         (warm-up, constant and noisy-constant range, smooth tracking,
//          spikes, the close-obstacle pause, bridged and repeated misses,
//          freshness, clamping, staleness, 249/250/251 mm around STOP_MM,
//          the OUTLIER_MM boundary, 1-clock latency, recovery, bypass = V1,
//          reset).
// Part B: 3000 randomised readings (random walk + spikes + misses) compared,
//         reading by reading, with an independent reference model written
//         with a sorted queue instead of comparators.
//
// Short MAX_AGE_CYCLES (50) keeps the run fast; the logic is identical at
// the 12.5 M cycles used in the top level.
// Ends with "PASS: sonar_filter_tb" or stops at the first "FAIL".
// ============================================================================

module sonar_filter_tb;

    localparam int MAX_AGE    = 50;
    localparam int MIN_RANGE  = 20;
    localparam int MAX_RANGE  = 3500;
    localparam int OUTLIER    = 80;
    localparam int STOP       = 250;
    localparam int MAX_MISSES = 2;
    localparam int N_RANDOM   = 3000;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic        rst_n = 1'b0;
    logic        bypass = 1'b0;
    logic        measurement_valid = 1'b0;
    logic [15:0] measurement_mm = 16'd0;

    logic        sonar_valid;
    logic [15:0] distance_mm;
    logic        sonar_fresh;
    logic        forward_clear;
    logic        update;
    logic [15:0] raw_mm;
    logic [1:0]  window_count;
    logic        close_suspect;
    logic        fault_no_echo;
    logic        fault_stale;
    logic [15:0] rejected_count;
    logic [15:0] bridged_count;
    logic        bypass_active;

    sonar_filter #(
        .MAX_AGE_CYCLES(MAX_AGE),
        .MIN_RANGE_MM(16'(MIN_RANGE)),
        .MAX_RANGE_MM(16'(MAX_RANGE)),
        .OUTLIER_MM(16'(OUTLIER)),
        .STOP_MM(16'(STOP)),
        .MAX_MISSES(MAX_MISSES)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .bypass(bypass),
        .measurement_valid(measurement_valid),
        .measurement_mm(measurement_mm),
        .sonar_valid(sonar_valid),
        .distance_mm(distance_mm),
        .sonar_fresh(sonar_fresh),
        .forward_clear(forward_clear),
        .update(update),
        .raw_mm(raw_mm),
        .window_count(window_count),
        .close_suspect(close_suspect),
        .fault_no_echo(fault_no_echo),
        .fault_stale(fault_stale),
        .rejected_count(rejected_count),
        .bridged_count(bridged_count),
        .bypass_active(bypass_active)
    );

    // Independent V1 reference: in bypass, the DUT must match it exactly.
    logic        v1_valid;
    logic [15:0] v1_distance_mm;

    sonar_freshness #(
        .MAX_AGE_CYCLES(MAX_AGE),
        .MIN_RANGE_MM(16'(MIN_RANGE)),
        .MAX_RANGE_MM(16'(MAX_RANGE))
    ) v1_ref (
        .clk(clk),
        .rst_n(rst_n),
        .measurement_valid(measurement_valid),
        .measurement_mm(measurement_mm),
        .sonar_valid(v1_valid),
        .distance_mm(v1_distance_mm)
    );

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    int checks = 0;
    int cyc = 0;                       // index of the latest rising edge
    int last_sample_cyc = 0;           // edge at which the last reading was taken

    always @(posedge clk) cyc++;

    task automatic pass(input string name);
        checks++;
        $display("PASS [%0d]: %s", checks, name);
    endtask

    // One reading: a one-clock measurement_valid pulse. Returns just after
    // the edge that takes it, when the registered outputs have updated.
    task automatic send(input int mm);
        @(negedge clk);
        measurement_valid = 1'b1;
        measurement_mm    = 16'(mm);
        @(posedge clk);
        #1;
        last_sample_cyc = cyc;
        @(negedge clk);
        measurement_valid = 1'b0;
        #1;
    endtask

    task automatic expect_out(
        input string name,
        input bit    exp_valid,
        input int    exp_distance,
        input bit    exp_forward_clear
    );
        if (sonar_valid !== exp_valid ||
            distance_mm !== 16'(exp_distance) ||
            forward_clear !== exp_forward_clear) begin
            $fatal(1,
                "FAIL %s: valid=%b distance=%0d forward_clear=%b, expected %b/%0d/%b",
                name, sonar_valid, distance_mm, forward_clear,
                exp_valid, exp_distance, exp_forward_clear);
        end
        pass(name);
    endtask

    task automatic expect_bit(input string name, input logic actual, input bit expected);
        if (actual !== expected)
            $fatal(1, "FAIL %s: got %b, expected %b", name, actual, expected);
        pass(name);
    endtask

    task automatic expect_int(input string name, input int actual, input int expected);
        if (actual != expected)
            $fatal(1, "FAIL %s: got %0d, expected %0d", name, actual, expected);
        pass(name);
    endtask

    // Read cyc 1 ns after each edge, never in the same instant it changes.
    task automatic wait_until_cycle(input int c);
        while (cyc < c) begin
            @(posedge clk);
            #1;
        end
    endtask

    // Bypass equivalence monitor: whenever bypass is active, the DUT's
    // navigation outputs must equal the unmodified V1 module's.
    bit bypass_checks_on = 1'b0;
    int bypass_cycles_checked = 0;

    always @(negedge clk) begin
        if (bypass_checks_on && rst_n && bypass_active) begin
            if (sonar_valid !== v1_valid || distance_mm !== v1_distance_mm ||
                sonar_fresh !== v1_valid ||
                forward_clear !== (v1_valid && (v1_distance_mm > 16'(STOP))))
                $fatal(1, "FAIL bypass: DUT %b/%0d differs from V1 %b/%0d",
                       sonar_valid, distance_mm, v1_valid, v1_distance_mm);
            bypass_cycles_checked++;
        end
    end

    // ------------------------------------------------------------------
    // Reference model for Part B (different structure: a sorted queue)
    // ------------------------------------------------------------------

    int m_win[$];
    bit m_valid;
    int m_dist;
    bit m_fresh;
    bit m_susp;
    int m_misses;
    int m_rejected;
    int m_bridged;

    task automatic model_reset();
        m_win.delete();
        m_valid    = 1'b0;
        m_dist     = 0;
        m_fresh    = 1'b0;
        m_susp     = 1'b0;
        m_misses   = 0;
        m_rejected = 0;
        m_bridged  = 0;
    endtask

    task automatic model_step(input int mm);
        int s;
        int med;
        int dev;
        bit is_outlier;
        int sorted[$];

        if (mm > MAX_RANGE) begin
            if (m_misses + 1 >= MAX_MISSES) begin
                m_valid  = 1'b0;
                m_fresh  = 1'b0;
                m_dist   = 0;
                m_susp   = 1'b0;
                m_misses = 0;
                m_win.delete();
            end else begin
                m_misses++;
                m_fresh = 1'b0;
                if (m_valid) m_bridged++;
            end
        end else begin
            s = (mm < MIN_RANGE) ? MIN_RANGE : mm;
            m_misses = 0;
            if (m_win.size() == 2) begin
                sorted = {m_win[0], m_win[1], s};
                sorted.sort();
                med = sorted[1];
                dev = (s > med) ? (s - med) : (med - s);
                is_outlier = (dev > OUTLIER);
                m_dist  = is_outlier ? med : s;
                m_valid = 1'b1;
                m_fresh = 1'b1;
                m_susp  = is_outlier && (s < med) && (s <= STOP);
                if (is_outlier) m_rejected++;
            end
            m_win.push_back(s);
            if (m_win.size() > 2) void'(m_win.pop_front());
        end
    endtask

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------

    int truth;
    int reading;
    int r;
    int non_range_run;
    int n_spikes;
    int n_misses;

    initial begin
        if ($test$plusargs("wave")) begin
            $dumpfile("sonar_filter.fst");
            $dumpvars(0, sonar_filter_tb);
        end

        // ---------------- Part A: directed ----------------

        repeat (3) @(negedge clk);
        expect_out("reset: invalid, 0 mm, no forward motion", 0, 0, 0);
        expect_int("reset: window empty", int'(window_count), 0);

        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        // Warm-up: three in-range readings before anything is believed.
        send(1000);
        expect_out("warm-up: 1st reading not yet believed", 0, 0, 0);
        expect_bit("warm-up: not fresh", sonar_fresh, 1'b0);
        expect_bit("update pulses for one clock", update, 1'b1);
        @(posedge clk); #1;
        expect_bit("update clears after one clock", update, 1'b0);
        send(990);
        expect_out("warm-up: 2nd reading not yet believed", 0, 0, 0);
        send(980);
        expect_out("valid after three in-range readings", 1, 980, 1);
        expect_bit("fresh once valid", sonar_fresh, 1'b1);

        // Smooth approach: report the newest reading, no median lag.
        send(940);  expect_out("smooth approach: 940 reported at once", 1, 940, 1);
        send(900);  expect_out("smooth approach: 900 reported at once", 1, 900, 1);
        send(860);  expect_out("smooth approach: 860 reported at once", 1, 860, 1);

        // A single short spike: distance holds (no false DOCKED), but forward
        // motion pauses for one reading in case it is real.
        send(150);
        expect_out("short spike: distance held at median, forward paused", 1, 860, 0);
        expect_bit("short spike: close_suspect raised", close_suspect, 1'b1);
        expect_int("short spike: counted as rejected", int'(rejected_count), 1);
        send(840);
        expect_out("spike not confirmed: tracking and motion resume", 1, 840, 1);
        expect_bit("close_suspect cleared", close_suspect, 1'b0);

        // A single long spike: rejected, no pause needed.
        send(3000);
        expect_out("long spike: rejected, motion continues", 1, 840, 1);
        send(820);
        expect_out("after long spike: 820 tracked", 1, 820, 1);

        // A large step that stays above STOP_MM: no pause; believed on the
        // second reading.
        send(600);
        expect_out("far step (600 > STOP): held, no pause", 1, 820, 1);
        send(600);
        expect_out("far step confirmed on 2nd reading", 1, 600, 1);

        // One missed echo (driver's 4000 mm): bridged.
        send(4000);
        expect_out("single miss bridged: still valid, distance held", 1, 600, 1);
        expect_bit("held value during a bridged miss is NOT marked fresh", sonar_fresh, 1'b0);
        expect_int("bridged miss counted", int'(bridged_count), 1);
        expect_int("raw_mm shows the actual reading (4000)", int'(raw_mm), 4000);
        send(580);
        expect_out("after bridged miss: 580 tracked", 1, 580, 1);
        expect_bit("fresh again after the next echo", sonar_fresh, 1'b1);

        // Beyond MAX_RANGE_MM is a miss; two in a row invalidate.
        send(3600);
        expect_out("3600 mm (beyond range) treated as a miss and bridged", 1, 580, 1);
        send(3700);
        expect_out("2nd consecutive miss: invalid, 0 mm, stop", 0, 0, 0);
        expect_bit("invalid is never fresh", sonar_fresh, 1'b0);
        expect_bit("fault_no_echo set", fault_no_echo, 1'b1);
        expect_int("window emptied", int'(window_count), 0);

        // Recovery needs a full window again.
        send(560);
        expect_out("recovery: 1st reading not yet believed", 0, 0, 0);
        send(550);
        expect_out("recovery: 2nd reading not yet believed", 0, 0, 0);
        send(540);
        expect_out("recovery: valid on 3rd reading", 1, 540, 1);
        expect_bit("fault_no_echo cleared", fault_no_echo, 1'b0);

        // Approach to the stop distance; boundaries of OUTLIER_MM and STOP_MM.
        send(480);  expect_out("approach 480", 1, 480, 1);
        send(400);  expect_out("deviation exactly OUTLIER_MM (80) accepted", 1, 400, 1);
        send(330);  expect_out("approach 330", 1, 330, 1);
        send(270);  expect_out("approach 270", 1, 270, 1);
        send(251);  expect_out("251 mm > STOP_MM: forward still clear", 1, 251, 1);
        send(250);  expect_out("250 mm = STOP_MM: forward blocked", 1, 250, 0);
        send(249);  expect_out("249 mm < STOP_MM: forward blocked", 1, 249, 0);

        // Too-close echoes are clamped to MIN_RANGE_MM (treated as close).
        send(5);
        expect_out("one too-close echo: rejected, forward blocked", 1, 249, 0);
        send(5);
        expect_out("two too-close echoes: believed as 20 mm", 1, 20, 0);

        // Staleness: no in-range reading for MAX_AGE_CYCLES.
        wait_until_cycle(last_sample_cyc + MAX_AGE - 1);
        expect_out("valid until MAX_AGE_CYCLES-1 clocks after last reading", 1, 20, 0);
        wait_until_cycle(last_sample_cyc + MAX_AGE);
        expect_out("stale after MAX_AGE_CYCLES clocks: invalid", 0, 0, 0);
        expect_bit("stale is never fresh", sonar_fresh, 1'b0);
        expect_bit("fault_stale set", fault_stale, 1'b1);

        // Recovery from stale: three readings of a constant range.
        send(1000); send(1000); send(1000);
        expect_out("recovery after stale: valid on 3rd reading", 1, 1000, 1);
        expect_bit("fault_stale cleared", fault_stale, 1'b0);
        r = int'(rejected_count);
        repeat (5) send(1000);
        expect_out("constant 1000 mm held exactly", 1, 1000, 1);

        // Exact latency: outputs change on the first clock edge after the
        // measurement_valid pulse, and not before.
        @(negedge clk);
        measurement_valid = 1'b1;
        measurement_mm    = 16'd1003;
        #1;
        expect_out("latency: unchanged before the clock edge", 1, 1000, 1);
        @(posedge clk);
        #1;
        expect_out("latency: new value 1 clock (20 ns at 50 MHz) after the pulse", 1, 1003, 1);
        @(negedge clk);
        measurement_valid = 1'b0;

        // Noisy constant (+/-3 mm, the sensor's accuracy): tracked, nothing rejected.
        send(997);   expect_out("noisy constant: 997 tracked", 1, 997, 1);
        send(1002);  expect_out("noisy constant: 1002 tracked", 1, 1002, 1);
        send(998);   expect_out("noisy constant: 998 tracked", 1, 998, 1);
        expect_int("noisy constant: no readings rejected", int'(rejected_count), r);

        send(1000); send(1000);
        send(919);
        expect_out("deviation OUTLIER_MM+1 (81) rejected", 1, 1000, 1);
        send(920);
        expect_out("920 tracked", 1, 920, 1);

        // A miss does not restart the staleness timer.
        r = last_sample_cyc;
        wait_until_cycle(r + 20);
        send(4000);
        expect_out("miss mid-wait bridged", 1, 920, 1);
        wait_until_cycle(r + MAX_AGE - 1);
        expect_out("still valid at MAX_AGE-1 after last IN-RANGE reading", 1, 920, 1);
        wait_until_cycle(r + MAX_AGE);
        expect_out("stale at MAX_AGE after last in-range reading despite the miss", 0, 0, 0);

        expect_int("rejected_count total", int'(rejected_count), 5);
        expect_int("bridged_count total", int'(bridged_count), 3);

        // Bypass = V1 behaviour (embedded sonar_freshness), checked every clock
        // against an independent sonar_freshness instance.
        @(negedge clk);
        bypass = 1'b1;
        bypass_checks_on = 1'b1;
        repeat (3) @(posedge clk);
        #1;
        expect_bit("bypass switch synchronised", bypass_active, 1'b1);
        send(800);
        expect_out("bypass: V1 believes the first reading", 1, 800, 1);
        send(150);
        expect_out("bypass: V1 believes a single short spike (V1 weakness)", 1, 150, 0);
        send(4000);
        expect_out("bypass: V1 drops out on a single miss (V1 weakness)", 0, 0, 0);
        send(700);
        expect_out("bypass: V1 valid again at once", 1, 700, 1);
        repeat (5) @(posedge clk);
        @(negedge clk);
        bypass = 1'b0;
        repeat (3) @(posedge clk);
        #1;
        bypass_checks_on = 1'b0;
        expect_bit("bypass released", bypass_active, 1'b0);
        expect_out("V2 kept running underneath while bypassed", 1, 700, 1);
        if (bypass_cycles_checked < 10)
            $fatal(1, "FAIL: bypass monitor checked only %0d clocks", bypass_cycles_checked);
        pass("bypass outputs equal V1 on every clock checked");

        // Distance changes without a valid pulse are ignored.
        @(negedge clk);
        measurement_mm = 16'd123;
        repeat (5) @(posedge clk);
        #1;
        expect_out("input ignored without measurement_valid", 1, 700, 1);

        // Asynchronous reset clears everything at once.
        @(negedge clk);
        rst_n = 1'b0;
        #1;
        expect_out("reset clears outputs immediately", 0, 0, 0);
        expect_int("reset clears rejected_count", int'(rejected_count), 0);
        expect_int("reset clears bridged_count", int'(bridged_count), 0);
        expect_bit("reset clears bypass", bypass_active, 1'b0);

        // ---------------- Part B: randomised vs reference model ----------------

        repeat (2) @(negedge clk);
        rst_n = 1'b1;
        model_reset();
        void'($urandom(32'd26));

        truth = 1500;
        non_range_run = 0;
        n_spikes = 0;
        n_misses = 0;

        for (int i = 0; i < N_RANDOM; i++) begin
            // Random walk of the true distance (up to 60 mm per reading).
            truth = truth + int'($urandom_range(0, 120)) - 60;
            if (truth < 30)   truth = 30;
            if (truth > 3400) truth = 3400;

            r = int'($urandom_range(0, 99));
            if (non_range_run >= 3) r = 99;          // keep clear of staleness

            if (r < 6) begin                         // short spike (may be < 20 mm)
                reading = int'($urandom_range(0, truth));
                n_spikes++;
            end else if (r < 10) begin               // long spike
                reading = int'($urandom_range(truth, MAX_RANGE));
                n_spikes++;
            end else if (r < 18) begin               // missed echo
                reading = 4000;
                n_misses++;
            end else if (r < 20) begin               // beyond range
                reading = int'($urandom_range(MAX_RANGE + 1, 3999));
                n_misses++;
            end else begin                           // normal, +/-3 mm noise
                reading = truth + int'($urandom_range(0, 6)) - 3;
            end

            non_range_run = (reading > MAX_RANGE) ? non_range_run + 1 : 0;

            // Readings 2 to 13 clocks apart. A filled window is emptied by the
            // second consecutive miss at most 26 clocks after its last in-range
            // reading, so staleness (tested in Part A) cannot change the outputs
            // here and the model does not need to include it.
            repeat (int'($urandom_range(0, 11))) @(negedge clk);

            send(reading);
            model_step(reading);

            if (sonar_valid !== m_valid ||
                sonar_fresh !== (m_valid && m_fresh) ||
                distance_mm !== 16'(m_dist) ||
                close_suspect !== m_susp ||
                forward_clear !== (m_valid && (m_dist > STOP) && !m_susp))
                $fatal(1,
                    "FAIL random reading %0d (%0d mm): DUT %b/%0d/fresh %b/susp %b/fc %b, model %b/%0d/fresh %b/susp %b",
                    i, reading, sonar_valid, distance_mm, sonar_fresh, close_suspect, forward_clear,
                    m_valid, m_dist, m_fresh, m_susp);
        end

        expect_int("random: rejected_count matches model", int'(rejected_count), m_rejected);
        expect_int("random: bridged_count matches model", int'(bridged_count), m_bridged);
        pass($sformatf("random: %0d readings (%0d spikes, %0d misses) match the reference model",
                       N_RANDOM, n_spikes, n_misses));

        $display("-----------------------------------------");
        $display("PASS: sonar_filter_tb (%0d checks)", checks);
        $display("-----------------------------------------");
        $finish;
    end

    initial begin
        #20ms;
        $fatal(1, "TIMEOUT: sonar_filter_tb");
    end

endmodule

`default_nettype wire
