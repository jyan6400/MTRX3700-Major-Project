`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// MTRX3700 Major Project - Group 26 - V2
// Ultrasonic pipeline with paired V1 / V2 comparison (owner: Advay Hassan, M5)
//
//   hcsr04_model (Ed) -> sonar_range (Ed, sonar PLL clock)
//     -> sonar_result_cdc (43.904 -> 50 MHz)
//        -> sonar_freshness   (V1, as in the current top level)
//        -> sonar_filter      (V2, bypass = 0)
//
// Both receive the SAME echoes. The approach runs from 600 mm to a dock at
// 245 mm in 30 mm steps, with two injected faults that happen on real robots:
//   reading 5: a spurious 150 mm echo (e.g. a side reflection)
//   reading 7: a missed echo (nothing reflects; the driver reports 4000 mm)
//
// Scored per reading, against the true distance:
//   false dock = valid && distance <= 250 mm while truly > 280 mm
//                (docking_fsm would latch DOCKED: the robot stops short)
//   dropout    = sonar_valid low while the dock is truly in range
//                (the motor gate stops the wheels for a sonar period)
//
// Expected: V1 shows 1 false dock and 1 dropout; V2 shows 0 and 0, docks on
// the same reading as the true arrival, pauses forward motion for exactly one
// reading on the spike, and both go invalid 250 ms after the last echo.
// Readings are 62 ms apart (the sensor asks for at least 60 ms).
// ============================================================================

module sonar_filter_pipeline_tb;

    localparam int STOP_MM   = 250;
    localparam int MARGIN_MM = 30;
    localparam int N_READ    = 14;

    logic clk_50 = 1'b0;
    always #10 clk_50 = ~clk_50;

    logic rst_n = 1'b0;

    wire sonar_clk;
    wire pll_locked;

    sonar_pll u_pll (
        .refclk(clk_50),
        .rst(1'b0),
        .outclk_0(sonar_clk),
        .locked(pll_locked)
    );

    // ---------------- Ed sensor model and driver ----------------

    logic start_measure = 1'b0;

    wire        trig;
    wire        echo;
    wire [11:0] sonar_distance;
    wire        sonar_ready;
    wire        sonar_measure_valid;

    int   wall_distance = 1000;
    logic object_present = 1'b1;

    int model_measurements;
    int model_errors;
    int model_notes;
    int model_last_echo_us;

    sonar_range u_sonar (
        .clk(sonar_clk),
        .start_measure(start_measure),
        .rst(!rst_n),
        .echo(echo),
        .trig(trig),
        .distance(sonar_distance),
        .ready(sonar_ready),
        .valid(sonar_measure_valid)
    );

    hcsr04_model #(
        .NAME("SONAR"),
        .VERBOSE(0)
    ) u_sensor (
        .trig(trig),
        .echo(echo),
        .distance_mm(wall_distance),
        .object_present(object_present),
        .measurements(model_measurements),
        .errors(model_errors),
        .notes(model_notes),
        .last_echo_us(model_last_echo_us)
    );

    // ---------------- CDC to CLOCK_50 ----------------

    wire        cdc_ready;
    wire        cdc_overrun;
    wire        measurement_valid;
    wire [15:0] measurement_mm;

    sonar_result_cdc u_cdc (
        .src_clk(sonar_clk),
        .dst_clk(clk_50),
        .rst_n(rst_n),
        .src_measurement_valid(sonar_measure_valid),
        .src_distance(sonar_distance),
        .src_ready(cdc_ready),
        .src_overrun(cdc_overrun),
        .measurement_valid(measurement_valid),
        .measurement_mm(measurement_mm)
    );

    // ---------------- V1: sonar_freshness ----------------

    wire        v1_valid;
    wire [15:0] v1_distance;

    sonar_freshness #(
        .MAX_AGE_CYCLES(12_500_000),
        .MIN_RANGE_MM(16'd20),
        .MAX_RANGE_MM(16'd3500)
    ) u_v1 (
        .clk(clk_50),
        .rst_n(rst_n),
        .measurement_valid(measurement_valid),
        .measurement_mm(measurement_mm),
        .sonar_valid(v1_valid),
        .distance_mm(v1_distance)
    );

    // ---------------- V2: sonar_filter ----------------

    wire        v2_valid;
    wire [15:0] v2_distance;
    wire        v2_fresh;
    wire        v2_forward_clear;
    wire        v2_update;
    wire [15:0] v2_raw_mm;
    wire [1:0]  v2_window_count;
    wire        v2_close_suspect;
    wire        v2_fault_no_echo;
    wire        v2_fault_stale;
    wire [15:0] v2_rejected;
    wire [15:0] v2_bridged;
    wire        v2_bypass_active;

    sonar_filter #(
        .MAX_AGE_CYCLES(12_500_000),
        .MIN_RANGE_MM(16'd20),
        .MAX_RANGE_MM(16'd3500),
        .OUTLIER_MM(16'd80),
        .STOP_MM(16'(STOP_MM)),
        .MAX_MISSES(2)
    ) u_v2 (
        .clk(clk_50),
        .rst_n(rst_n),
        .bypass(1'b0),
        .measurement_valid(measurement_valid),
        .measurement_mm(measurement_mm),
        .sonar_valid(v2_valid),
        .distance_mm(v2_distance),
        .sonar_fresh(v2_fresh),
        .forward_clear(v2_forward_clear),
        .update(v2_update),
        .raw_mm(v2_raw_mm),
        .window_count(v2_window_count),
        .close_suspect(v2_close_suspect),
        .fault_no_echo(v2_fault_no_echo),
        .fault_stale(v2_fault_stale),
        .rejected_count(v2_rejected),
        .bridged_count(v2_bridged),
        .bypass_active(v2_bypass_active)
    );

    // ---------------- Helpers ----------------

    int received = 0;

    always @(negedge clk_50) begin
        if (rst_n && measurement_valid)
            received++;
    end

    int checks = 0;

    task automatic pass(input string name);
        checks++;
        $display("PASS [%0d]: %s", checks, name);
    endtask

    // Ask the Ed driver for one measurement (as in sonar_pipeline_tb).
    task automatic request_measurement(input int distance, input bit present);
        wait (sonar_ready === 1'b1);
        wait (echo === 1'b0);
        @(negedge sonar_clk);
        wall_distance  = distance;
        object_present = present;
        start_measure  = 1'b1;
        do begin
            @(negedge sonar_clk);
        end while (sonar_ready);
        start_measure = 1'b0;
    endtask

    // ---------------- Scenario ----------------
    //                         1    2    3    4    5    6    7    8    9   10   11   12   13   14
    int truth_mm  [N_READ] = '{600, 570, 540, 510, 480, 450, 420, 390, 360, 330, 300, 270, 245, 245};
    int echo_mm   [N_READ] = '{600, 570, 540, 510, 150, 450, 420, 390, 360, 330, 300, 270, 245, 245};
    bit present   [N_READ] = '{  1,   1,   1,   1,   1,   1,   0,   1,   1,   1,   1,   1,   1,   1};

    int v1_false_dock = 0;
    int v2_false_dock = 0;
    int v1_dropout    = 0;
    int v2_dropout    = 0;
    int v2_pauses     = 0;
    int v1_dock_at    = -1;
    int v2_dock_at    = -1;
    int true_dock_at  = -1;
    int start_received;

    initial begin
        if ($test$plusargs("wave")) begin
            $dumpfile("sonar_filter_pipeline.fst");
            $dumpvars(0, sonar_filter_pipeline_tb);
        end

        repeat (10) @(negedge clk_50);
        rst_n = 1'b1;
        repeat (10) @(negedge clk_50);

        if (v1_valid !== 1'b0 || v2_valid !== 1'b0 || v2_forward_clear !== 1'b0)
            $fatal(1, "FAIL: both paths must start invalid with forward motion blocked");
        pass("reset: V1 and V2 invalid, forward motion blocked");

        for (int i = 0; i < N_READ; i++) begin
            start_received = received;
            request_measurement(echo_mm[i], present[i]);
            wait (received > start_received);
            repeat (3) @(posedge clk_50);
            @(negedge clk_50);

            $display("  reading %2d  truth %4d  echo %-9s | V1 %b %4d | V2 %b %4d fresh %b fwd %b%s",
                     i + 1, truth_mm[i],
                     present[i] ? $sformatf("%0d", echo_mm[i]) : "none",
                     v1_valid, v1_distance, v2_valid, v2_distance, v2_fresh, v2_forward_clear,
                     v2_close_suspect ? "  (paused: unconfirmed close echo)" : "");

            if (true_dock_at < 0 && truth_mm[i] <= STOP_MM) true_dock_at = i + 1;
            if (v1_dock_at < 0 && v1_valid && v1_distance <= 16'(STOP_MM)) v1_dock_at = i + 1;
            if (v2_dock_at < 0 && v2_valid && v2_distance <= 16'(STOP_MM)) v2_dock_at = i + 1;

            if (truth_mm[i] > STOP_MM + MARGIN_MM) begin
                if (v1_valid && v1_distance <= 16'(STOP_MM)) v1_false_dock++;
                if (v2_valid && v2_distance <= 16'(STOP_MM)) v2_false_dock++;
            end

            // From reading 3, both paths have had enough readings to be valid.
            if (i >= 2) begin
                if (!v1_valid) v1_dropout++;
                if (!v2_valid) v2_dropout++;
            end

            if (v2_close_suspect) v2_pauses++;

            // Spot checks on the two injected faults.
            if (i == 4) begin
                if (!(v2_valid && v2_distance > 16'(STOP_MM) && !v2_forward_clear))
                    $fatal(1, "FAIL: V2 on the spurious echo must hold its distance and pause");
                pass("spurious 150 mm echo: V2 holds distance, pauses forward motion");
            end
            if (i == 5) begin
                if (!v2_forward_clear)
                    $fatal(1, "FAIL: V2 must resume after the unconfirmed echo");
                pass("next reading: V2 resumes forward motion");
            end
            if (i == 6) begin
                if (!(v2_valid && v2_forward_clear && !v2_fresh))
                    $fatal(1, "FAIL: V2 must bridge a single missed echo and mark it not fresh");
                pass("missed echo: V2 bridges it (valid, not fresh), forward motion continues");
            end
            if (i >= 2 && i != 6 && !v2_fresh)
                $fatal(1, "FAIL: V2 not fresh on echo reading %0d", i + 1);

            if (i != N_READ - 1)
                #62ms;
        end

        $display("");
        $display("  Paired comparison over %0d readings (same echoes):", N_READ);
        $display("                         V1 freshness   V2 filter");
        $display("    false docks               %0d             %0d", v1_false_dock, v2_false_dock);
        $display("    sonar dropouts            %0d             %0d", v1_dropout, v2_dropout);
        $display("    first reading <= 250 mm   %0d             %0d   (true arrival: reading %0d)",
                 v1_dock_at, v2_dock_at, true_dock_at);
        $display("    forward pauses            -             %0d", v2_pauses);
        $display("");

        if (v1_false_dock != 1 || v1_dropout != 1)
            $fatal(1, "FAIL: scenario did not reproduce the V1 weaknesses (%0d false docks, %0d dropouts)",
                   v1_false_dock, v1_dropout);
        pass("V1 baseline reproduced: 1 false dock, 1 dropout");

        if (v2_false_dock != 0 || v2_dropout != 0)
            $fatal(1, "FAIL: V2 had %0d false docks and %0d dropouts", v2_false_dock, v2_dropout);
        pass("V2: 0 false docks, 0 dropouts");

        if (v2_dock_at != true_dock_at)
            $fatal(1, "FAIL: V2 docked at reading %0d, true arrival %0d", v2_dock_at, true_dock_at);
        pass("V2 docks on the same reading as the true arrival (no added lag)");

        if (v2_forward_clear !== 1'b0 || v2_distance > 16'(STOP_MM) || v2_distance < 16'(STOP_MM - 10))
            $fatal(1, "FAIL: V2 at the dock: %0d mm, forward_clear %b", v2_distance, v2_forward_clear);
        pass($sformatf("V2 at the dock: %0d mm, forward motion blocked", v2_distance));

        if (v2_pauses != 1 || v2_rejected != 16'd1 || v2_bridged != 16'd1)
            $fatal(1, "FAIL: V2 diagnostics: %0d pauses, %0d rejected, %0d bridged",
                   v2_pauses, v2_rejected, v2_bridged);
        pass("V2 diagnostics: 1 pause, 1 rejected echo, 1 bridged miss");

        // No further readings: both must go invalid 250 ms after the last one.
        #200ms;
        if (!v2_valid)
            $fatal(1, "FAIL: V2 expired too early");
        #60ms;
        if (v1_valid || v2_valid || v2_fresh || v2_forward_clear || !v2_fault_stale)
            $fatal(1, "FAIL: stale readings not invalidated (V1 %b, V2 %b, stale flag %b)",
                   v1_valid, v2_valid, v2_fault_stale);
        pass("250 ms without an echo: V1 and V2 invalid, V2 flags stale");

        if (model_errors != 0)
            $fatal(1, "FAIL: sensor model reported %0d errors", model_errors);
        if (cdc_overrun)
            $fatal(1, "FAIL: CDC overrun");
        if (received != N_READ)
            $fatal(1, "FAIL: expected %0d readings through the CDC, got %0d", N_READ, received);
        pass("all readings crossed the CDC, 0 sensor-model errors, no overrun");

        $display("---------------------------------------");
        $display("PASS: sonar_filter_pipeline_tb (%0d checks)", checks);
        $display("---------------------------------------");
        $finish;
    end

    initial begin
        #2s;
        $fatal(1, "TIMEOUT: sonar_filter_pipeline_tb");
    end

endmodule

`default_nettype wire
