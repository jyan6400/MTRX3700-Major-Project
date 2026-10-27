`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// MTRX3700 Major Project - Group 26 - V2 (robust docking)
// Workstream M5: sensors, safety and chassis actuation (Advay Hassan)
//
// sonar_filter: robust ultrasonic range for docking decisions and the
// forward-motion gate.
//
// Position in the design (50 MHz CLOCK_50 domain):
//
//   sonar_range (Ed) -> sonar_result_cdc -> [sonar_filter] -> docking_fsm,
//                                                              motor gating
//
// It replaces sonar_freshness in major_project_top. The two V1 outputs keep
// their names and meaning (sonar_valid, distance_mm, distance 0 when invalid),
// so docking_fsm, motion_interface and the top-level tests need no changes.
//
// ---------------------------------------------------------------------------
// The V1 weakness this fixes (sonar_freshness believes every single reading)
// ---------------------------------------------------------------------------
//   * One spurious short echo (e.g. 150 mm while the dock is 800 mm away) is
//     passed straight through; docking_fsm latches DOCKED on it and the robot
//     stops short of the bay for good.
//   * One missed echo (the Ed driver reports 4000 mm) clears sonar_valid and
//     stops the wheels for a whole sonar period (100 ms at the 10 Hz pace).
//
// ---------------------------------------------------------------------------
// What happens to each new reading (V2 mode, bypass = 0)
// ---------------------------------------------------------------------------
//   1. Classify. Above MAX_RANGE_MM (which includes the driver's 4000 mm
//      no-echo value) the reading is a MISS. Below MIN_RANGE_MM it is clamped
//      to MIN_RANGE_MM: the safe reading of a too-close echo is "very close".
//   2. Median.   med = median of this reading and the two previous in-range
//      readings held in the window.
//   3. Outlier.  If |reading - med| > OUTLIER_MM the reading is an outlier and
//      med is reported instead. Otherwise the reading itself is reported, so
//      a smooth approach gets no extra sonar-period lag (a plain median would
//      always report the middle, i.e. one-period-old, reading).
//   4. Misses.   Up to MAX_MISSES-1 consecutive misses are bridged: the last
//      distance is held and sonar_valid stays high. MAX_MISSES misses in a row
//      invalidate the output and empty the window.
//   5. Stale.    No in-range reading for MAX_AGE_CYCLES (sensor unplugged,
//      driver stalled) invalidates the output and empties the window. Misses
//      do not restart this timer, so bridging can never outlast it.
//   6. Warm-up.  After reset or invalidation, sonar_valid returns only when
//      the window has three in-range readings again.
//   7. Safety.   forward_clear drops on the FIRST outlier that is closer than
//      the median and at or below STOP_MM ("close_suspect"). If that reading
//      is real, forward motion has already stopped as fast as in V1; if it is
//      a glitch, motion resumes one reading later. distance_mm, which the FSM
//      uses to declare DOCKED, changes only once a second reading confirms.
//
// bypass (a switch; synchronised here) = 1 gives exactly the V1 behaviour,
// from an embedded, unmodified sonar_freshness, for the R9 bypass comparison.
// The V2 path keeps running underneath, so switching back is immediate.
//
// ---------------------------------------------------------------------------
// Validity and freshness (what each output promises)
// ---------------------------------------------------------------------------
//   sonar_valid  distance_mm is usable for control: confirmed by a full window
//                and the last in-range echo is younger than MAX_AGE_CYCLES
//                (the same 250 ms limit as V1; bridging never extends it).
//   sonar_fresh  sonar_valid AND the most recent sonar measurement was itself
//                an in-range echo. Low while a miss is being bridged, so a held
//                value is never reported as a fresh observation.
//   forward_clear  threshold status for the motor gate: valid, distance_mm >
//                STOP_MM (250 mm) and no unconfirmed close echo pending.
//   raw_mm       the actual measured range of the last reading, unfiltered.
//
// ---------------------------------------------------------------------------
// Latency
// ---------------------------------------------------------------------------
//   * All registered outputs update 1 clock (20 ns) after measurement_valid.
//   * Smooth approach (steps <= OUTLIER_MM per reading): no added sonar-period
//     lag compared with V1.
//   * Sudden change in distance_mm: reported on the 2nd consecutive reading
//     (one extra sonar period). forward_clear still drops on the 1st reading
//     when the sudden change is a close obstacle (step 7).
//
// Choosing OUTLIER_MM: it must exceed the furthest the robot can travel in one
// sonar period plus sensor noise (about 3 mm), or a fast, real approach would
// be treated as glitches. Default 80 mm suits up to about 0.75 m/s at 10 Hz.
//
// Limitations
//   * Two equal bad readings in a row look exactly like a real object and are
//     believed. The IR stop remains the emergency mechanism.
//   * Front sensor only: no protection while reversing (V3 work).
//   * Inputs must be synchronous to clk (sonar_result_cdc guarantees this),
//     except bypass, which is synchronised here.
// ============================================================================

module sonar_filter #(
    parameter int unsigned MAX_AGE_CYCLES = 32'd12_500_000, // 250 ms at 50 MHz (as V1)
    parameter logic [15:0] MIN_RANGE_MM   = 16'd20,         // RCWL-1601 minimum range
    parameter logic [15:0] MAX_RANGE_MM   = 16'd3500,       // beyond this = miss (as V1)
    parameter logic [15:0] OUTLIER_MM     = 16'd80,         // largest believable jump per reading
    parameter logic [15:0] STOP_MM        = 16'd250,        // forward motion needs distance > STOP_MM
    parameter int unsigned MAX_MISSES     = 2               // consecutive misses that invalidate (>= 1)
) (
    input  wire         clk,
    input  wire         rst_n,              // asynchronous assert, active low
    input  wire         bypass,             // 1 = V1 behaviour (asynchronous switch)

    input  wire         measurement_valid,  // one-clock pulse per new reading
    input  wire  [15:0] measurement_mm,     // that reading, in mm

    // To navigation and the motor gate (same names and meaning as V1)
    output logic        sonar_valid,
    output logic [15:0] distance_mm,        // filtered range; 0 while sonar_valid is low
    output logic        sonar_fresh,        // valid and the latest measurement was an echo
    output logic        forward_clear,      // safe to drive forward on range grounds

    // Diagnostics (LEDs, SignalTap, V2-DIAG overlay)
    output logic        update,             // one-clock pulse: a reading was processed
    output logic [15:0] raw_mm,             // actual measured range of the last reading, unfiltered
    output logic [1:0]  window_count,       // in-range readings held (0..2)
    output logic        close_suspect,      // forward paused on an unconfirmed close reading
    output logic        fault_no_echo,      // last invalidation: MAX_MISSES misses in a row
    output logic        fault_stale,        // last invalidation: no in-range reading in time
    output logic [15:0] rejected_count,     // outliers replaced by the median (saturating)
    output logic [15:0] bridged_count,      // single misses bridged while valid (saturating)
    output logic        bypass_active       // the synchronised bypass switch
);

    // ------------------------------------------------------------------
    // Sizes
    // ------------------------------------------------------------------

    localparam int AGE_W  = $clog2(MAX_AGE_CYCLES + 1);
    localparam int MISS_W = (MAX_MISSES > 1) ? $clog2(MAX_MISSES) : 1;

    localparam logic [AGE_W-1:0]  AGE_LAST  = AGE_W'(MAX_AGE_CYCLES - 1);
    localparam logic [MISS_W-1:0] MISS_LAST = MISS_W'(MAX_MISSES - 1);

    // ------------------------------------------------------------------
    // Bypass switch synchroniser
    // ------------------------------------------------------------------

    (* async_reg = "true" *) logic bypass_meta;
    (* async_reg = "true" *) logic bypass_sync;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bypass_meta <= 1'b0;
            bypass_sync <= 1'b0;
        end else begin
            bypass_meta <= bypass;
            bypass_sync <= bypass_meta;
        end
    end

    assign bypass_active = bypass_sync;

    // ------------------------------------------------------------------
    // V1 path: the unmodified V1 module, used only when bypassed
    // ------------------------------------------------------------------

    logic        v1_valid;
    logic [15:0] v1_distance_mm;

    sonar_freshness #(
        .MAX_AGE_CYCLES(MAX_AGE_CYCLES),
        .MIN_RANGE_MM(MIN_RANGE_MM),
        .MAX_RANGE_MM(MAX_RANGE_MM)
    ) u_v1_freshness (
        .clk(clk),
        .rst_n(rst_n),
        .measurement_valid(measurement_valid),
        .measurement_mm(measurement_mm),
        .sonar_valid(v1_valid),
        .distance_mm(v1_distance_mm)
    );

    // ------------------------------------------------------------------
    // 1. Classify the incoming reading
    // ------------------------------------------------------------------

    wire        is_miss   = (measurement_mm > MAX_RANGE_MM);
    wire        too_close = (measurement_mm < MIN_RANGE_MM);
    wire [15:0] sample_mm = too_close ? MIN_RANGE_MM : measurement_mm;

    // ------------------------------------------------------------------
    // 2-3. Median of three, then the outlier test
    // ------------------------------------------------------------------

    // Window of the two previous in-range readings (w0 newest).
    logic [15:0] w0_mm;
    logic [15:0] w1_mm;
    logic [1:0]  wcount;

    wire window_full = (wcount == 2'd2);

    // Median of three: c clamped into [min(a,b), max(a,b)].
    function automatic logic [15:0] median3(
        input logic [15:0] a,
        input logic [15:0] b,
        input logic [15:0] c
    );
        logic [15:0] lo;
        logic [15:0] hi;
        lo = (a < b) ? a : b;
        hi = (a < b) ? b : a;
        if (c < lo)
            median3 = lo;
        else if (c > hi)
            median3 = hi;
        else
            median3 = c;
    endfunction

    wire [15:0] med_mm = median3(w1_mm, w0_mm, sample_mm);

    wire [15:0] dev_mm = (sample_mm >= med_mm) ? (sample_mm - med_mm)
                                               : (med_mm - sample_mm);

    wire outlier       = (dev_mm > OUTLIER_MM);
    wire [15:0] new_mm = outlier ? med_mm : sample_mm;

    // An unconfirmed reading that would require a stop if it were real.
    wire close_outlier = outlier &&
                         (sample_mm < med_mm) &&
                         (sample_mm <= STOP_MM);

    // ------------------------------------------------------------------
    // 4-6. Misses, staleness and the window
    // ------------------------------------------------------------------

    logic [AGE_W-1:0]  age;       // clocks since the last in-range reading
    logic [MISS_W-1:0] misses;    // consecutive misses so far

    logic        f_valid;
    logic        f_fresh;         // latest measurement was an in-range echo
    logic [15:0] f_distance_mm;

    wire age_expired = (age >= AGE_LAST);
    wire miss_limit  = (misses >= MISS_LAST);   // this miss is the MAX_MISSES-th

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w0_mm          <= 16'd0;
            w1_mm          <= 16'd0;
            wcount         <= 2'd0;
            age            <= '0;
            misses         <= '0;
            f_valid        <= 1'b0;
            f_fresh        <= 1'b0;
            f_distance_mm  <= 16'd0;
            close_suspect  <= 1'b0;
            fault_no_echo  <= 1'b0;
            fault_stale    <= 1'b0;
            rejected_count <= 16'd0;
            bridged_count  <= 16'd0;
            raw_mm         <= 16'd0;
            update         <= 1'b0;

        end else begin
            update <= 1'b0;

            if (measurement_valid) begin
                update <= 1'b1;
                raw_mm <= measurement_mm;

                if (is_miss) begin
                    // A miss is not evidence of range: the age keeps running.
                    if (!age_expired)
                        age <= age + 1'b1;

                    if (miss_limit) begin
                        // MAX_MISSES in a row: invalidate and empty the window.
                        f_valid       <= 1'b0;
                        f_fresh       <= 1'b0;
                        f_distance_mm <= 16'd0;
                        close_suspect <= 1'b0;
                        wcount        <= 2'd0;
                        w0_mm         <= 16'd0;
                        w1_mm         <= 16'd0;
                        misses        <= '0;
                        fault_no_echo <= 1'b1;
                        fault_stale   <= 1'b0;
                    end else if (age_expired) begin
                        f_valid       <= 1'b0;
                        f_fresh       <= 1'b0;
                        f_distance_mm <= 16'd0;
                        close_suspect <= 1'b0;
                        wcount        <= 2'd0;
                        w0_mm         <= 16'd0;
                        w1_mm         <= 16'd0;
                        misses        <= '0;
                        fault_no_echo <= 1'b0;
                        fault_stale   <= 1'b1;
                    end else begin
                        // Bridge it: hold the last distance, but it is no
                        // longer a fresh observation.
                        misses  <= misses + 1'b1;
                        f_fresh <= 1'b0;
                        if (f_valid && (bridged_count != 16'hFFFF))
                            bridged_count <= bridged_count + 16'd1;
                    end

                end else begin
                    // In-range (or clamped) reading.
                    age    <= '0;
                    misses <= '0;

                    w1_mm <= w0_mm;
                    w0_mm <= sample_mm;

                    if (!window_full)
                        wcount <= wcount + 2'd1;

                    if (window_full) begin
                        f_valid       <= 1'b1;
                        f_fresh       <= 1'b1;
                        f_distance_mm <= new_mm;
                        close_suspect <= close_outlier;
                        fault_no_echo <= 1'b0;
                        fault_stale   <= 1'b0;

                        if (outlier && (rejected_count != 16'hFFFF))
                            rejected_count <= rejected_count + 16'd1;
                    end
                end

            end else if (age_expired) begin
                // No in-range reading for MAX_AGE_CYCLES.
                if (f_valid || (wcount != 2'd0)) begin
                    f_valid       <= 1'b0;
                    f_fresh       <= 1'b0;
                    f_distance_mm <= 16'd0;
                    close_suspect <= 1'b0;
                    wcount        <= 2'd0;
                    w0_mm         <= 16'd0;
                    w1_mm         <= 16'd0;
                    misses        <= '0;
                    fault_no_echo <= 1'b0;
                    fault_stale   <= 1'b1;
                end

            end else begin
                age <= age + 1'b1;
            end
        end
    end

    assign window_count = wcount;

    // ------------------------------------------------------------------
    // Outputs: V2 or, when bypassed, V1
    // ------------------------------------------------------------------

    always_comb begin
        if (bypass_active) begin
            sonar_valid   = v1_valid;
            distance_mm   = v1_distance_mm;
            sonar_fresh   = v1_valid;          // V1 never holds a value
            forward_clear = v1_valid && (v1_distance_mm > STOP_MM);
        end else begin
            sonar_valid   = f_valid;
            distance_mm   = f_distance_mm;
            sonar_fresh   = f_valid && f_fresh;
            forward_clear = f_valid &&
                            (f_distance_mm > STOP_MM) &&
                            !close_suspect;
        end
    end

endmodule

`default_nettype wire
