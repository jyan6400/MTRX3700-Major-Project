
`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Major Project - Group 26
// V3-C1: Three-bay docking sequencer
//
// This module coordinates the existing V2 docking controller.
// It does not directly drive the chassis UART.
//
// Bay numbering: 0, 1, 2.
//
// Required integration:
//   dock_restart: locally reset the V2 motion_interface
//   dock_enable:  gate the V2 docking controller
//   nav_rev_*:    connect to motion_supervisor
//   bay_index:    select the active LED via the ESP32 controller
//
// LED handshake:
//   For each new bay, bay_led_ready must first be observed LOW,
//   then HIGH. This prevents a stale acknowledgement from a
//   previous bay from being accepted.
//
// Reverse:
//   The sequencer requests a bounded reverse shorter than the
//   motion_supervisor's independent 1500 ms maximum.
//   A timeout or fault NEVER counts as a successful reverse.

module multi_bay_sequencer #(
    parameter int unsigned CLK_HZ = 50_000_000,
    parameter int unsigned NUM_BAYS = 3,
    parameter int unsigned REVERSE_PWM = 50,
    parameter int unsigned REVERSE_MS = 650,
    parameter int unsigned REVERSE_START_TIMEOUT_MS = 250,
    parameter int unsigned REVERSE_STOP_TIMEOUT_MS = 250,
    parameter int unsigned SETTLE_MS = 150,
    parameter int unsigned DOCK_CONFIRM_MS = 100,
    parameter int unsigned LED_TIMEOUT_MS = 2000,
    parameter int unsigned TARGET_TIMEOUT_MS = 3000,
    parameter int unsigned DOCK_TIMEOUT_MS = 15000,
    parameter logic [15:0] DOCK_DISTANCE_MM = 16'd250
) (
    input  wire clk,
    input  wire rst_n,
    input  wire arm,
    input  wire emergency_stop,

    // Bay LED selection acknowledgement.
    // Eventually supplied by the ESP32 interface.
    input  wire bay_led_ready,

    // Camera detection for the selected bay.
    input  wire frame_done,
    input  wire target_valid,

    // Existing V2 docking controller.
    // IDLE=0, ACQUIRE=1, APPROACH=2, DOCKED=3, STOPPED=4.
    input  wire [2:0] dock_state,

    // Ultrasonic distance confirmation.
    input  wire sonar_valid,
    input  wire [15:0] distance_mm,

    // Advay's motion supervisor.
    input  wire reverse_active,
    input  wire reverse_fault,
    input  wire reverse_timeout,

    // Current bay: 0, 1, 2.
    output logic [1:0] bay_index,

    // Local V2 docking control.
    output logic dock_enable,
    output logic dock_restart,

    // Reverse command to motion_supervisor.
    output logic nav_rev_request,
    output logic signed [15:0] nav_rev_left_pwm,
    output logic signed [15:0] nav_rev_right_pwm,

    // Sequence status.
    output logic sequence_complete,
    output logic sequence_fault,
    output logic [3:0] state_debug
);

    localparam int unsigned CYC_PER_MS =
        (CLK_HZ >= 1000) ? CLK_HZ / 1000 : 1;

    localparam int unsigned LED_CYC =
        LED_TIMEOUT_MS * CYC_PER_MS;
    localparam int unsigned TARGET_CYC =
        TARGET_TIMEOUT_MS * CYC_PER_MS;
    localparam int unsigned DOCK_CYC =
        DOCK_TIMEOUT_MS * CYC_PER_MS;
    localparam int unsigned CONFIRM_CYC =
        DOCK_CONFIRM_MS * CYC_PER_MS;
    localparam int unsigned REV_CYC =
        REVERSE_MS * CYC_PER_MS;
    localparam int unsigned REV_START_CYC =
        REVERSE_START_TIMEOUT_MS * CYC_PER_MS;
    localparam int unsigned REV_STOP_CYC =
        REVERSE_STOP_TIMEOUT_MS * CYC_PER_MS;
    localparam int unsigned SETTLE_CYC =
        SETTLE_MS * CYC_PER_MS;

    typedef enum logic [3:0] {
        S_IDLE        = 4'd0,
        S_WAIT_LED    = 4'd1,
        S_WAIT_TARGET = 4'd2,
        S_DOCKING     = 4'd3,
        S_CONFIRM     = 4'd4,
        S_REV_START   = 4'd5,
        S_REV_RUN     = 4'd6,
        S_REV_STOP    = 4'd7,
        S_SETTLE      = 4'd8,
        S_NEXT_BAY    = 4'd9,
        S_COMPLETE    = 4'd10,
        S_FAULT       = 4'd11
    } state_t;

    state_t state;
    state_t next_state;

    logic [31:0] timer;
    logic ready_low_seen;
    logic saw_approach;

    wire dock_close =
        sonar_valid && (distance_mm <= DOCK_DISTANCE_MM);

    wire reverse_failed = reverse_fault || reverse_timeout;

    // All movement requests are suppressed immediately on
    // disarm, emergency stop or reset.
    wire operational = rst_n && arm && !emergency_stop;

    // -------------------------------------------------------
    // State transitions
    // -------------------------------------------------------

    always_comb begin
        next_state = state;

        if (!arm)
            next_state = S_IDLE;
        else if (emergency_stop)
            next_state = S_FAULT;
        else begin
            case (state)

                S_IDLE:
                    next_state = S_WAIT_LED;

                S_WAIT_LED: begin
                    if (timer >= LED_CYC)
                        next_state = S_FAULT;
                    else if (ready_low_seen && bay_led_ready)
                        next_state = S_WAIT_TARGET;
                end

                S_WAIT_TARGET: begin
                    if (!bay_led_ready ||
                        timer >= TARGET_CYC)
                        next_state = S_FAULT;
                    else if (frame_done && target_valid)
                        next_state = S_DOCKING;
                end

                S_DOCKING: begin
                    if (!bay_led_ready ||
                        timer >= DOCK_CYC)
                        next_state = S_FAULT;
                    else if (saw_approach &&
                             dock_state == 3'd3 &&
                             dock_close)
                        next_state = S_CONFIRM;
                end

                S_CONFIRM: begin
                    if (!bay_led_ready || !dock_close)
                        next_state = S_FAULT;
                    else if (timer >= CONFIRM_CYC) begin
                        if (int'(bay_index) == NUM_BAYS - 1)
                            next_state = S_COMPLETE;
                        else
                            next_state = S_REV_START;
                    end
                end

                S_REV_START: begin
                    if (reverse_failed ||
                        timer >= REV_START_CYC)
                        next_state = S_FAULT;
                    else if (reverse_active)
                        next_state = S_REV_RUN;
                end

                S_REV_RUN: begin
                    if (reverse_failed || !reverse_active)
                        next_state = S_FAULT;
                    else if (timer >= REV_CYC)
                        next_state = S_REV_STOP;
                end

                S_REV_STOP: begin
                    if (reverse_failed ||
                        timer >= REV_STOP_CYC)
                        next_state = S_FAULT;
                    else if (!reverse_active)
                        next_state = S_SETTLE;
                end

                S_SETTLE: begin
                    if (reverse_failed)
                        next_state = S_FAULT;
                    else if (timer >= SETTLE_CYC)
                        next_state = S_NEXT_BAY;
                end

                S_NEXT_BAY:
                    next_state = S_WAIT_LED;

                S_COMPLETE:
                    next_state = S_COMPLETE;

                S_FAULT:
                    next_state = S_FAULT;

                default:
                    next_state = S_FAULT;

            endcase
        end
    end

    // -------------------------------------------------------
    // Sequential state and bookkeeping
    // -------------------------------------------------------

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            timer          <= 32'd0;
            bay_index      <= 2'd0;
            ready_low_seen <= 1'b0;
            saw_approach   <= 1'b0;
        end else begin
            state <= next_state;

            if (next_state != state)
                timer <= 32'd0;
            else if (timer != 32'hFFFFFFFF)
                timer <= timer + 1'b1;

            if (state == S_IDLE) begin
                bay_index      <= 2'd0;
                ready_low_seen <= 1'b0;
                saw_approach   <= 1'b0;
            end else begin

                if (state == S_WAIT_LED &&
                    !bay_led_ready)
                    ready_low_seen <= 1'b1;

                if (state == S_DOCKING &&
                    dock_state == 3'd2)
                    saw_approach <= 1'b1;

                if (state == S_NEXT_BAY) begin
                    bay_index      <= bay_index + 1'b1;
                    ready_low_seen <= 1'b0;
                    saw_approach   <= 1'b0;
                end

                // Disarming starts a new sequence when
                // rearmed. A reset or disarm is necessary
                // to clear a completed or faulted run.
                if (!arm) begin
                    bay_index      <= 2'd0;
                    ready_low_seen <= 1'b0;
                    saw_approach   <= 1'b0;
                end
            end
        end
    end

    // -------------------------------------------------------
    // Outputs
    // -------------------------------------------------------

    always_comb begin
        dock_enable = 1'b0;

        // Hold the V2 docking controller in local reset
        // until the selected bay's target is acquired.
        dock_restart = 1'b1;

        nav_rev_request   = 1'b0;
        nav_rev_left_pwm  = 16'sd0;
        nav_rev_right_pwm = 16'sd0;

        sequence_complete = (state == S_COMPLETE);
        sequence_fault    = (state == S_FAULT);
        state_debug       = state;

        if (operational) begin
            case (state)
                S_DOCKING: begin
                    dock_enable  = 1'b1;
                    dock_restart = 1'b0;
                end

                S_CONFIRM: begin
                    // Stop further approach, but keep the
                    // V2 DOCKED state intact.
                    dock_restart = 1'b0;
                end

                S_REV_START,
                S_REV_RUN: begin
                    nav_rev_request   = 1'b1;
                    nav_rev_left_pwm  = -16'(REVERSE_PWM);
                    nav_rev_right_pwm = -16'(REVERSE_PWM);
                end

                default: begin
                    // All other states command zero.
                end
            endcase
        end
    end

endmodule

`default_nettype wire
