`timescale 1ns/1ps
`default_nettype none

module multi_bay_sequencer_tb;
    logic clk;

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end
    logic rst_n = 1'b0, arm = 1'b0, emergency_stop = 1'b0;
    logic bay_led_ready = 1'b0, frame_done = 1'b0, target_valid = 1'b0;
    logic [2:0] dock_state = 3'd0;
    logic sonar_valid = 1'b1;
    logic [15:0] distance_mm = 16'd600;
    logic reverse_active = 1'b0, reverse_fault = 1'b0, reverse_timeout = 1'b0;
    wire [1:0] bay_index;
    wire dock_enable, dock_restart, nav_rev_request;
    wire signed [15:0] nav_rev_left_pwm, nav_rev_right_pwm;
    wire sequence_complete, sequence_fault;
    wire [3:0] state_debug;

    localparam logic [3:0]
        S_WAIT_LED=4'd1, S_WAIT_TARGET=4'd2, S_DOCKING=4'd3,
        S_CONFIRM=4'd4, S_REV_START=4'd5, S_REV_RUN=4'd6,
        S_REV_STOP=4'd7, S_COMPLETE=4'd10, S_FAULT=4'd11;

    multi_bay_sequencer #(
        .CLK_HZ(1000), .NUM_BAYS(3), .REVERSE_PWM(50),
        .REVERSE_MS(8), .REVERSE_START_TIMEOUT_MS(10),
        .REVERSE_STOP_TIMEOUT_MS(10), .SETTLE_MS(4),
        .DOCK_CONFIRM_MS(3), .LED_TIMEOUT_MS(20),
        .TARGET_TIMEOUT_MS(20), .DOCK_TIMEOUT_MS(40)
    ) dut (
        .clk(clk), .rst_n(rst_n), .arm(arm), .emergency_stop(emergency_stop),
        .bay_led_ready(bay_led_ready), .frame_done(frame_done),
        .target_valid(target_valid), .dock_state(dock_state),
        .sonar_valid(sonar_valid), .distance_mm(distance_mm),
        .reverse_active(reverse_active), .reverse_fault(reverse_fault),
        .reverse_timeout(reverse_timeout), .bay_index(bay_index),
        .dock_enable(dock_enable), .dock_restart(dock_restart),
        .nav_rev_request(nav_rev_request), .nav_rev_left_pwm(nav_rev_left_pwm),
        .nav_rev_right_pwm(nav_rev_right_pwm),
        .sequence_complete(sequence_complete), .sequence_fault(sequence_fault),
        .state_debug(state_debug)
    );

    int checks = 0;
    task automatic tick(input int cycles);
        repeat (cycles) begin @(posedge clk); #1; end
    endtask
    task automatic check(input logic condition, input string description);
        if (!condition) $fatal(1, "FAIL: %s (state=%0d, bay=%0d)", description, state_debug, bay_index);
        checks++;
        $display("PASS [%0d]: %s", checks, description);
    endtask
    task automatic await_state(input logic [3:0] wanted, input int max_cycles);
        for (int i=0; i<max_cycles; i++) begin
            if (state_debug == wanted) return;
            tick(1);
        end
        $fatal(1, "TIMEOUT waiting for state %0d; got %0d, bay=%0d", wanted, state_debug, bay_index);
    endtask
    task automatic reset_system();
        @(negedge clk);
        rst_n=0; arm=0; emergency_stop=0;
        bay_led_ready=0; frame_done=0; target_valid=0;
        dock_state=0; sonar_valid=1; distance_mm=600;
        reverse_active=0; reverse_fault=0; reverse_timeout=0;
        tick(3);
        @(negedge clk); rst_n=1; arm=1;
        await_state(S_WAIT_LED, 10);
    endtask
    task automatic acquire_bay(input int expected_bay);
        await_state(S_WAIT_LED, 20);
        check(bay_index == 2'(expected_bay), $sformatf("Bay %0d selected", expected_bay));
        check(dock_restart && !dock_enable, "Docking held in reset before acquisition");
        @(negedge clk); bay_led_ready=0;
        tick(2);
        @(negedge clk); bay_led_ready=1;
        await_state(S_WAIT_TARGET, 10);
        check(!dock_enable, "No forward driving before acquisition");
        @(negedge clk); frame_done=1; target_valid=1;
        tick(1);
        @(negedge clk); frame_done=0;
        await_state(S_DOCKING, 10);
        check(dock_enable && !dock_restart, "Docking enabled after acquisition");
    endtask
    task automatic perform_docking();
        @(negedge clk); dock_state=3'd2; distance_mm=600;
        tick(3);
        @(negedge clk); dock_state=3'd3; distance_mm=240;
        await_state(S_CONFIRM, 10);
        check(!dock_enable, "Forward stopped during confirmation");
        check(!dock_restart, "DOCKED state held during confirmation");
    endtask
    task automatic perform_reverse();
        await_state(S_REV_START, 15);
        check(nav_rev_request, "Reverse request asserted");
        check(nav_rev_left_pwm == -16'sd50 && nav_rev_right_pwm == -16'sd50,
              "Reverse request bounded");
        @(negedge clk); reverse_active=1;
        await_state(S_REV_RUN, 10);
        await_state(S_REV_STOP, 20);
        check(!nav_rev_request, "Reverse withdrawn after retreat");
        @(negedge clk); reverse_active=0;
        await_state(S_WAIT_LED, 20);
        check(!nav_rev_request && nav_rev_left_pwm == 0 && nav_rev_right_pwm == 0,
              "Reverse cleared before next bay");
    endtask

    initial begin
        $display("=== V3-C1 multi-bay sequencer tests ===");
        reset_system();
        acquire_bay(0); perform_docking(); perform_reverse();
        acquire_bay(1); perform_docking(); perform_reverse();
        acquire_bay(2); perform_docking();
        await_state(S_COMPLETE, 15);
        check(sequence_complete && !sequence_fault, "Three-bay sequence completed");
        check(!dock_enable && !nav_rev_request, "All movement stopped after completion");
        tick(10);
        check(sequence_complete && bay_index == 2'd2, "Completion latched at Bay 3");

        reset_system();
        acquire_bay(0); perform_docking();
        await_state(S_REV_START, 15);
        @(negedge clk); reverse_fault=1;
        await_state(S_FAULT, 10);
        check(sequence_fault && !nav_rev_request, "Reverse fault cancels movement");

        reset_system();
        await_state(S_FAULT, 30);
        check(sequence_fault && dock_restart, "Missing LED acknowledgement faults");

        reset_system();
        @(negedge clk); bay_led_ready=0;
        tick(2);
        @(negedge clk); bay_led_ready=1;
        await_state(S_WAIT_TARGET, 10);
        await_state(S_FAULT, 30);
        check(sequence_fault && !dock_enable, "Missing target faults without movement");

        reset_system(); acquire_bay(0);
        @(negedge clk); emergency_stop=1;
        #1;
        check(!dock_enable && !nav_rev_request, "Emergency stop disables movement immediately");
        await_state(S_FAULT, 10);
        check(sequence_fault, "Emergency stop latches fault");

        reset_system(); acquire_bay(0); perform_docking();
        await_state(S_REV_START, 15);
        @(negedge clk); reverse_active=1;
        await_state(S_REV_RUN, 10);
        @(negedge clk); reverse_timeout=1;
        await_state(S_FAULT, 10);
        check(sequence_fault && !nav_rev_request, "Reverse timeout cannot advance bay");

        $display("PASS: multi_bay_sequencer_tb (%0d checks)", checks);
        $finish;
    end
    initial begin
        #100000;
        $fatal(1, "TIMEOUT: multi_bay_sequencer_tb");
    end
endmodule

`default_nettype wire
