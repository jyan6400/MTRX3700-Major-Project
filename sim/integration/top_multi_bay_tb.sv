`timescale 1ns/1ps
`default_nettype none

// V3-C2 integration: real top-level, docking_fsm, motion_supervisor.
// Controlled camera/sonar/LED stimuli; long sequencing timers accelerated.
// This is NOT a real LED-identity, UART-on-wire, or physical-distance test.
module top_multi_bay_tb;
    logic CLOCK_50;
    initial begin CLOCK_50 = 1'b0; forever #10 CLOCK_50 = ~CLOCK_50; end
    logic cam_pclk;
    initial begin cam_pclk = 1'b0; forever #12.5 cam_pclk = ~cam_pclk; end
    logic [3:0] KEY;
    logic [9:0] SW;
    logic IRDA_RXD;
    tri [35:0] GPIO_1;
    assign GPIO_1[21] = cam_pclk;
    assign GPIO_1[22] = 1'b0;
    assign GPIO_1[23] = 1'b1;
    assign GPIO_1[19:12] = 8'd0;
    assign GPIO_1[29] = 1'b0;
    assign GPIO_1[24] = 1'bz;
    wire [9:0] LEDR;

    major_project_top #(
        .REVERSE_DWELL_MS(0)
    ) dut (
        .CLOCK_50(CLOCK_50), .KEY(KEY), .SW(SW),
        .IRDA_RXD(IRDA_RXD), .GPIO_1(GPIO_1), .LEDR(LEDR)
    );

    localparam logic [3:0] WAIT_LED=4'd1, WAIT_TARGET=4'd2,
        DOCKING=4'd3, CONFIRM=4'd4, REV_START=4'd5,
        REV_RUN=4'd6, REV_STOP=4'd7, SETTLE=4'd8,
        NEXT_BAY=4'd9, COMPLETE=4'd10, FAULT=4'd11;
    int checks=0;
    int reverse_sessions=0;

    task automatic tick(input int n);
        repeat(n) begin @(posedge CLOCK_50); #1; end
    endtask
    task automatic check(input bit cond, input string description);
        if (!cond) $fatal(1,"FAIL %s: seq=%0d bay=%0d dock=%0d rev=%b fault=%b",
          description,dut.seq_state,dut.seq_bay_index,dut.state_debug,
          dut.reverse_active,dut.seq_fault);
        checks++;
        $display("PASS [%0d]: %s",checks,description);
    endtask
    task automatic wait_seq(input logic [3:0] wanted, input int maxclks);
        for(int k=0;k<maxclks;k++) begin
            if(dut.seq_state === wanted) return;
            tick(1);
        end
        $fatal(1,"TIMEOUT waiting for seq=%0d (got %0d bay=%0d dock=%0d)",
          wanted,dut.seq_state,dut.seq_bay_index,dut.state_debug);
    endtask
    task automatic pulse_target();
        @(negedge CLOCK_50);
        force dut.nav_target_valid=1'b1;
        force dut.nav_frame_done=1'b1;
        tick(3);
        @(negedge CLOCK_50);
        force dut.nav_frame_done=1'b0;
    endtask
    task automatic accelerate_timer(input int unsigned count_value);
        @(negedge CLOCK_50);
        force dut.u_multi_bay_sequencer.timer = count_value;
        tick(1);
        @(negedge CLOCK_50);
        force dut.u_multi_bay_sequencer.timer = 32'd0;
        #1;
        release dut.u_multi_bay_sequencer.timer;
    endtask
    task automatic acquire(input int index);
        wait_seq(WAIT_LED,100);
        check(dut.seq_bay_index == 2'(index), "selected expected bay");
        check(dut.seq_dock_restart && !dut.safe_motor_enable,
              "local controller reset and chassis stopped before new target");
        // SW6 must be LOW first, then HIGH per bay.
        SW[6]=1'b0;
        tick(5);
        SW[6]=1'b1;
        wait_seq(WAIT_TARGET,30);
        check(!dut.seq_dock_enable,"LED-ready alone does not authorise docking");
        pulse_target();
        wait_seq(DOCKING,30);
        check(dut.docking_local_rst_n && dut.docking_local_arm,
              "local V2 controller re-enabled");
        // Feed a distinct valid frame to the actual V2 docking FSM.
        pulse_target();
        for(int j=0;j<30;j++) begin
            if (dut.state_debug == 3'd2) break;
            tick(1);
        end
        check(dut.state_debug == 3'd2,"real V2 docking FSM entered APPROACH");
        tick(3);
    endtask
    task automatic dock();
        @(negedge CLOCK_50);
        force dut.distance_mm=16'd240;
        wait_seq(CONFIRM,50);
        check(dut.state_debug == 3'd3,"real V2 docking FSM reached DOCKED");
        check(!dut.safe_motor_enable && !dut.nav_rev_request,
              "motors stopped in dock confirmation");
        // 100 ms physical confirmation timer is accelerated only in this bench.
        accelerate_timer(32'd5000000);
    endtask
    task automatic reverse_and_next(input int next_index);
        wait_seq(REV_START,30);
        check(dut.nav_rev_request,"reverse requested by real sequencer");
        $display("REVERSE DEBUG: request=%b active=%b mode=%0d status=%0d permitted=%b dwell_count=%0d",
            dut.nav_rev_request,
            dut.reverse_active,
            dut.reverse_mode,
            dut.reverse_status,
            dut.reverse_permitted,
            dut.u_motion_supervisor.zero_cnt);
        wait_seq(REV_RUN,30);
        check(dut.reverse_active && dut.chassis_left == -10'sd50 &&
              dut.chassis_right == -10'sd50,
              "real supervisor produced bounded reverse commands");
        reverse_sessions++;
        // 650 ms navigation timer accelerated: supervisor remains real.
        accelerate_timer(32'd32500000);
        wait_seq(REV_STOP,30);
        check(!dut.nav_rev_request && !dut.reverse_active,
              "withdrawal immediately removes reverse permission");
        wait_seq(SETTLE,30);
        check(!dut.safe_motor_enable,"motors remain stopped during settle");
        accelerate_timer(32'd7500000); // 150 ms settle
        wait_seq(WAIT_LED,50);
        check(dut.seq_bay_index == 2'(next_index),"next bay selected after reverse");
        check(dut.docking_local_rst_n == 1'b0,
              "V2 docking controller reset between bays");
        // Reset simulated target and place sonar back at viewing distance.
        @(negedge CLOCK_50);
        force dut.nav_target_valid=1'b0;
        force dut.distance_mm=16'd600;
        SW[6]=1'b0;
        tick(5);
    endtask

    initial begin
        KEY=4'b1110;
        SW='0;
        IRDA_RXD=1'b1;
        force dut.sonar_valid=1'b1;
        force dut.distance_mm=16'd600;
        force dut.sonar_forward_clear=1'b1;
        force dut.emergency_stop=1'b0;
        force dut.motion_permitted=1'b1;
        force dut.nav_frame_done=1'b0;
        force dut.nav_target_valid=1'b0;
        force dut.nav_target_x=10'd320;
        force dut.camera_ok=1'b1;
        force dut.camera_target_fresh=1'b1;
        tick(10);
        KEY[0]=1'b1;
        SW[0]=1'b1; // arm
        SW[3]=1'b1; // manually cleared reverse corridor in simulation
        SW[5]=1'b1; // V3 sequencing
        tick(50);

        acquire(0);
        dock();
        reverse_and_next(1);
        acquire(1);
        dock();
        reverse_and_next(2);
        acquire(2);
        dock();
        wait_seq(COMPLETE,30);
        check(dut.seq_complete && !dut.seq_fault,"three-bay sequence COMPLETE");
        check(reverse_sessions==2,"exactly two reverse sessions");
        check(!dut.safe_motor_enable && !dut.nav_rev_request,
              "no fourth reverse; motors disabled after bay 3");
        tick(20);
        check(dut.seq_state==COMPLETE,"completion remains latched");
        SW[0]=1'b0;
        tick(5);
        check(!dut.safe_motor_enable,"disarm still stops chassis");
        $display("PASS: top_multi_bay_tb (%0d checks; %0d reverse sessions)",checks,reverse_sessions);
        $finish;
    end
    initial begin #10000000; $fatal(1,"TIMEOUT: top_multi_bay_tb"); end
endmodule
`default_nettype wire
