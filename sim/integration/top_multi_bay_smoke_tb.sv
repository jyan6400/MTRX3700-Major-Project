`timescale 1ns/1ps
`default_nettype none

// V3-C2 top-level smoke test: real sequencer/top-level wiring.
// This does NOT replace a three-bay UART integration test.
module top_multi_bay_smoke_tb;
    logic CLOCK_50;
    initial begin
        CLOCK_50 = 1'b0;
        forever #10 CLOCK_50 = ~CLOCK_50;
    end
    logic cam_pclk;
    initial begin
        cam_pclk = 1'b0;
        forever #12.5 cam_pclk = ~cam_pclk;
    end
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

    major_project_top dut (
        .CLOCK_50(CLOCK_50), .KEY(KEY), .SW(SW),
        .IRDA_RXD(IRDA_RXD), .GPIO_1(GPIO_1), .LEDR(LEDR)
    );

    int checks;
    task automatic tick(input int n);
        repeat (n) begin @(posedge CLOCK_50); #1; end
    endtask
    task automatic wait_seq(input logic [3:0] s, input int n);
        for (int i=0; i<n; i++) begin
            if (dut.seq_state === s) return;
            tick(1);
        end
        $fatal(1,"FAIL: expected seq_state=%0d, got=%0d", s, dut.seq_state);
    endtask
    task automatic check(input bit cond, input string name);
        if (!cond) $fatal(1,"FAIL: %s",name);
        checks++;
        $display("PASS [%0d]: %s",checks,name);
    endtask

    initial begin
        checks=0;
        KEY=4'b1110;
        SW=10'b0;
        IRDA_RXD=1'b1;
        // Stabilise sonar and prohibit IR stop through existing hierarchies.
        force dut.sonar_valid=1'b1;
        force dut.distance_mm=16'd600;
        force dut.sonar_forward_clear=1'b1;
        force dut.emergency_stop=1'b0;
        force dut.motion_permitted=1'b1;
        force dut.nav_frame_done=1'b0;
        force dut.nav_target_valid=1'b0;
        force dut.camera_ok=1'b1;

        tick(10);
        KEY[0]=1'b1;
        SW[0]=1'b1;
        SW[5]=1'b1;
        tick(50);
        wait_seq(4'd1, 50); // WAIT_LED
        check(dut.seq_bay_index==2'd0,"bay 0 initially selected");
        check(dut.seq_dock_restart && !dut.seq_dock_enable,
              "V2 docking reset until acknowledgement and target");
        check(!dut.nav_rev_request && !dut.safe_motor_enable,
              "no chassis movement before acknowledgement");

        // SW6 must be observed low before a rising acknowledgement.
        SW[6]=1'b1;
        wait_seq(4'd2, 50); // WAIT_TARGET
        check(!dut.seq_dock_enable && !dut.nav_rev_request,
              "LED ready alone cannot start driving");

        // Camera target arrives for current bay (mock visual result).
        @(negedge CLOCK_50);
        force dut.nav_target_valid=1'b1;
        force dut.nav_frame_done=1'b1;
        wait_seq(4'd3, 30); // DOCKING
        check(dut.seq_dock_enable && !dut.seq_dock_restart,
              "valid frame enables local V2 docking controller");
        check(dut.docking_local_rst_n && dut.docking_local_arm,
              "local docking reset released and arm asserted");
        @(negedge CLOCK_50);
        force dut.nav_frame_done=1'b0;

        // Disarm must immediately suppress requests, independent of camera.
        SW[0]=1'b0;
        tick(5);
        check(!dut.safe_motor_enable && !dut.nav_rev_request,
              "disarm suppresses chassis enable and reverse request");

        $display("PASS: top_multi_bay_smoke_tb (%0d checks)",checks);
        $finish;
    end
    initial begin
        #1000000;
        $fatal(1,"TIMEOUT: top_multi_bay_smoke_tb");
    end
endmodule
`default_nettype wire
