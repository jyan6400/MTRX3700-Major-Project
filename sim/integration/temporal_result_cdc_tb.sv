`timescale 1ns/1ps
module temporal_result_cdc_tb;
    logic src_clk=0, dst_clk=0;
    always #13 src_clk=~src_clk; // 38.46 MHz PCLK example
    always #10 dst_clk=~dst_clk; // 50 MHz system clock
    logic rst_n=0, src_valid=0, src_target_valid=0;
    logic [8:0] src_x=0;
    logic [7:0] src_y=0;
    logic [16:0] src_count=0;
    wire src_ready, src_overrun, dst_valid, dst_target_valid;
    wire [8:0] dst_x;
    wire [7:0] dst_y;
    wire [16:0] dst_count;
    int received=0, overruns=0;
    temporal_result_cdc dut(
       .src_clk,.src_rst_n(rst_n),.src_valid,.src_target_valid,.src_x,.src_y,.src_count,
       .src_ready,.src_overrun,.dst_clk,.dst_rst_n(rst_n),
       .dst_valid,.dst_target_valid,.dst_x,.dst_y,.dst_count
    );
    always @(posedge src_clk) if (rst_n && src_overrun) overruns++;
    always @(posedge dst_clk) if (rst_n && dst_valid) begin
        case (received)
            0: if (!(dst_target_valid===1'b1 && dst_x===9'd42 && dst_y===8'd31 && dst_count===17'd500)) $fatal(1,"payload A mismatch");
            1: if (!(dst_target_valid===1'b0 && dst_x===9'd0 && dst_y===8'd0 && dst_count===17'd0)) $fatal(1,"payload B mismatch");
            2: if (!(dst_target_valid===1'b1 && dst_x===9'd319 && dst_y===8'd239 && dst_count===17'd76800)) $fatal(1,"payload C mismatch");
            default: $fatal(1,"unexpected extra result");
        endcase
        received++;
        $display("PASS CDC result %0d: valid=%b xy=(%0d,%0d) count=%0d", received,dst_target_valid,dst_x,dst_y,dst_count);
    end
    task automatic send(input bit tv,input logic[8:0] x,input logic[7:0] y,input logic[16:0] count);
        do @(negedge src_clk); while (!src_ready);
        src_valid=1; src_target_valid=tv; src_x=x; src_y=y; src_count=count;
        @(negedge src_clk); src_valid=0;
    endtask
    initial begin
        #71; rst_n=1;
        send(1,42,31,500);
        // Deliberately attempt a second event before acknowledgement.
        @(negedge src_clk); if (src_ready) $fatal(1,"mailbox unexpectedly ready");
        src_valid=1; src_target_valid=1; src_x=9'd5; src_y=8'd5; src_count=17'd5;
        @(negedge src_clk); src_valid=0;
        send(0,0,0,0);
        send(1,319,239,76800);
        wait(received==3);
        repeat (8) @(negedge dst_clk);
        if (overruns!=1) $fatal(1,"expected 1 dropped-busy event, saw %0d",overruns);
        $display("PASS V3-D10B Stage 3 coherent CDC 3 delivered, 1 busy drop");
        $finish;
    end
    initial begin #100000; $fatal(1,"CDC test timeout"); end
endmodule
