`timescale 1ns/1ps
`default_nettype none
module temporal_result_cdc_stage6a_tb;
    parameter int SRC_HALF = 13;
    parameter int DST_HALF = 10;
    localparam int PW=35;
    logic src_clk=0, dst_clk=0;
    always #(SRC_HALF) src_clk=~src_clk;
    always #(DST_HALF) dst_clk=~dst_clk;
    logic src_rst_n=0, dst_rst_n=0;
    logic src_valid=0, src_target_valid=0;
    logic [8:0] src_x=0;
    logic [7:0] src_y=0;
    logic [16:0] src_count=0;
    wire src_ready,src_overrun,dst_valid,dst_target_valid;
    wire [8:0] dst_x;
    wire [7:0] dst_y;
    wire [16:0] dst_count;
    logic [PW-1:0] expected [0:1023];
    integer wr=0,rd=0,accepted=0,delivered=0,dropped=0;
    logic [31:0] rng=32'hA51C0FFE;
    temporal_result_cdc dut (
        .src_clk,.src_rst_n,.src_valid,.src_target_valid,.src_x,.src_y,.src_count,
        .src_ready,.src_overrun,.dst_clk,.dst_rst_n,
        .dst_valid,.dst_target_valid,.dst_x,.dst_y,.dst_count
    );
    // Sample outputs on negative edge, after nonblocking updates have settled.
    always @(negedge dst_clk) if (src_rst_n && dst_rst_n && dst_valid) begin
        if (rd >= wr) $fatal(1,"Unexpected or duplicate destination transaction");
        if ({dst_target_valid,dst_x,dst_y,dst_count} !== expected[rd])
            $fatal(1,"CDC corruption index=%0d expected=%h actual=%h",rd,expected[rd],{dst_target_valid,dst_x,dst_y,dst_count});
        rd=rd+1;
        delivered=delivered+1;
    end
    // Stimulus changes only on source negative edges, no races with the DUT.
    task automatic offer(input logic [PW-1:0] payload, input bit should_accept);
        @(negedge src_clk);
        if (src_ready !== should_accept)
            $fatal(1,"Ready mismatch: expected=%0d got=%0b",should_accept,src_ready);
        if (should_accept) begin
            if (wr>=1024) $fatal(1,"Scoreboard full");
            expected[wr]=payload;
            wr=wr+1;
            accepted=accepted+1;
        end else dropped=dropped+1;
        {src_target_valid,src_x,src_y,src_count}=payload;
        src_valid=1;
        @(negedge src_clk);
        src_valid=0;
        if (src_overrun !== !should_accept)
            $fatal(1,"Overrun mismatch: expected=%0d got=%0b",!should_accept,src_overrun);
    endtask
    task automatic await_ready;
        integer k;
        begin
            k=0;
            while (src_ready !== 1'b1 && k<100) begin
                @(negedge src_clk);
                k=k+1;
            end
            if (src_ready !== 1'b1) $fatal(1,"Source mailbox never became ready");
        end
    endtask
    task automatic drain;
        integer k;
        begin
            k=0;
            while ((rd!=wr || !src_ready) && k<200) begin
                @(negedge dst_clk);
                k=k+1;
            end
            if (rd!=wr || !src_ready) $fatal(1,"Mailbox failed to drain expected=%0d received=%0d",wr,rd);
            repeat (6) @(negedge dst_clk);
            if (rd!=wr) $fatal(1,"Duplicate result after drain");
        end
    endtask
    task automatic common_reset(input bit source_first);
        begin
            // Both resets assert together, releases intentionally staggered.
            @(negedge src_clk);
            src_valid=0;
            src_rst_n=0;
            dst_rst_n=0;
            wr=0;
            rd=0;
            repeat (5) @(negedge dst_clk);
            if (source_first) begin
                src_rst_n=1;
                repeat (4) @(negedge dst_clk);
                dst_rst_n=1;
            end else begin
                dst_rst_n=1;
                repeat (4) @(negedge src_clk);
                src_rst_n=1;
            end
            repeat (6) @(negedge dst_clk);
            if (dst_valid !== 0) $fatal(1,"Unexpected destination valid after reset");
            await_ready();
        end
    endtask
    function automatic logic [31:0] next_rand(input logic [31:0] x);
        logic [31:0] y;
        begin
            y=x^(x<<13);
            y=y^(y>>17);
            next_rand=y^(y<<5);
        end
    endfunction
    initial begin : stimulus
        logic [PW-1:0] p;
        integer i;
        common_reset(1);
        offer({1'b1,9'd42,8'd31,17'd500},1);
        offer({1'b1,9'd5,8'd6,17'd7},0); // Busy offer must not overwrite payload.
        await_ready();
        offer({1'b0,9'd0,8'd0,17'd0},1);
        await_ready();
        offer({1'b1,9'h1ff,8'hff,17'h1ffff},1);
        drain();
        for (i=0;i<120;i=i+1) begin
            await_ready();
            rng=next_rand(rng);
            p={rng[0],rng[9:1],rng[17:10],rng[31:18],rng[2:0]};
            offer(p,1);
            if (i%3==0) begin
                rng=next_rand(rng);
                offer({rng[0],rng[9:1],rng[17:10],rng[16:0]},0);
            end
        end
        drain();
        // Reset in flight: intentionally abandon an unacknowledged transaction.
        await_ready();
        offer({1'b1,9'd123,8'd77,17'd1001},1);
        common_reset(0);
        offer({1'b1,9'd319,8'd239,17'd76800},1);
        drain();
        $display("PASS: temporal_result_cdc_stage6a_tb SRC_HALF=%0d DST_HALF=%0d accepted=%0d delivered=%0d dropped=%0d",SRC_HALF,DST_HALF,accepted,delivered,dropped);
        $finish;
    end
    initial begin
        #2000000;
        $fatal(1,"Stage 6A global timeout");
    end
endmodule
`default_nettype wire
