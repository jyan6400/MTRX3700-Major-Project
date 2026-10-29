`timescale 1ns/1ps
`default_nettype none
// V3-D10A: production 320x240 standalone CCL latency and ownership regression.
// Source RTL unchanged; measurements are from end-of-capture to result_valid.
module temporal_ccl_fullres_tb;
    localparam int W=320, H=240, N=W*H;
    localparam int XW=$clog2(W), YW=$clog2(H), AW=$clog2(N), CW=$clog2(N+1);
    localparam int MAX_PROCESS_CYCLES=4000000;
    logic clk=0;
    always #10 clk=~clk; // 50 MHz
    logic rst_n=0, invalidate=0, hit_valid=0, hit=0, frame_start=0, frame_end=0;
    logic [XW-1:0] hit_x='0;
    logic [YW-1:0] hit_y='0;
    wire release_frame, frame_ready, frame_error, overrun;
    wire read_en, read_valid, read_hit;
    wire [AW-1:0] read_addr;
    wire busy, result_valid, target_valid;
    wire [XW-1:0] target_x;
    wire [YW-1:0] target_y;
    wire [CW-1:0] blob_pixels;
    bit mask [0:N-1];
    int checks=0;
    temporal_hit_frame_buffer #(.WIDTH(W), .HEIGHT(H)) frame_buffer (
        .clk, .rst_n, .invalidate, .hit_valid, .hit, .frame_start, .frame_end,
        .hit_x, .hit_y, .release_frame, .frame_ready, .frame_error, .overrun,
        .read_en, .read_addr, .read_valid, .read_hit
    );
    temporal_ccl_sequential #(.WIDTH(W), .HEIGHT(H), .MIN_PIXELS(5)) ccl (
        .clk, .rst_n, .invalidate, .frame_ready, .frame_error,
        .read_en, .read_addr, .read_valid, .read_hit, .release_frame,
        .busy, .result_valid, .target_valid, .target_x, .target_y, .blob_pixels
    );
    task automatic clear_mask;
        for (int i=0;i<N;i++) mask[i]=0;
    endtask
    task automatic rectangle(input int x0,y0,x1,y1);
        for (int y=y0;y<=y1;y++)
            for (int x=x0;x<=x1;x++) mask[y*W+x]=1;
    endtask
    task automatic pixel(input int x,y);
        if (x>=0 && x<W && y>=0 && y<H) mask[y*W+x]=1;
    endtask
    task automatic submit_frame;
        for (int y=0;y<H;y++) begin
            for (int x=0;x<W;x++) begin
                @(negedge clk);
                hit_valid=1;
                hit=mask[y*W+x];
                hit_x=XW'(x);
                hit_y=YW'(y);
                frame_start=(x==0 && y==0);
                frame_end=(x==W-1 && y==H-1);
            end
        end
        @(negedge clk);
        hit_valid=0; hit=0; frame_start=0; frame_end=0;
        if (!frame_ready || frame_error)
            $fatal(1,"Frame upload rejected, ready=%b error=%b",frame_ready,frame_error);
    endtask
    task automatic await_result(input string name, input bit want_valid,
        input int want_count,want_x,want_y);
        int cycles;
        cycles=0;
        while (!result_valid && cycles < MAX_PROCESS_CYCLES) begin
            @(negedge clk);
            cycles++;
        end
        if (!result_valid) $fatal(1,"TIMEOUT %s after %0d cycles",name,cycles);
        if (target_valid !== want_valid || blob_pixels !== CW'(want_count) ||
            target_x !== XW'(want_x) || target_y !== YW'(want_y))
            $fatal(1,"FAIL %s valid=%b count=%0d centroid=(%0d,%0d), expected valid=%b count=%0d centroid=(%0d,%0d)",
                name,target_valid,blob_pixels,target_x,target_y,
                want_valid,want_count,want_x,want_y);
        $display("PASS %-25s count=%0d centroid=(%0d,%0d) processing=%0d cycles (%.3f ms at 50 MHz)",
            name,blob_pixels,target_x,target_y,cycles,real'(cycles)/50000.0);
        checks++;
        repeat (4) @(negedge clk); // allow DONE/release/IDLE
    endtask
    task automatic test_case(input string name, input bit want_valid,
        input int want_count,want_x,want_y);
        submit_frame();
        await_result(name,want_valid,want_count,want_x,want_y);
    endtask
    initial begin
        repeat (4) @(negedge clk);
        rst_n=1;
        clear_mask();
        test_case("empty",0,0,0,0);
        clear_mask(); rectangle(40,30,44,34);
        test_case("single_5x5",1,25,42,32);
        clear_mask(); rectangle(12,12,17,17); rectangle(180,100,199,109);
        test_case("largest_of_two",1,200,189,104);
        clear_mask(); rectangle(10,20,109,119);
        test_case("dense_100x100",1,10000,59,69);
        clear_mask(); rectangle(0,0,W-1,H-1);
        test_case("full_frame_76800",1,N,159,119);
        // Overflow/ownership: an attempted new frame while the CCL owns FULL
        // should set overrun, but cannot overwrite the existing frame.
        clear_mask(); rectangle(40,30,44,34);
        submit_frame();
        @(negedge clk);
        if (!busy || !frame_ready) $fatal(1,"Expected active CCL on owned frame");
        hit_valid=1; hit=1; hit_x='0; hit_y='0;
        frame_start=1; frame_end=0;
        @(negedge clk);
        hit_valid=0; frame_start=0;
        if (!overrun) $fatal(1,"Expected overrun flag after competing frame_start");
        await_result("overrun_preserves_frame",1,25,42,32);
        // Invalidate during a full-resolution processing operation, then retry.
        clear_mask(); rectangle(70,60,75,65);
        submit_frame();
        repeat (20) @(negedge clk);
        invalidate=1;
        @(negedge clk);
        invalidate=0;
        repeat (4) @(negedge clk);
        if (busy || frame_ready || result_valid)
            $fatal(1,"Invalidation failed to clear active frame ownership");
        $display("PASS invalidate_during_processing");
        checks++;
        clear_mask(); rectangle(90,80,95,85);
        test_case("after_invalidate",1,36,92,82);
        $display("PASS V3-D10A FULLRES: %0d checks, %0dx%0d at 50MHz",checks,W,H);
        $finish;
    end
endmodule
`default_nettype wire
