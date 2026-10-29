`timescale 1ns/1ps
// V3-D9C Stage 2C stress regression. Deliberately standalone.
module temporal_ccl_stage2c_tb;
    localparam int W=32, H=24, N=W*H;
    localparam int XW=$clog2(W), YW=$clog2(H), AW=$clog2(N), CW=$clog2(N+1);
    logic clk=0;
    always #5 clk=~clk;
    logic rst_n=0, invalidate=0, hit_valid=0, hit=0, frame_start=0, frame_end=0;
    logic [XW-1:0] hit_x=0;
    logic [YW-1:0] hit_y=0;
    logic release_frame, frame_ready, frame_error, overrun;
    logic read_en, read_valid, read_hit;
    logic [AW-1:0] read_addr;
    logic busy, result_valid, target_valid;
    logic [XW-1:0] target_x;
    logic [YW-1:0] target_y;
    logic [CW-1:0] blob_pixels;
    bit mask [0:N-1];
    int checks=0;
    temporal_hit_frame_buffer #(.WIDTH(W),.HEIGHT(H)) buffer (
        .clk,.rst_n,.invalidate,.hit_valid,.hit,.frame_start,.frame_end,
        .hit_x,.hit_y,.release_frame,.frame_ready,.frame_error,.overrun,
        .read_en,.read_addr,.read_valid,.read_hit
    );
    temporal_ccl_sequential #(.WIDTH(W),.HEIGHT(H),.MIN_PIXELS(5)) dut (
        .clk,.rst_n,.invalidate,.frame_ready,.frame_error,.read_en,.read_addr,
        .read_valid,.read_hit,.release_frame,.busy,.result_valid,
        .target_valid,.target_x,.target_y,.blob_pixels
    );
    task automatic clear_mask;
        for (int i=0; i<N; i++) mask[i]=0;
    endtask
    task automatic pixel(input int x,y);
        if (x>=0 && x<W && y>=0 && y<H) mask[y*W+x]=1;
    endtask
    task automatic fill_rectangle(input int x0,y0,x1,y1);
        for(int y=y0;y<=y1;y++) for(int x=x0;x<=x1;x++) pixel(x,y);
    endtask
    task automatic submit_frame;
        for(int y=0;y<H;y++) for(int x=0;x<W;x++) begin
            @(negedge clk);
            hit_valid=1; hit=mask[y*W+x]; hit_x=XW'(x); hit_y=YW'(y);
            frame_start=(x==0 && y==0); frame_end=(x==W-1 && y==H-1);
        end
        @(negedge clk); hit_valid=0; hit=0; frame_start=0; frame_end=0;
    endtask
    task automatic check_case(input string name, input bit want_valid,
        input int want_count, want_x, want_y);
        int cycles;
        submit_frame();
        cycles=0;
        while (!result_valid && cycles < 200000) begin
            @(negedge clk);
            cycles++;
        end
        if (!result_valid) $fatal(1,"TIMEOUT %s at %0d cycles",name,cycles);
        if (target_valid !== want_valid || blob_pixels !== CW'(want_count) ||
            target_x !== XW'(want_x) || target_y !== YW'(want_y))
            $fatal(1,"FAIL %s got valid=%b count=%0d x=%0d y=%0d expected %b,%0d,%0d,%0d",
                name,target_valid,blob_pixels,target_x,target_y,
                want_valid,want_count,want_x,want_y);
        $display("PASS %s count=%0d centroid=(%0d,%0d) cycles=%0d",
            name,blob_pixels,target_x,target_y,cycles);
        checks++;
        repeat (4) @(negedge clk);
    endtask
    initial begin
        repeat (4) @(negedge clk); rst_n=1;
        // Non-overlapping frames, no reset between cases.
        clear_mask();
        for(int i=0;i<5;i++) pixel(i,i);
        check_case("diagonal",1,5,2,2);
        clear_mask(); fill_rectangle(4,3,8,5);
        check_case("rectangle_15",1,15,6,4);
        clear_mask(); fill_rectangle(0,0,1,1); fill_rectangle(20,10,22,11);
        check_case("largest_blob",1,6,21,10);
        clear_mask(); pixel(1,1); pixel(10,3); pixel(21,14); pixel(31,23);
        check_case("isolated",0,0,0,0);
        clear_mask(); fill_rectangle(0,0,W-1,H-1);
        check_case("full_frame_queue_capacity",1,N,(W-1)/2,(H-1)/2);
        clear_mask();
        check_case("empty_after_full",0,0,0,0);
        clear_mask(); fill_rectangle(7,7,9,9);
        check_case("rectangle_after_empty",1,9,8,8);
        // Invalidate during an active scan. Must abort and permit a clean new frame.
        clear_mask(); fill_rectangle(2,2,5,5);
        submit_frame();
        repeat(12) @(negedge clk);
        invalidate=1;
        @(negedge clk); invalidate=0;
        repeat(4) @(negedge clk);
        if (busy || result_valid || frame_ready)
            $fatal(1,"Invalidation did not abort processing and clear ownership");
        checks++;
        $display("PASS invalidate_during_processing");
        clear_mask(); fill_rectangle(12,10,15,12);
        check_case("post_invalidate",1,12,13,11);
        $display("PASS Stage2C stress suite: %0d checks (W=%0d H=%0d)",checks,W,H);
        $finish;
    end
endmodule
