`timescale 1ns/1ps
`default_nettype none
module temporal_sequential_pipeline_tb;
    localparam int W=12, H=8;
    logic clk=0; always #5 clk=~clk;
    logic rst_n=0, invalidate=0, capture_off=0, capture_on=0;
    logic pixel_valid=0, frame_start=0, frame_end=0;
    logic [15:0] pixel_rgb565=0;
    wire off_ready, pair_complete, pair_error, frame_ready, frame_error, overrun;
    wire busy, result_valid, target_valid;
    wire [$clog2(W)-1:0] target_x;
    wire [$clog2(H)-1:0] target_y;
    wire [$clog2(W*H+1)-1:0] blob_pixels;
    int checks=0;
    temporal_sequential_pipeline #(.WIDTH(W),.HEIGHT(H),.MIN_PIXELS(5)) dut (.*);

    task automatic assert_ok(input bit condition, input string message);
        if(!condition) $fatal(1,"FAIL %s: valid=%b count=%0d xy=(%0d,%0d) err=%b pair_err=%b overrun=%b",
             message,target_valid,blob_pixels,target_x,target_y,frame_error,pair_error,overrun);
        checks++;
        $display("PASS [%0d] %s",checks,message);
    endtask
    // RGB565 OFF reference is black; ON frame changes selected pixels.
    // scenario 1: six-pixel red blob; 2: green-only; 3: two isolated reds.
    task automatic send_frame(input bit is_on, input int scenario);
        @(negedge clk); capture_off=!is_on; capture_on=is_on;
        for(int y=0;y<H;y++) for(int x=0;x<W;x++) begin
            @(negedge clk);
            pixel_valid=1;
            frame_start=(x==0 && y==0);
            frame_end=(x==W-1 && y==H-1);
            pixel_rgb565=16'h0000;
            if(is_on) begin
                if(scenario==1 && x>=4 && x<=6 && y>=2 && y<=3) pixel_rgb565=16'hF800;
                if(scenario==2 && x>=4 && x<=6 && y>=2 && y<=3) pixel_rgb565=16'h07E0;
                if(scenario==3 && ((x==2 && y==1)||(x==9 && y==6))) pixel_rgb565=16'hF800;
            end
        end
        @(negedge clk);
        pixel_valid=0; frame_start=0; frame_end=0;
        capture_off=0; capture_on=0;
        repeat(4) @(negedge clk);
    endtask
    task automatic check_pair(input int scenario, input bit expect_target,
                               input int expect_count, expect_x, expect_y);
        int waited;
        send_frame(0,0);
        assert_ok(off_ready,"OFF frame stored");
        send_frame(1,scenario);
        waited=0;
        while(!result_valid && waited<2000) begin @(negedge clk); waited++; end
        assert_ok(result_valid,"CCL result produced");
        assert_ok(!frame_error && !pair_error && !overrun,"frame and pair error-free");
        assert_ok(target_valid==expect_target &&
                  blob_pixels==expect_count &&
                  target_x==expect_x && target_y==expect_y,"target matches expected");
        repeat(3) @(negedge clk);
    endtask
    initial begin
        repeat(5) @(negedge clk); rst_n=1;
        check_pair(1,1,6,5,2);
        check_pair(2,0,0,0,0);
        check_pair(3,0,0,0,0);
        @(negedge clk); invalidate=1;
        @(negedge clk); invalidate=0;
        assert_ok(!off_ready && !target_valid && !frame_ready,"invalidate clears ownership and target");
        check_pair(1,1,6,5,2);
        $display("PASS V3-D10B temporal sequential pipeline (%0d checks)",checks);
        $finish;
    end
endmodule
`default_nettype wire
