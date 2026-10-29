`timescale 1ns/1ps
`default_nettype none
module temporal_camera_to_50_pipeline_tb;
    localparam int RW=24, RH=16, W=RW/2, H=RH/2;
    logic pclk=0, clk_50=0;
    always #12.5 pclk=~pclk; // 40 MHz
    always #10 clk_50=~clk_50; // 50 MHz, unrelated clock phases
    logic rst_n_pclk=0, rst_n_50=0, invalidate=0;
    logic capture_off=0,capture_on=0,cam_frame_start=0,cam_valid=0;
    logic [15:0] cam_pixel=0;
    logic [9:0] cam_x=0;
    logic [8:0] cam_y=0;
    wire frame_start,frame_end,pixel_valid,frame_good;
    wire [15:0] pixel_rgb565;
    wire [9:0] pixel_x,pixel_y;
    wire stream_ok;
    wire [1:0] last_error;
    wire [15:0] good_frames,bad_frames,dropped_pixels;
    wire [23:0] frame_period;
    camera_stream_adapter #(.W(RW),.H(RH)) adapter (
        .clk(pclk), .rst_n(rst_n_pclk), .cam_frame_start, .cam_valid, .cam_pixel,
        .cam_x, .cam_y, .frame_start, .frame_end, .pixel_valid,
        .pixel_rgb565, .pixel_x, .pixel_y, .frame_good,
        .stream_ok,.last_error,.good_frames,.bad_frames,.dropped_pixels,.frame_period
    );
    wire off_ready,pair_complete,pair_error,frame_ready,frame_error,frame_overrun;
    wire ccl_busy,ccl_result_valid,cdc_src_ready,cdc_src_overrun;
    wire result_50_valid,target_50_valid;
    wire [$clog2(W)-1:0] target_50_x;
    wire [$clog2(H)-1:0] target_50_y;
    wire [$clog2(W*H+1)-1:0] blob_50_pixels;
    temporal_camera_to_50_pipeline #(.RAW_W(RW),.RAW_H(RH)) dut (
        .pclk,.rst_n_pclk,.clk_50,.rst_n_50,.invalidate,.capture_off,.capture_on,
        .frame_start,.frame_end,.frame_good,.pixel_valid,.pixel_rgb565,.pixel_x,.pixel_y,
        .off_ready,.pair_complete,.pair_error,.frame_ready,.frame_error,.frame_overrun,
        .ccl_busy,.ccl_result_valid,.cdc_src_ready,.cdc_src_overrun,
        .result_50_valid,.target_50_valid,.target_50_x,.target_50_y,.blob_50_pixels
    );
    int results_seen=0;
    logic expected_valid=0;
    int expected_count=0;
    always @(posedge clk_50) begin
        if (rst_n_50 && result_50_valid) begin
            if (target_50_valid!==expected_valid || blob_50_pixels!==expected_count ||
                 (expected_valid && (target_50_x!==5 || target_50_y!==2)))
                $fatal(1,"CDC mismatch valid=%b count=%0d xy=%0d,%0d expected valid=%b count=%0d",
                    target_50_valid,blob_50_pixels,target_50_x,target_50_y,expected_valid,expected_count);
            results_seen++;
            $display("PASS camera->CCL->CDC result=%0d valid=%b count=%0d xy=(%0d,%0d)",
                    results_seen,target_50_valid,blob_50_pixels,target_50_x,target_50_y);
        end
    end
    task automatic send_frame(input bit on,input bit red);
        @(negedge pclk); capture_off=!on;capture_on=on;cam_frame_start=1;
        @(negedge pclk);cam_frame_start=0;
        for(int y=0;y<RH;y++)begin
            for(int x=0;x<RW;x++)begin
                @(negedge pclk);
                cam_valid=1;cam_x=10'(x);cam_y=9'(y);cam_pixel=16'h0000;
                if(on && (x>=8 && x<=12) && (y>=4 && y<=6))
                    cam_pixel=red ? 16'hF800 : 16'h07E0;
            end
        end
        @(negedge pclk);cam_valid=0;
        repeat(5)@(negedge pclk);
        capture_off=0;capture_on=0;
    endtask
    task automatic run_pair(input bit red,input int expected_result_count);
        int watchdog;
        expected_valid=red;
        expected_count=red ? 6 : 0;
        send_frame(0,0);
        if(!off_ready)$fatal(1,"OFF reference not captured");
        send_frame(1,red);
        watchdog=0;
        while(results_seen<expected_result_count && watchdog<20000)begin
            @(negedge clk_50);watchdog++;
        end
        if(results_seen!=expected_result_count)$fatal(1,"Missing destination result");
        if(pair_error || frame_error || frame_overrun || cdc_src_overrun)
            $fatal(1,"Unexpected integrity/CDC overrun");
        repeat(8)@(negedge clk_50);
    endtask
    initial begin
        repeat(5)@(negedge clk_50);
        rst_n_pclk=1;rst_n_50=1;
        run_pair(1,1);
        run_pair(0,2);
        $display("PASS V3-D10C STAGE1: 2 coherent camera-to-50MHz results");
        $finish;
    end
endmodule
`default_nettype wire
