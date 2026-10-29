`timescale 1ns/1ps
`default_nettype none
module temporal_camera_pclk_pipeline_tb;
    localparam int RW=24, RH=16, W=RW/2, H=RH/2;
    logic pclk=0;
    always #12.5 pclk=~pclk; // representative 40MHz camera clock
    logic rst_n=0, invalidate=0, capture_off=0, capture_on=0;
    logic cam_frame_start=0, cam_valid=0;
    logic [15:0] cam_pixel=0;
    logic [9:0] cam_x=0;
    logic [8:0] cam_y=0;
    wire frame_start, frame_end, pixel_valid, frame_good;
    wire [15:0] pixel_rgb565;
    wire [9:0] pixel_x, pixel_y;
    wire stream_ok;
    wire [1:0] last_error;
    wire [15:0] good_frames,bad_frames,dropped_pixels;
    wire [23:0] frame_period;
    camera_stream_adapter #(.W(RW),.H(RH)) adapter (
        .clk(pclk), .rst_n, .cam_frame_start, .cam_valid, .cam_pixel,
        .cam_x, .cam_y, .frame_start, .frame_end, .pixel_valid,
        .pixel_rgb565, .pixel_x, .pixel_y, .frame_good,
        .stream_ok, .last_error, .good_frames, .bad_frames,
        .dropped_pixels, .frame_period
    );
    wire off_ready,pair_complete,pair_error,frame_ready,frame_error,overrun;
    wire busy,result_valid,target_valid;
    wire [$clog2(W)-1:0] target_x;
    wire [$clog2(H)-1:0] target_y;
    wire [$clog2(W*H+1)-1:0] blob_pixels;
    temporal_camera_pclk_pipeline #(.RAW_W(RW),.RAW_H(RH)) dut (
        .pclk,.rst_n,.invalidate,.capture_off,.capture_on,
        .frame_start,.frame_end,.frame_good,.pixel_valid,.pixel_rgb565,
        .pixel_x,.pixel_y,.off_ready,.pair_complete,.pair_error,
        .frame_ready,.frame_error,.overrun,.busy,.result_valid,
        .target_valid,.target_x,.target_y,.blob_pixels
    );
    int checks=0;
    task automatic ck(input bit ok,input string message);
        if(!ok) $fatal(1,"FAIL %s valid=%b count=%0d xy=%0d,%0d err=%b",
                        message,target_valid,blob_pixels,target_x,target_y,frame_error);
        checks++;
        $display("PASS [%0d] %s",checks,message);
    endtask
    task automatic send_frame(input bit on,input int scenario);
        @(negedge pclk);
        capture_off=!on; capture_on=on;
        cam_frame_start=1;
        @(negedge pclk);
        cam_frame_start=0;
        for(int y=0;y<RH;y++) begin
            for(int x=0;x<RW;x++) begin
                @(negedge pclk);
                cam_valid=1;
                cam_x=10'(x); cam_y=9'(y);
                cam_pixel=0;
                // 2x2 decimation maps (8..12,4..6) raw to 3x2 sampled
                if(on && scenario==1 && (x>=8 && x<=12) && (y>=4 && y<=6))
                    cam_pixel=16'hF800;
                if(on && scenario==2 && (x>=8 && x<=12) && (y>=4 && y<=6))
                    cam_pixel=16'h07E0;
            end
        end
        @(negedge pclk); cam_valid=0;
        repeat(5) @(negedge pclk);
        capture_off=0; capture_on=0;
    endtask
    task automatic check_pair(input int scenario,input bit valid_exp,input int count_exp);
        int n;
        send_frame(0,0);
        ck(off_ready,"OFF reference captured through checked 2x-decimated camera stream");
        send_frame(1,scenario);
        n=0;
        while(!result_valid && n<5000) begin @(negedge pclk);n++;end
        ck(result_valid,"CCL emitted a result");
        ck(!pair_error && !frame_error && !overrun,"integrity signals clean");
        ck(target_valid==valid_exp && blob_pixels==count_exp &&
           ((!valid_exp)||(target_x==5 && target_y==2)), "centroid and count correct");
        repeat(3) @(negedge pclk);
    endtask
    initial begin
        repeat(5) @(negedge pclk);rst_n=1;
        check_pair(1,1,6);
        check_pair(2,0,0);
        @(negedge pclk);invalidate=1;
        @(negedge pclk);invalidate=0;
        ck(!off_ready && !target_valid,"invalidation clears stale data");
        check_pair(1,1,6);
        $display("PASS V3-D10B STAGE2 CAMERA PCLK (%0d checks)",checks);
        $finish;
    end
endmodule
`default_nettype wire
