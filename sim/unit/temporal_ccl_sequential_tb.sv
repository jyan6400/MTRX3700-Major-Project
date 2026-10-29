`timescale 1ns/1ps
module temporal_ccl_sequential_tb;
    localparam int W=7, H=5, N=W*H;
    logic clk=0; always #5 clk=~clk;
    logic rst_n=0, invalidate=0, hit_valid=0, hit=0, frame_start=0, frame_end=0;
    logic [$clog2(W)-1:0] hit_x=0;
    logic [$clog2(H)-1:0] hit_y=0;
    logic release_frame, frame_ready, frame_error, overrun;
    logic read_en, read_valid, read_hit;
    logic [$clog2(N)-1:0] read_addr;
    logic busy, result_valid, target_valid;
    logic [$clog2(W)-1:0] target_x;
    logic [$clog2(H)-1:0] target_y;
    logic [$clog2(N+1)-1:0] blob_pixels;
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
    string rows[H];
    task automatic run_case(input string name, input bit valid_exp,
       input int count_exp, x_exp, y_exp);
        int cycles;
        // Start a complete raster image at x=0,y=0.
        for (int y=0; y<H; y++) for (int x=0; x<W; x++) begin
            @(negedge clk);
            hit_valid=1; hit= (rows[y].getc(x)==8'h23);
            hit_x=$clog2(W)'(x); hit_y=$clog2(H)'(y);
            frame_start=(x==0 && y==0);
            frame_end=(x==W-1 && y==H-1);
        end
        @(negedge clk); hit_valid=0; frame_start=0; frame_end=0; hit=0;
        cycles=0;
        // Sample on falling edges to avoid ordering races at posedge.
        while (!result_valid && cycles<5000) begin @(negedge clk); cycles++; end
        if (!result_valid) $fatal(1,"TIMEOUT %s",name);
        if (target_valid !== valid_exp || blob_pixels !== count_exp ||
            target_x !== x_exp || target_y !== y_exp)
            $fatal(1,"FAIL %s actual=(%0d,%0d,%0d,%0d) expected=(%0d,%0d,%0d,%0d)",
                name,target_valid,blob_pixels,target_x,target_y,
                valid_exp,count_exp,x_exp,y_exp);
        $display("PASS %s (%0d,%0d,%0d,%0d) cycles=%0d",name,
            target_valid,blob_pixels,target_x,target_y,cycles);
        // Wait for DONE to release frame and return to IDLE.
        repeat (4) @(negedge clk);
    endtask
    initial begin
        repeat (3) @(negedge clk); rst_n=1;
        rows[0]="#......"; rows[1]=".#....."; rows[2]="..#....";
        rows[3]="...#..."; rows[4]="....#..";
        run_case("diagonal",1,5,2,2);
        rows[0]="###...."; rows[1]="......."; rows[2]="....###";
        rows[3]="......."; rows[4]=".......";
        run_case("separate_3_3",0,0,0,0);
        rows[0]="......."; rows[1]=".###..."; rows[2]=".###...";
        rows[3]="......."; rows[4]=".......";
        run_case("rectangle",1,6,2,1);
        rows[0]="......."; rows[1]="......."; rows[2]="...#...";
        rows[3]="......."; rows[4]=".......";
        run_case("singleton",0,0,0,0);
        rows[0]="##....."; rows[1]="..#...."; rows[2]="...##..";
        rows[3]="......."; rows[4]=".......";
        run_case("bridge",1,5,2,1);
        rows[0]="......."; rows[1]="......."; rows[2]=".......";
        rows[3]="......."; rows[4]=".......";
        run_case("empty",0,0,0,0);
        $display("PASS: V3-D9C Stage2B six reference scenarios");
        $finish;
    end
endmodule
