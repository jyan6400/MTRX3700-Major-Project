`timescale 1ns/1ps
/*
 *  vo_grab_top_tb -- Group 26 (Wilson, M2): the VO bench board, simulated. vo_grab_top pins to pins with the repo's
 *  camera model showing scene_in.hex (golden/vo/vo_grab_sim.py draws the three LEDs from a known pose). Two
 *  snapshots: KEY1 tracking green (SW[5:4] = 1), then the PC's request tracking red (SW[5:4] = 0). Each snapshot's
 *  memory goes to snap_<k>.words, and the HEX digits shown at the time to hex_<k>.txt. The bench checks the
 *  plumbing (stored pixels as sent, the 21 words as held at frame_snap's moment); vo_validate_grab.py checks the
 *  numbers against the golden model and the pose.
 */
/* verilator lint_off BLKSEQ */
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off UNUSEDSIGNAL */
// (A testbench: never synthesised.)
/*
 *  vo_grab_top_tb -- Group 26 (Wilson, M2): the board, simulated. vision_grab_top pins to pins with the repo's
 *  camera model showing scene_in.hex (a 656 x 488 RGB888 scene, in the folder it runs in). ov7670_config sets the
 *  camera up over the SCCB at the robot's 50 fps; then two snapshots are taken, as on the bench:
 *      1. KEY1, with SW1 down (V2 gates)
 *      2. the PC's request through the GCTL probes (what tcl/grab_jtag.tcl does), with SW1 up (V1 bypass)
 *  Each snapshot's memory GRAB is written to snap_<k>.words (one 4-digit hex word a line, address 0 first): the
 *  bytes grab_jtag.tcl would put in dump.bin. framegrab_sim.py turns them into dump.bin and runs grab.py and
 *  validate_grab.py on them, exactly as on a real grab.
 *
 *  The bench checks the plumbing only: every stored pixel is what the camera sent for the kept frame, and the 18
 *  result words are what vision_grab_results held at frame_snap's moment. Whether those words are RIGHT is the
 *  golden model's job (validate_grab.py).
 *  Plusargs: +out=<dir> (default .)
 */
module vo_grab_top_tb;
    localparam int W = 320, H = 240, N = 21, PIX0 = 7 + 2 * N, DEPTH = PIX0 + W * H + 1;

    logic CLOCK_50 = 1'b0; always #10 CLOCK_50 = ~CLOCK_50;
    logic [3:0] KEY = 4'b1111;
    logic [9:0] SW  = '0;
    wire  [9:0] LEDR;
    wire  [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
    wire  [35:0] GPIO_1;
    wire  VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N; wire [7:0] VGA_R, VGA_G, VGA_B;
    vo_grab_top dut (.CLOCK_50, .KEY, .SW, .LEDR, .HEX0, .HEX1, .HEX2, .HEX3, .HEX4, .HEX5, .GPIO_1, .VGA_CLK, .VGA_HS, .VGA_VS, .VGA_BLANK_N, .VGA_SYNC_N,
                         .VGA_R, .VGA_G, .VGA_B);

    wire pclk, vsync, href; wire [7:0] d;
    assign GPIO_1[21] = pclk; assign GPIO_1[22] = href; assign GPIO_1[23] = vsync; assign GPIO_1[19:12] = d;
    pullup(GPIO_1[24]);
    int frames_done, write_count, error_count;
    ov7670_model #(.VERBOSE(0), .SCENE_FILE("scene_in.hex")) cam (
        .sio_c(GPIO_1[25]), .sio_d(GPIO_1[24]), .xclk(GPIO_1[20]), .reset_n(GPIO_1[11]), .pwdn(GPIO_1[10]),
        .pclk(pclk), .vsync(vsync), .href(href), .d(d),
        .scene_mode(1), .scene_x0(0), .scene_y0(0), .scene_code(8'hA5),
        .frames_done(frames_done), .write_count(write_count), .error_count(error_count));

    // the result words at frame_snap's moment (S_RESULTS, the next frame's first pixel)
    logic [32*N-1:0] taken;
    int              n_taken = 0;
    always @(posedge pclk) if (dut.snap_u.state == 3'd3 && dut.cam_pixel_valid) begin taken = dut.results; n_taken++; end

    function automatic string hexchar(input logic [6:0] h);
        case (h)
            7'b1000000: return "0"; 7'b1111001: return "1"; 7'b0100100: return "2"; 7'b0110000: return "3";
            7'b0011001: return "4"; 7'b0010010: return "5"; 7'b0000010: return "6"; 7'b1111000: return "7";
            7'b0000000: return "8"; 7'b0010000: return "9"; 7'b0111111: return "-"; 7'b1111111: return " ";
            7'b0101111: return "r"; 7'b1000010: return "G"; 7'b0000011: return "b"; default: return "?";
        endcase
    endfunction

    int fails = 0;
    string out = ".";
    task automatic fail(input string s); fails++; $display("FAIL: %s", s); endtask

    task automatic check_snapshot(input int k, input string how);
        int fd, wrong = 0;
        logic [15:0] want;
        for (int v = 0; v < H; v++)
            for (int u = 0; u < W; u++) begin
                want = cam.expected_word(2 * u, 2 * v);
                if (dut.snap_u.ram_u.mem[PIX0 + v * W + u] !== want) begin
                    wrong++;
                    if (wrong <= 3) $display("  pixel (%0d,%0d): %04h, the camera sent %04h", u, v, dut.snap_u.ram_u.mem[PIX0 + v * W + u], want);
                end
            end
        if (wrong != 0) fail($sformatf("snapshot %0d (%s): %0d of %0d stored pixels are not what the camera sent", k, how, wrong, W * H));
        for (int i = 0; i < N; i++)
            if ({dut.snap_u.ram_u.mem[7 + 2 * i], dut.snap_u.ram_u.mem[8 + 2 * i]} !== taken[32 * (N - 1 - i) +: 32])
                fail($sformatf("snapshot %0d (%s): result word %0d is not vision_grab_results' at frame_snap's moment", k, how, i));
        fd = $fopen($sformatf("%s/hex_%0d.txt", out, k), "w");
        $fdisplay(fd, "%s%s%s%s%s%s", hexchar(HEX5), hexchar(HEX4), hexchar(HEX3), hexchar(HEX2), hexchar(HEX1), hexchar(HEX0));
        $fclose(fd);
        fd = $fopen($sformatf("%s/snap_%0d.words", out, k), "w");
        for (int a = 0; a < DEPTH; a++) $fdisplay(fd, "%04x", dut.snap_u.ram_u.mem[a]);
        $fclose(fd);
        $display("vo_grab_top: snapshot %0d (%s, SW[5:4]=%0d, HEX %s%s%s%s%s%s): %0d of %0d pixels as sent, words %08x %08x ... -> %s/snap_%0d.words",
                 k, how, SW[5:4], hexchar(HEX5), hexchar(HEX4), hexchar(HEX3), hexchar(HEX2), hexchar(HEX1), hexchar(HEX0), W * H - wrong, W * H, taken[32*N-1 -: 32], taken[32*N-33 -: 32], out, k);
    endtask

    initial begin
        int c0;
        if ($value$plusargs("out=%s", out)) ;
        fork begin #3s; $display("FAILED: vo_grab_top_tb: timed out"); $finish; end join_none
        KEY[0] = 1'b0; #500ns KEY[0] = 1'b1;
        wait (dut.cfg_done);
        $display("[tb @%0t] camera configured over the SCCB (%0d writes, %0d SCCB errors)", $time, write_count, error_count);
        if (error_count != 0) fail("SCCB errors during configuration");
        repeat (2) @(posedge dut.cam_frame_start);

        // 1. KEY1, tracking green
        SW[5:4] = 2'd1;
        repeat (2) @(posedge dut.cam_frame_start);
        wait (dut.cam_pixel_valid && dut.cam_y == 200);
        KEY[1] = 1'b0; #1ms KEY[1] = 1'b1;
        wait (LEDR[3]);
        #1us;
        check_snapshot(1, "KEY1");
        if (!LEDR[6]) fail("LEDR6: the latest frame did not pass the adapter's check");
        if (!LEDR[5]) fail("LEDR5: no track of the green LED");

        // 2. the PC's request, tracking red (two frames: the synchroniser, then a whole frame)
        SW[5:4] = 2'd0;
        repeat (2) @(posedge dut.cam_frame_start);
        wait (dut.cam_pixel_valid && dut.cam_y == 100);
        c0 = int'(dut.snap_u.probes_u.probe[20:5]);
        dut.snap_u.probes_u.src_jtag = ~dut.snap_u.probes_u.src_jtag;
        wait (int'(dut.snap_u.probes_u.probe[20:5]) == c0 + 1 && LEDR[3]);
        #1us;
        check_snapshot(2, "the PC");
        if (dut.vga_u.errors != 0) fail($sformatf("%0d Avalon-ST protocol errors at the VGA sink", dut.vga_u.errors));

        if (fails == 0) $display("PASS: vo_grab_top_tb: 2 snapshots (KEY1 tracking green, the PC's request tracking red): every stored pixel as the camera sent it, the 21 words as the VO held them");
        else            $display("FAILED: vo_grab_top_tb: %0d problems", fails);
        $finish;
    end
endmodule
