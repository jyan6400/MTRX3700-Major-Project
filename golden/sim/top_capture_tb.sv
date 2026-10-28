`timescale 1ns/1ps
`default_nettype none

// Group 26 - simulated board capture for the golden-model replay (Wilson, M2).
//
// The REAL, unmodified major_project_top sees a lab photo through Ed's OV7670
// model (configured over SCCB by the real ov7670_config). vision_tap watches
// the vision chain through hierarchical references - exactly the nets it is
// wired to on the board (golden/rtl/vision_tap_hookup.svh) - and a SignalTap
// emulator stores tap_word whenever tap_store is high, like SignalTap with
// storage qualifier = tap_store and a finite sample buffer.
//
// It writes the capture in the formats golden/capture.py reads:
//   <prefix>.taplog       one stored sample per line (hex tap_word)
//   <prefix>.vcd          the same samples as a value-change dump, one time
//                         step per sample, tap_word as one 56-bit vector
//   <prefix>_bits.vcd     the same, with tap_word split into 56 1-bit signals
//                         (some SignalTap exports list bus bits separately)
//
// Plusargs: +PREFIX=<path prefix>  +DEPTH=<SignalTap sample depth, 32768>
//           +SCENES=<n>  scene files in +SCENE0= ... (656x488 RGB888 hex,
//           made by golden/capture_sim.py) and +SW1_0= ... per scene
//           +FRAMES_PER_SCENE=<2>

module top_capture_tb;

    logic        CLOCK_50 = 1'b0;
    logic [3:0]  KEY = 4'b1111;
    logic [9:0]  SW  = 10'd0;
    wire  [9:0]  LEDR;
    wire  [35:0] GPIO_1;
    always #10 CLOCK_50 = ~CLOCK_50;

    major_project_top #(.CAMERA_FPS(50), .UART_PERIOD_MS(10)) dut (
        .CLOCK_50(CLOCK_50), .KEY(KEY), .SW(SW), .IRDA_RXD(1'b1),
        .GPIO_1(GPIO_1), .LEDR(LEDR)
    );

    // ---------------- camera ----------------------------------------------
    wire pclk, vsync, href;
    wire [7:0] cam_d;
    assign GPIO_1[21] = pclk;
    assign GPIO_1[22] = href;
    assign GPIO_1[23] = vsync;
    assign GPIO_1[19:12] = cam_d;
    assign GPIO_1[29] = 1'b0;            // sonar echo idle (not under test)
    pullup (GPIO_1[24]);

    int cam_frames, cam_writes, cam_errors;
    ov7670_model #(.VERBOSE(0), .SCENE_FILE("build/golden/capture/scene_init.hex")) cam (
        .sio_c(GPIO_1[25]), .sio_d(GPIO_1[24]), .xclk(GPIO_1[20]),
        .reset_n(GPIO_1[11]), .pwdn(GPIO_1[10]),
        .pclk(pclk), .vsync(vsync), .href(href), .d(cam_d),
        .scene_mode(0), .scene_x0(0), .scene_y0(0), .scene_code(8'h00),
        .frames_done(cam_frames), .write_count(cam_writes), .error_count(cam_errors)
    );

    // ---------------- vision_tap on the board's nets -----------------------
    wire [55:0] tap_word;
    wire        tap_store, tap_trigger;

    vision_tap u_vision_tap (
        .clk(dut.cam_pclk), .rst_n(dut.rst_n_cam),
        .cam_frame_start(dut.cam_frame_start), .cam_valid(dut.cam_pixel_valid),
        .cam_pixel(dut.cam_pixel), .cam_x(dut.cam_x), .cam_y(dut.cam_y),
        .frame_start(dut.frame_start), .frame_end(dut.frame_end), .frame_good(dut.cam_frame_good),
        .pixel_valid(dut.pixel_valid), .is_red(dut.is_red),
        .result_done(dut.cam_result_done), .target_valid(dut.cam_target_valid),
        .target_x(dut.cam_target_x), .target_y(dut.cam_target_y), .red_count(dut.red_count),
        .bypass(dut.target_bypass_sync[1]),
        .tap_word(tap_word), .tap_store(tap_store), .tap_trigger(tap_trigger)
    );

    // ---------------- SignalTap emulator ------------------------------------
    string prefix = "build/golden/capture/sim";
    int    depth  = 32768;
    int    fd_log, fd_vcd, fd_bits;
    int    stored = 0;
    bit    armed = 0, triggered = 0;

    task automatic vcd_header(int fd, bit bits);
        $fwrite(fd, "$date simulated SignalTap capture $end\n$version top_capture_tb $end\n");
        $fwrite(fd, "$timescale 1 ns $end\n$scope module auto_signaltap_0 $end\n");
        $fwrite(fd, "$var wire 1 ! u_vision_tap|tap_store $end\n");
        if (bits)
            for (int i = 0; i < 56; i++)
                $fwrite(fd, "$var wire 1 w%0d u_vision_tap|tap_word[%0d] $end\n", i, i);
        else
            $fwrite(fd, "$var wire 56 # u_vision_tap|tap_word[55:0] $end\n");
        $fwrite(fd, "$upscope $end\n$enddefinitions $end\n");
    endtask

    always @(posedge dut.cam_pclk) begin
        if (armed && !triggered && tap_trigger) triggered = 1;
        if (triggered && tap_store && stored < depth) begin
            $fwrite(fd_log, "%014h\n", tap_word);
            $fwrite(fd_vcd, "#%0d\n1!\nb%b #\n", stored * 25, tap_word);
            $fwrite(fd_bits, "#%0d\n1!\n", stored * 25);
            for (int i = 0; i < 56; i++) $fwrite(fd_bits, "%bw%0d\n", tap_word[i], i);
            stored++;
        end
    end

    // ---------------- scenario -------------------------------------------
    int    n_scenes = 1, fps = 2;
    string scene_file [8];
    int    scene_sw1 [8];

    initial begin
        string key;
        void'($value$plusargs("PREFIX=%s", prefix));
        void'($value$plusargs("DEPTH=%d", depth));
        void'($value$plusargs("SCENES=%d", n_scenes));
        void'($value$plusargs("FRAMES_PER_SCENE=%d", fps));
        for (int i = 0; i < n_scenes; i++) begin
            key = $sformatf("SCENE%0d=%%s", i);
            if (!$value$plusargs(key, scene_file[i])) $fatal(1, "top_capture_tb: +SCENE%0d= missing", i);
            key = $sformatf("SW1_%0d=%%d", i);
            if (!$value$plusargs(key, scene_sw1[i])) scene_sw1[i] = 0;
        end
        fd_log  = $fopen({prefix, ".taplog"}, "w");
        fd_vcd  = $fopen({prefix, ".vcd"}, "w");
        fd_bits = $fopen({prefix, "_bits.vcd"}, "w");
        if (fd_log == 0 || fd_vcd == 0 || fd_bits == 0) $fatal(1, "top_capture_tb: cannot write %s.*", prefix);
        $fwrite(fd_log, "# vision_tap capture, simulated board (top_capture_tb), depth %0d\n", depth);
        vcd_header(fd_vcd, 0);
        vcd_header(fd_bits, 1);

        $readmemh(scene_file[0], cam.video.scene_mem);
        SW[1] = scene_sw1[0][0];

        fork
            wait (dut.cfg_done);
            begin #80ms; $fatal(1, "FAIL: top_capture_tb: camera not configured within 80 ms"); end
        join_any
        disable fork;
        $display("top_capture_tb: camera configured at %0t", $time);

        // let the first configured frame start, then arm the "SignalTap"
        @(posedge vsync);
        armed = 1;
        for (int s = 0; s < n_scenes; s++) begin
            if (s > 0) begin
                @(posedge vsync);
                $readmemh(scene_file[s], cam.video.scene_mem);
                SW[1] = scene_sw1[s][0];
            end
            repeat (fps) @(posedge vsync);
            $display("top_capture_tb: scene %0d (%s, SW1=%0d) done, %0d samples stored", s,
                     scene_file[s], scene_sw1[s], stored);
        end
        // let the last frame's result arrive
        #2ms;
        $fclose(fd_log); $fclose(fd_vcd); $fclose(fd_bits);
        if (cam_errors != 0) $fatal(1, "FAIL: top_capture_tb: %0d SCCB errors", cam_errors);
        $display("PASS: top_capture_tb (%0d samples of %0d, %0d camera frames) -> %s.taplog",
                 stored, depth, cam_frames, prefix);
        $finish;
    end

    initial begin
        #2s;
        $fatal(1, "FAIL: top_capture_tb: timeout");
    end

endmodule

`default_nettype wire
