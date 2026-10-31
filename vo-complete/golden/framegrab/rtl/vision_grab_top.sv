`timescale 1ns/1ps
`default_nettype none

/*
 *  vision_grab_top.sv -- Group 26 (Wilson, M2): take a photo with the robot's camera over JTAG, together with
 *  what the robot's vision chain computed from that same frame, for the golden-model validation.
 *
 *  The camera code is the team repo's (vendor/camera/rtl: video_pll, clock_divider, ov7670_config,
 *  ov7670_capture, frame_buffer, camera_display, vga_sink.qsys). The frame grab is FrameGrabJtag V2026.3
 *  (golden/framegrab/vendor: frame_snap, snapshot_ram, snapshot_probes, ov7670_hold, tcl/grab_jtag.tcl,
 *  tools/grab.py). The vision chain is the robot's, unchanged, wired as in rtl/integration/major_project_top.sv:
 *
 *      camera --> ov7670_capture --+--> camera_stream_adapter --> red_mask --> led_centroid (SW1 = gate_bypass)
 *                                  |                                     \--> vision_grab_results (FULL + KEPT) --+
 *                                  +--> frame_snap (320 x 240 RGB565 + the 18 result words) <-------------------+
 *                                  +--> frame_buffer --> camera_display --> VGA (with the robot's target marked)
 *
 *  Controls (as close to the robot as the board allows):
 *      KEY0  reset (re-runs the camera configuration)
 *      KEY1  take a snapshot (or run tools/take_photo.py / tcl/grab_jtag.tcl on the PC)
 *      SW1   led_centroid gate_bypass, through the robot's two-flop synchroniser: 0 = V2 gates, 1 = V1 bypass
 *      SW9   hold the camera's automatic exposure, gain and white balance (ov7670_hold), so colours stay put
 *            between grabs; down lets them run again
 *  LEDs:
 *      LEDR0 camera configured     LEDR1 SCCB error (retrying)     LEDR2 toggles every camera frame
 *      LEDR3 a snapshot is held    LEDR4 a snapshot is being taken
 *      LEDR5 robot target valid (latest frame)    LEDR6 latest frame passed the adapter's integrity check
 *      LEDR7 SW1 bypass as led_centroid sees it   LEDR8 hold's write not acknowledged    LEDR9 camera held
 *  VGA: the camera picture (frame_buffer, 320 x 240 doubled) with a green cross on the robot's latest target
 *  (SHOW_TARGET), so you can aim the LED before grabbing.
 *
 *  CAMERA_FPS is 50, as major_project_top. SNAP_MODE is 0 (320 x 240 RGB565): the only picture the replay and
 *  the KEPT check can use. Memory: frame_buffer 150 M10K + frame_snap about 150, of 397.
 */
module vision_grab_top #(
    parameter int CAMERA_FPS  = 50,
    parameter bit SHOW_TARGET = 1'b1
) (
    input  wire        CLOCK_50,
    input  wire [3:0]  KEY,
    input  wire [9:0]  SW,
    output wire [9:0]  LEDR,
    inout  wire [35:0] GPIO_1,
    output wire        VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N,
    output wire [7:0]  VGA_R, VGA_G, VGA_B
);
    localparam int N_RESULTS = 18;

    // ---- clocks ------------------------------------------------------------------------------
    wire clk_25, clk_10, pll_locked;
    video_pll video_pll_u (.refclk(CLOCK_50), .rst(1'b0), .outclk_0(clk_25), .outclk_1(clk_10), .locked(pll_locked));
    wire sccb_clk;
    clock_divider #(.DIVISOR(250)) sccb_clk_u (.clk_in(clk_25), .clk_out(sccb_clk));       // 100 kHz

    // ---- the camera's pins (Robot Adaptor Board, as major_project_top) -----------------------
    wire       cam_pclk  = GPIO_1[21];
    wire       cam_href  = GPIO_1[22];
    wire       cam_vsync = GPIO_1[23];
    wire [7:0] cam_d     = GPIO_1[19:12];
    wire       cam_reset_n, cam_pwdn, cfg_sioc, hold_sioc;
    assign GPIO_1[20] = clk_10;          // XCLK 10 MHz
    assign GPIO_1[11] = cam_reset_n;
    assign GPIO_1[10] = cam_pwdn;
    // GPIO_1[24] is SIO_D (open drain, inside ov7670_config and ov7670_hold); SIO_C is muxed below

    // ---- resets: KEY0 or the PLL unlocked; asserted at once, released in each domain ---------------
    wire reset_request_n = KEY[0] & pll_locked;
    (* async_reg = "true" *) logic [1:0] rs25 = 2'b00, rs_sccb = 2'b00, rs_cam = 2'b00;
    always_ff @(posedge clk_25 or negedge reset_request_n)
        if (!reset_request_n) rs25 <= 2'b00; else rs25 <= {rs25[0], 1'b1};
    always_ff @(posedge sccb_clk or negedge reset_request_n)
        if (!reset_request_n) rs_sccb <= 2'b00; else rs_sccb <= {rs_sccb[0], 1'b1};
    always_ff @(posedge cam_pclk or negedge reset_request_n)
        if (!reset_request_n) rs_cam <= 2'b00; else rs_cam <= {rs_cam[0], 1'b1};
    wire reset25 = ~rs25[1], reset_sccb = ~rs_sccb[1];
    wire rst_n_cam = rs_cam[1];

    // ---- configure the camera, then the hold (SW9) on the same bus -------------------------------
    wire cfg_done, cfg_error;
    ov7670_config #(.FPS(CAMERA_FPS)) cfg_u (
        .clk(sccb_clk), .reset(reset_sccb), .sioc(cfg_sioc), .siod(GPIO_1[24]),
        .cam_reset_n(cam_reset_n), .cam_pwdn(cam_pwdn), .done(cfg_done), .error(cfg_error));

    wire cam_held, hold_error;
    ov7670_hold hold_u (
        .clk(sccb_clk), .reset(reset_sccb), .cfg_done(cfg_done), .hold(SW[9]),
        .sioc(hold_sioc), .siod(GPIO_1[24]), .held(cam_held), .busy(), .error(hold_error));
    assign GPIO_1[25] = cfg_done ? hold_sioc : cfg_sioc;

    // ---- capture (PCLK) ----------------------------------------------------------------------------
    wire        cam_frame_start, cam_pixel_valid;
    wire [15:0] cam_pixel;
    wire [9:0]  cam_x;
    wire [8:0]  cam_y;
    ov7670_capture cap_u (
        .pclk(cam_pclk), .vsync(cam_vsync), .href(cam_href), .d(cam_d),
        .frame_start(cam_frame_start), .valid(cam_pixel_valid), .pixel(cam_pixel), .x(cam_x), .y(cam_y));

    // ---- the robot's vision chain (major_project_top section 4, unchanged) ----------------------
    wire        frame_start, frame_end, pixel_valid;
    wire [15:0] pixel_rgb565;
    wire [9:0]  pixel_x, pixel_y;
    wire        cam_frame_good, cam_stream_ok;
    wire [1:0]  cam_last_error;
    wire [15:0] cam_good_frames, cam_bad_frames, cam_dropped_pixels;
    wire [23:0] cam_frame_period;

    camera_stream_adapter #(.W(640), .H(480)) u_camera_adapter (
        .clk(cam_pclk), .rst_n(rst_n_cam),
        .cam_frame_start(cam_frame_start), .cam_valid(cam_pixel_valid), .cam_pixel(cam_pixel),
        .cam_x(cam_x), .cam_y(cam_y),
        .frame_start(frame_start), .frame_end(frame_end), .pixel_valid(pixel_valid),
        .pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y),
        .frame_good(cam_frame_good), .stream_ok(cam_stream_ok), .last_error(cam_last_error),
        .good_frames(cam_good_frames), .bad_frames(cam_bad_frames),
        .dropped_pixels(cam_dropped_pixels), .frame_period(cam_frame_period));

    wire       mask_valid, is_red;
    wire [9:0] mask_x, mask_y;
    red_mask u_red_mask (
        .pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y), .pixel_valid(pixel_valid),
        .mask_valid(mask_valid), .is_red(is_red), .mask_x(mask_x), .mask_y(mask_y));

    (* async_reg = "true" *) logic [1:0] target_bypass_sync;
    always_ff @(posedge cam_pclk or negedge rst_n_cam)
        if (!rst_n_cam) target_bypass_sync <= 2'b00;
        else            target_bypass_sync <= {target_bypass_sync[0], SW[1]};
    wire gate_bypass = target_bypass_sync[1];

    wire        cam_result_done, cam_target_valid;
    wire [9:0]  cam_target_x, cam_target_y, c_bx0, c_bx1, c_by0, c_by1;
    wire [31:0] red_count;
    wire [7:0]  c_conf;
    wire [3:0]  c_flags;
    led_centroid u_led_centroid (
        .clk(cam_pclk), .rst_n(rst_n_cam), .gate_bypass(gate_bypass),
        .frame_start(frame_start), .frame_end(frame_end),
        .pixel_valid(mask_valid), .is_red(is_red), .pixel_x(mask_x), .pixel_y(mask_y),
        .frame_done(cam_result_done), .target_valid(cam_target_valid),
        .target_x(cam_target_x), .target_y(cam_target_y), .red_count(red_count),
        .confidence(c_conf), .reject_flags(c_flags),
        .bbox_x_min(c_bx0), .bbox_x_max(c_bx1), .bbox_y_min(c_by0), .bbox_y_max(c_by1));

    // ---- the result words for the grab ------------------------------------------------------------
    wire [32*N_RESULTS-1:0] results;
    wire                    kept_done;
    vision_grab_results #(.N_RESULTS(N_RESULTS)) results_u (
        .clk(cam_pclk), .rst_n(rst_n_cam), .gate_bypass(gate_bypass),
        .frame_start(frame_start), .frame_end(frame_end), .frame_good(cam_frame_good),
        .stream_ok(cam_stream_ok), .last_error(cam_last_error), .good_frames(cam_good_frames),
        .bad_frames(cam_bad_frames), .dropped_pixels(cam_dropped_pixels), .frame_period(cam_frame_period),
        .mask_valid(mask_valid), .is_red(is_red), .mask_x(mask_x), .mask_y(mask_y),
        .full_done(cam_result_done), .full_valid(cam_target_valid), .full_x(cam_target_x), .full_y(cam_target_y),
        .full_count(red_count), .full_conf(c_conf), .full_flags(c_flags),
        .full_bx0(c_bx0), .full_bx1(c_bx1), .full_by0(c_by0), .full_by1(c_by1),
        .results(results), .kept_done(kept_done));

    // ---- the frame grab: KEY1 or the PC -> the next whole frame + its words -> memory GRAB ----------
    wire snap_busy, snap_ready;
    frame_snap #(.MODE(0), .N_RESULTS(N_RESULTS)) snap_u (
        .pclk(cam_pclk), .reset(~rst_n_cam), .frame_start(cam_frame_start), .px_valid(cam_pixel_valid),
        .pixel(cam_pixel), .x(cam_x), .y(cam_y),
        .results(results), .trigger(~KEY[1]), .busy(snap_busy), .ready(snap_ready));

    // ---- show it: frame_buffer -> camera_display -> VGA ---------------------------------------------
    wire [8:0]  rd_x;
    wire [7:0]  rd_y;
    wire [15:0] rd_pixel, shown_pixel;
    frame_buffer #(.W(320), .H(240), .DW(16), .DECIMATE(2), .BUFFERS(1), .PACKED(0)) fb_u (
        .wr_clk(cam_pclk), .wr_frame_start(cam_frame_start), .wr_valid(cam_pixel_valid), .wr_x(cam_x),
        .wr_y(cam_y), .wr_pixel(cam_pixel),
        .rd_clk(clk_25), .rd_x(rd_x), .rd_y(rd_y), .rd_pixel(rd_pixel));

    // the robot's latest target, for the cross (PCLK -> clk_25; a torn value only misplaces the cross for one
    // VGA frame, so the bits are just synchronised: display only, never fed back)
    logic        tgt_valid_p = 1'b0;
    logic [8:0]  tgt_u_p = '0;
    logic [7:0]  tgt_v_p = '0;
    always_ff @(posedge cam_pclk)
        if (cam_result_done) begin
            tgt_valid_p <= cam_target_valid;
            tgt_u_p     <= cam_target_x[9:1];
            tgt_v_p     <= cam_target_y[8:1];
        end
    (* async_reg = "true" *) logic [17:0] tgt_s1 = '0, tgt_s2 = '0;
    always_ff @(posedge clk_25) begin
        tgt_s1 <= {tgt_valid_p, tgt_u_p, tgt_v_p};
        tgt_s2 <= tgt_s1;
    end
    logic [8:0] rd_x_q = '0;
    logic [7:0] rd_y_q = '0;
    always_ff @(posedge clk_25) begin rd_x_q <= rd_x; rd_y_q <= rd_y; end      // rd_pixel is one clock behind
    wire [8:0] du = (rd_x_q > tgt_s2[16:8]) ? rd_x_q - tgt_s2[16:8] : tgt_s2[16:8] - rd_x_q;
    wire [7:0] dv = (rd_y_q > tgt_s2[7:0])  ? rd_y_q - tgt_s2[7:0]  : tgt_s2[7:0]  - rd_y_q;
    wire on_cross = SHOW_TARGET && tgt_s2[17] && ((du == 9'd0 && dv <= 8'd8) || (dv == 8'd0 && du <= 9'd8));
    assign shown_pixel = on_cross ? 16'h07E0 : rd_pixel;

    wire [29:0] st_data;
    wire        st_sop, st_eop, st_valid, st_ready;
    camera_display #(.H_RES(640), .V_RES(480), .W(320), .H(240)) disp_u (
        .clk(clk_25), .reset(reset25), .hold_enable(1'b1), .rd_x(rd_x), .rd_y(rd_y), .rd_pixel(shown_pixel),
        .data(st_data), .startofpacket(st_sop), .endofpacket(st_eop), .valid(st_valid), .ready(st_ready));

    vga_sink vga_u (
        .clk_clk(clk_25), .reset_reset_n(~reset25),
        .video_in_data(st_data), .video_in_startofpacket(st_sop), .video_in_endofpacket(st_eop),
        .video_in_valid(st_valid), .video_in_ready(st_ready),
        .vga_CLK(VGA_CLK), .vga_HS(VGA_HS), .vga_VS(VGA_VS), .vga_BLANK(VGA_BLANK_N), .vga_SYNC(VGA_SYNC_N),
        .vga_R(VGA_R), .vga_G(VGA_G), .vga_B(VGA_B));

    // ---- status LEDs ---------------------------------------------------------------------------------
    logic frame_toggle = 1'b0, last_valid = 1'b0, last_good = 1'b0;
    always_ff @(posedge cam_pclk) begin
        if (cam_frame_start) frame_toggle <= ~frame_toggle;
        if (cam_result_done) last_valid <= cam_target_valid;
        if (frame_end)       last_good  <= cam_frame_good;
    end
    assign LEDR = {cam_held, hold_error, gate_bypass, last_good, last_valid, snap_busy, snap_ready, frame_toggle,
                   cfg_error, cfg_done};
endmodule

`default_nettype wire
