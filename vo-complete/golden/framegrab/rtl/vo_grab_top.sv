`timescale 1ns/1ps
`default_nettype none
`include "vo_calibration.svh"

/*
 *  vo_grab_top.sv -- Group 26 (Wilson, M2): the beacon visual odometry on the bench, with the JTAG photo.
 *
 *      camera --> ov7670_capture --> camera_stream_adapter --+--> beacon_vo (detect, geometry, tracker) --+
 *                    |                                       +--> beacon_detect on the kept pixels (KEPT) -+--> vo_grab_results
 *                    +--> frame_snap (320 x 240 photo + 21 words) <---------------------------------------+
 *                    +--> frame_buffer --> VGA, a cross on each LED found, in its colour
 *      the tracked LED's range (or velocity) on HEX5..0
 *
 *  For measuring the VO before it drives the robot: put the robot at tape-measured spots, read the
 *  range on the HEX displays, take a photo (golden/vo/vo_take_photo.py) and check it against the
 *  golden model and the tape (vo_validate_grab.py), and calibrate the camera from such photos
 *  (vo_calibrate.py, which rewrites rtl/vo/vo_calibration.svh). Everything runs in the camera's
 *  PCLK here (beacon_vo's two clocks tied together), so the words need no crossing.
 *
 *  Controls: KEY0 reset, KEY1 snapshot (or the PC); SW[5:4] the LED to track (0 red, 1 green,
 *  2 blue); SW2 up shows velocity (mm/s) instead of range (mm); SW9 up holds the camera's exposure
 *  and white balance. LEDs: 0 camera configured, 1 SCCB error, 2 frame toggle, 3 snapshot held,
 *  4 snapshot being taken, 5 track valid, 6 frame passed the adapter, 7 two LEDs at the right
 *  spacing (pair_ok), 8 hold error, 9 camera held.
 */
module vo_grab_top #(
    parameter int CAMERA_FPS = 50
) (
    input  wire        CLOCK_50,
    input  wire [3:0]  KEY,
    input  wire [9:0]  SW,
    output wire [9:0]  LEDR,
    output wire [6:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5,
    inout  wire [35:0] GPIO_1,
    output wire        VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N,
    output wire [7:0]  VGA_R, VGA_G, VGA_B
);
    localparam int N_RESULTS = 21;

    // ---- clocks, pins, resets: as vision_grab_top ---------------------------------------------
    wire clk_25, clk_10, pll_locked;
    video_pll video_pll_u (.refclk(CLOCK_50), .rst(1'b0), .outclk_0(clk_25), .outclk_1(clk_10), .locked(pll_locked));
    wire sccb_clk;
    clock_divider #(.DIVISOR(250)) sccb_clk_u (.clk_in(clk_25), .clk_out(sccb_clk));

    wire       cam_pclk  = GPIO_1[21];
    wire       cam_href  = GPIO_1[22];
    wire       cam_vsync = GPIO_1[23];
    wire [7:0] cam_d     = GPIO_1[19:12];
    wire       cam_reset_n, cam_pwdn, cfg_sioc, hold_sioc;
    assign GPIO_1[20] = clk_10;
    assign GPIO_1[11] = cam_reset_n;
    assign GPIO_1[10] = cam_pwdn;

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

    wire cfg_done, cfg_error;
    ov7670_config #(.FPS(CAMERA_FPS)) cfg_u (
        .clk(sccb_clk), .reset(reset_sccb), .sioc(cfg_sioc), .siod(GPIO_1[24]),
        .cam_reset_n(cam_reset_n), .cam_pwdn(cam_pwdn), .done(cfg_done), .error(cfg_error));
    wire cam_held, hold_error;
    ov7670_hold hold_u (
        .clk(sccb_clk), .reset(reset_sccb), .cfg_done(cfg_done), .hold(SW[9]),
        .sioc(hold_sioc), .siod(GPIO_1[24]), .held(cam_held), .busy(), .error(hold_error));
    assign GPIO_1[25] = cfg_done ? hold_sioc : cfg_sioc;

    // ---- capture and the adapter --------------------------------------------------------------
    wire        cam_frame_start, cam_pixel_valid;
    wire [15:0] cam_pixel;
    wire [9:0]  cam_x;
    wire [8:0]  cam_y;
    ov7670_capture cap_u (
        .pclk(cam_pclk), .vsync(cam_vsync), .href(cam_href), .d(cam_d),
        .frame_start(cam_frame_start), .valid(cam_pixel_valid), .pixel(cam_pixel), .x(cam_x), .y(cam_y));

    wire        frame_start, frame_end, pixel_valid, frame_good, stream_ok;
    wire [15:0] pixel_rgb565, good_frames, bad_frames, dropped_pixels;
    wire [9:0]  pixel_x, pixel_y;
    wire [1:0]  last_error;
    wire [23:0] frame_period;
    camera_stream_adapter #(.W(640), .H(480)) u_camera_adapter (
        .clk(cam_pclk), .rst_n(rst_n_cam),
        .cam_frame_start(cam_frame_start), .cam_valid(cam_pixel_valid), .cam_pixel(cam_pixel),
        .cam_x(cam_x), .cam_y(cam_y),
        .frame_start(frame_start), .frame_end(frame_end), .pixel_valid(pixel_valid),
        .pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y),
        .frame_good(frame_good), .stream_ok(stream_ok), .last_error(last_error),
        .good_frames(good_frames), .bad_frames(bad_frames), .dropped_pixels(dropped_pixels),
        .frame_period(frame_period));

    // ---- the VO (both sides on PCLK here) -----------------------------------------------------
    (* async_reg = "true" *) logic [1:0] sel_s1 = '0, sel_s2 = '0;
    (* async_reg = "true" *) logic [1:0] showv_s = '0;
    always_ff @(posedge cam_pclk) begin
        sel_s1 <= SW[5:4]; sel_s2 <= sel_s1; showv_s <= {showv_s[0], SW[2]};
    end
    wire [1:0] sel = (sel_s2 == 2'd3) ? 2'd2 : sel_s2;

    wire        vo_updated, track_valid, settled, pair_valid, pair_ok;
    wire signed [15:0] range_mm, vel_mm_s, lateral_mm, disp_mm, heading_q12;
    wire [2:0]  led_valid, dbg_mask;
    wire [47:0] led_fwd, led_lat;
    wire [2:0]  t_coast;
    wire        d_done;
    wire [2:0]  d_valid;
    wire [41:0] d_x, d_y;
    wire [56:0] d_count;
    wire [14:0] d_flags;
    beacon_vo #(.F_Q4(`VO_F_Q4), .CX_Q4(`VO_CX_Q4), .CY_Q4(`VO_CY_Q4), .SIN_Q14(`VO_SIN_Q14),
                .COS_Q14(`VO_COS_Q14), .DH_MM(`VO_DH_MM)) u_vo (
        .pclk(cam_pclk), .rst_n_pclk(rst_n_cam),
        .frame_start(frame_start), .frame_end(frame_end), .frame_good(frame_good),
        .pixel_valid(pixel_valid), .pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y),
        .dbg_mask(dbg_mask),
        .clk(cam_pclk), .rst_n(rst_n_cam), .sel(sel),
        .standoff_enable(1'b0), .standoff_approaching(1'b0), .standoff_target_mm(16'd0),
        .vo_updated(vo_updated), .track_valid(track_valid), .track_settled(settled),
        .range_mm(range_mm), .vel_mm_s(vel_mm_s), .lateral_mm(lateral_mm), .disp_mm(disp_mm),
        .standoff_reached(), .standoff_lost(),
        .led_valid(led_valid), .led_forward_mm(led_fwd), .led_lateral_mm(led_lat),
        .pair_valid(pair_valid), .pair_ok(pair_ok), .heading_q12(heading_q12),
        .det_dropped(), .cdc_overrun(), .track_coast(t_coast),
        .det_done(d_done), .det_valid(d_valid), .det_x_q4(d_x), .det_y_q4(d_y), .det_count(d_count),
        .det_flags(d_flags));

    // ---- KEPT: the same detection on the photo's pixels only -------------------------------------
    wire        kept_px = pixel_valid && !pixel_x[0] && !pixel_y[0];
    wire        k_done, k_good;
    wire [2:0]  k_valid;
    wire [41:0] k_x, k_y;
    wire [56:0] k_count;
    wire [14:0] k_flags;
    beacon_detect #(.MIN_PIXELS(2), .MAX_PIXELS(1500), .W(320), .H(240)) u_kept (
        .clk(cam_pclk), .rst_n(rst_n_cam),
        .frame_start(frame_start), .frame_end(frame_end), .frame_good(frame_good),
        .pixel_valid(kept_px), .pixel_rgb565(pixel_rgb565), .pixel_x({1'b0, pixel_x[9:1]}),
        .pixel_y({1'b0, pixel_y[9:1]}),
        .done(k_done), .res_good(k_good), .valid(k_valid), .x_q4(k_x), .y_q4(k_y), .count(k_count),
        .flags(k_flags), .bbox(), .dropped(), .dbg_mask());

    wire [32*N_RESULTS-1:0] results;
    vo_grab_results results_u (
        .clk(cam_pclk), .rst_n(rst_n_cam), .frame_start(frame_start), .frame_end(frame_end),
        .frame_good(frame_good), .sel(sel),
        .f_done(d_done), .f_valid(d_valid), .f_x(d_x), .f_y(d_y), .f_count(d_count), .f_flags(d_flags),
        .k_done(k_done), .k_valid(k_valid), .k_x(k_x), .k_y(k_y), .k_count(k_count), .k_flags(k_flags),
        .vo_updated(vo_updated), .led_valid(led_valid), .led_fwd(led_fwd), .led_lat(led_lat),
        .pair_valid(pair_valid), .pair_ok(pair_ok), .heading_q12(heading_q12),
        .track_valid(track_valid), .settled(settled), .range_mm(range_mm), .vel_mm_s(vel_mm_s),
        .lateral_mm(lateral_mm), .disp_mm(disp_mm), .coast(t_coast),
        .results(results));

    wire snap_busy, snap_ready;
    frame_snap #(.MODE(0), .N_RESULTS(N_RESULTS)) snap_u (
        .pclk(cam_pclk), .reset(~rst_n_cam), .frame_start(cam_frame_start), .px_valid(cam_pixel_valid),
        .pixel(cam_pixel), .x(cam_x), .y(cam_y),
        .results(results), .trigger(~KEY[1]), .busy(snap_busy), .ready(snap_ready));

    // ---- HEX: the tracked LED's range or velocity --------------------------------------------------
    vo_hex_display hex_u (
        .clk(cam_pclk), .rst_n(rst_n_cam), .updated(vo_updated), .sel(sel), .track_valid(track_valid),
        .range_mm(range_mm), .vel_mm_s(vel_mm_s), .show_vel(showv_s[1]),
        .hex0(HEX0), .hex1(HEX1), .hex2(HEX2), .hex3(HEX3), .hex4(HEX4), .hex5(HEX5));

    // ---- VGA: the picture with a cross on each LED found, in its colour ------------------------------
    wire [8:0]  rd_x;
    wire [7:0]  rd_y;
    wire [15:0] rd_pixel;
    frame_buffer #(.W(320), .H(240), .DW(16), .DECIMATE(2), .BUFFERS(1), .PACKED(0)) fb_u (
        .wr_clk(cam_pclk), .wr_frame_start(cam_frame_start), .wr_valid(cam_pixel_valid), .wr_x(cam_x),
        .wr_y(cam_y), .wr_pixel(cam_pixel),
        .rd_clk(clk_25), .rd_x(rd_x), .rd_y(rd_y), .rd_pixel(rd_pixel));

    // centroids at PCLK, halved onto the 320 x 240 picture; display only, so plain two-flop copies
    logic [2:0]  cv_p = '0;
    logic [26:0] cxy_p = '0;               // 3 x 9 bits of x/2
    logic [23:0] cy_p  = '0;               // 3 x 8 bits of y/2
    always_ff @(posedge cam_pclk)
        if (d_done) begin
            cv_p <= d_valid;
            for (int k = 0; k < 3; k++) begin
                cxy_p[k*9 +: 9] <= d_x[k*14 + 5 +: 9];      // Q4 -> pixels, then / 2
                cy_p[k*8 +: 8]  <= d_y[k*14 + 5 +: 8];
            end
        end
    (* async_reg = "true" *) logic [53:0] ov_s1 = '0, ov_s2 = '0;
    always_ff @(posedge clk_25) begin ov_s1 <= {cv_p, cxy_p, cy_p}; ov_s2 <= ov_s1; end
    logic [8:0] rx_q = '0;
    logic [7:0] ry_q = '0;
    always_ff @(posedge clk_25) begin rx_q <= rd_x; ry_q <= rd_y; end
    wire [2:0] on_cross;
    genvar gk;
    generate
        for (gk = 0; gk < 3; gk++) begin : g_cross
            wire [8:0] cx = ov_s2[24 + gk*9 +: 9];
            wire [7:0] cy = ov_s2[gk*8 +: 8];
            wire [8:0] du = (rx_q > cx) ? rx_q - cx : cx - rx_q;
            wire [7:0] dv = (ry_q > cy) ? ry_q - cy : cy - ry_q;
            assign on_cross[gk] = ov_s2[51 + gk] && ((du == 9'd0 && dv <= 8'd6) || (dv == 8'd0 && du <= 9'd6));
        end
    endgenerate
    wire [15:0] shown = on_cross[0] ? 16'hF800 : on_cross[1] ? 16'h07E0 : on_cross[2] ? 16'h001F : rd_pixel;

    wire [29:0] st_data;
    wire        st_sop, st_eop, st_valid, st_ready;
    camera_display #(.H_RES(640), .V_RES(480), .W(320), .H(240)) disp_u (
        .clk(clk_25), .reset(reset25), .hold_enable(1'b1), .rd_x(rd_x), .rd_y(rd_y), .rd_pixel(shown),
        .data(st_data), .startofpacket(st_sop), .endofpacket(st_eop), .valid(st_valid), .ready(st_ready));
    vga_sink vga_u (
        .clk_clk(clk_25), .reset_reset_n(~reset25),
        .video_in_data(st_data), .video_in_startofpacket(st_sop), .video_in_endofpacket(st_eop),
        .video_in_valid(st_valid), .video_in_ready(st_ready),
        .vga_CLK(VGA_CLK), .vga_HS(VGA_HS), .vga_VS(VGA_VS), .vga_BLANK(VGA_BLANK_N), .vga_SYNC(VGA_SYNC_N),
        .vga_R(VGA_R), .vga_G(VGA_G), .vga_B(VGA_B));

    // ---- LEDs ------------------------------------------------------------------------------------------
    logic frame_toggle = 1'b0, last_good = 1'b0;
    always_ff @(posedge cam_pclk) begin
        if (cam_frame_start) frame_toggle <= ~frame_toggle;
        if (frame_end)       last_good    <= frame_good;
    end
    assign LEDR = {cam_held, hold_error, pair_ok, last_good, track_valid, snap_busy, snap_ready, frame_toggle,
                   cfg_error, cfg_done};
endmodule

`default_nettype wire
