`timescale 1ns/1ps
`default_nettype none

// MTRX3700 Major Project - Group 26
// V1 autonomous red-target docking robot.
//
// Hardware:
//   Terasic DE1-SoC
//   OV7670 RGB565 camera
//   RCWL-1601 ultrasonic sensor
//   DE1-SoC IR receiver
//   Waveshare UGV02 UART motor interface
//
// CLOCK_50: navigation, IR decoding and chassis UART.
// Camera PCLK: RGB565 capture and LED centroid.
// Sonar PLL clock: ultrasonic echo measurement.
//
// KEY0 = system reset (active low).
// SW0 = arm switch (1 = armed).
// SW1 = target-validation bypass (1 = V1 behaviour).
// SW2 = sonar-filter bypass (1 = V1 behaviour).
// SW3 = operator reverse authorisation (1 = cleared reverse corridor).
// SW4 = camera automatic-recovery bypass (1 = disabled).
// IR Power = emergency stop.
// IR CH+   = release emergency-stop latch.
//
// Safety:
//   Motor output is disabled until explicitly armed.
//   Missing targets or invalid sonar readings inhibit motion.
//   Emergency stop is latched.
//   The UART interface sends zero-PWM stop commands when disabled.
//
// IMPORTANT:
// UART stop commands are not instantaneous electrical motor
// cutoffs. Physical stopping latency must be measured.

module major_project_top #(
    parameter int CAMERA_FPS = 50,
    parameter int SONAR_PACE_CLKS = 4_390_400,
    parameter int UART_PERIOD_MS = 10,
    // Camera health monitor timeouts in CLOCK_50 cycles (benches shorten them).
    parameter int CAM_LOST_TIMEOUT    = 5_000_000,   // 100 ms without a frame
    parameter int CAM_RECOVER_TIMEOUT = 10_000_000   // 200 ms before reconfiguring
) (
    input  wire        CLOCK_50,
    input  wire [3:0]  KEY,
    input  wire [9:0]  SW,
    input  wire        IRDA_RXD,

    inout  wire [35:0] GPIO_1,

    output wire [9:0]  LEDR
);

    // ====================================================
    // 1. Clock generation
    // ====================================================

    wire clk_25;
    wire clk_10;
    wire video_locked;

    video_pll u_video_pll (
        .refclk(CLOCK_50),
        .rst(1'b0),
        .outclk_0(clk_25),
        .outclk_1(clk_10),
        .locked(video_locked)
    );

    wire sonar_clk;
    wire sonar_locked;

    sonar_pll u_sonar_pll (
        .refclk(CLOCK_50),
        .rst(1'b0),
        .outclk_0(sonar_clk),
        .locked(sonar_locked)
    );

    wire sccb_clk;

    clock_divider #(
        .DIVISOR(250)
    ) u_sccb_divider (
        .clk_in(clk_25),
        .clk_out(sccb_clk)
    );

    // ====================================================
    // 2. Reset and arm synchronisation
    // ====================================================

    // Reset asserts if KEY0 is pressed or a PLL is unlocked.
    // Each clock domain then synchronously releases reset.
    wire reset_request_n =
        KEY[0] & video_locked & sonar_locked;

    (* async_reg = "true" *) logic [1:0] rs50 = 2'b00;
    (* async_reg = "true" *) logic [1:0] rs_cam = 2'b00;
    (* async_reg = "true" *) logic [1:0] rs_sonar = 2'b00;
    (* async_reg = "true" *) logic [1:0] rs_sccb = 2'b00;

    always_ff @(posedge CLOCK_50 or negedge reset_request_n) begin
        if (!reset_request_n)
            rs50 <= 2'b00;
        else
            rs50 <= {rs50[0], 1'b1};
    end

    wire cam_pclk = GPIO_1[21];

    always_ff @(posedge cam_pclk or negedge reset_request_n) begin
        if (!reset_request_n)
            rs_cam <= 2'b00;
        else
            rs_cam <= {rs_cam[0], 1'b1};
    end

    always_ff @(posedge sonar_clk or negedge reset_request_n) begin
        if (!reset_request_n)
            rs_sonar <= 2'b00;
        else
            rs_sonar <= {rs_sonar[0], 1'b1};
    end

    always_ff @(posedge sccb_clk or negedge reset_request_n) begin
        if (!reset_request_n)
            rs_sccb <= 2'b00;
        else
            rs_sccb <= {rs_sccb[0], 1'b1};
    end

    wire rst_n_50    = rs50[1];
    wire rst_n_cam   = rs_cam[1];
    wire rst_n_sonar = rs_sonar[1];
    wire rst_n_sccb  = rs_sccb[1];

    // Both sides of each CDC share the same asynchronous
    // reset assertion. Their individual pipeline states
    // recover via the request/acknowledge handshake.
    wire cdc_rst_n = reset_request_n;

    (* async_reg = "true" *) logic [1:0] arm_sync = 2'b00;

    always_ff @(posedge CLOCK_50 or negedge rst_n_50) begin
        if (!rst_n_50)
            arm_sync <= 2'b00;
        else
            arm_sync <= {arm_sync[0], SW[0]};
    end

    wire arm = arm_sync[1];

    // ====================================================
    // 3. Camera configuration and pin connections
    // ====================================================

    wire cam_href  = GPIO_1[22];
    wire cam_vsync = GPIO_1[23];
    wire [7:0] cam_d = GPIO_1[19:12];

    wire cam_reset_n;
    wire cam_pwdn;
    wire cam_sioc;

    assign GPIO_1[20] = clk_10;
    assign GPIO_1[11] = cam_reset_n;
    assign GPIO_1[10] = cam_pwdn;
    assign GPIO_1[25] = cam_sioc;

    // GPIO_1[24] is SCCB open-drain data.
    wire cfg_done;
    wire cfg_error;

    // V3 camera recovery (Luke): camera_health_monitor (50 MHz) holds
    // cam_reconfig high for 100 us when the stream is lost. It enters the
    // SCCB domain through two flops and restarts ov7670_config, which
    // pulses the camera's RESET# and writes every register again.
    wire cam_reconfig;
    (* async_reg = "true" *) logic [1:0] cam_reconfig_sccb = 2'b00;

    always_ff @(posedge sccb_clk or negedge rst_n_sccb) begin
        if (!rst_n_sccb)
            cam_reconfig_sccb <= 2'b00;
        else
            cam_reconfig_sccb <= {cam_reconfig_sccb[0], cam_reconfig};
    end

    ov7670_config #(
        .FPS(CAMERA_FPS)
    ) u_camera_config (
        .clk(sccb_clk),
        .reset(~rst_n_sccb | cam_reconfig_sccb[1]),
        .sioc(cam_sioc),
        .siod(GPIO_1[24]),
        .cam_reset_n(cam_reset_n),
        .cam_pwdn(cam_pwdn),
        .done(cfg_done),
        .error(cfg_error)
    );

    // ====================================================
    // 4. Camera capture and red LED perception
    // ====================================================

    wire        cam_frame_start;
    wire        cam_pixel_valid;
    wire [15:0] cam_pixel;
    wire [9:0]  cam_x;
    wire [8:0]  cam_y;

    ov7670_capture u_camera_capture (
        .pclk(cam_pclk),
        .vsync(cam_vsync),
        .href(cam_href),
        .d(cam_d),
        .frame_start(cam_frame_start),
        .valid(cam_pixel_valid),
        .pixel(cam_pixel),
        .x(cam_x),
        .y(cam_y)
    );

    wire        frame_start;
    wire        frame_end;
    wire        pixel_valid;
    wire [15:0] pixel_rgb565;
    wire [9:0]  pixel_x;
    wire [9:0]  pixel_y;

    // V2 frame-integrity stage (Luke): locks to frame_start, checks every
    // pixel's raster position, and closes rejected frames with an empty
    // close so led_centroid reports no target for them. One PCLK latency.
    wire        cam_frame_good;
    wire        cam_stream_ok;
    wire [1:0]  cam_last_error;
    wire [15:0] cam_good_frames;
    wire [15:0] cam_bad_frames;
    wire [15:0] cam_dropped_pixels;
    wire [23:0] cam_frame_period;

    camera_stream_adapter #(
        .W(640),
        .H(480)
    ) u_camera_adapter (
        .clk(cam_pclk),
        .rst_n(rst_n_cam),

        .cam_frame_start(cam_frame_start),
        .cam_valid(cam_pixel_valid),
        .cam_pixel(cam_pixel),
        .cam_x(cam_x),
        .cam_y(cam_y),

        .frame_start(frame_start),
        .frame_end(frame_end),
        .pixel_valid(pixel_valid),
        .pixel_rgb565(pixel_rgb565),
        .pixel_x(pixel_x),
        .pixel_y(pixel_y),

        .frame_good(cam_frame_good),
        .stream_ok(cam_stream_ok),
        .last_error(cam_last_error),
        .good_frames(cam_good_frames),
        .bad_frames(cam_bad_frames),
        .dropped_pixels(cam_dropped_pixels),
        .frame_period(cam_frame_period)
    );

    wire        mask_valid;
    wire        is_red;
    wire [9:0]  mask_x;
    wire [9:0]  mask_y;

    red_mask u_red_mask (
        .pixel_rgb565(pixel_rgb565),
        .pixel_x(pixel_x),
        .pixel_y(pixel_y),
        .pixel_valid(pixel_valid),

        .mask_valid(mask_valid),
        .is_red(is_red),
        .mask_x(mask_x),
        .mask_y(mask_y)
    );

    wire        cam_result_done;
    wire        cam_target_valid;
    wire [9:0]  cam_target_x;
    wire [9:0]  cam_target_y;
    wire [31:0] red_count;

    // ====================================================
    // R9: SW1 target-validation bypass
    // 0 = V2 validation enabled
    // 1 = V1-equivalent target acceptance
    // ====================================================

    (* async_reg = "true" *) logic [1:0] target_bypass_sync;

    always_ff @(posedge cam_pclk or negedge rst_n_cam) begin
        if (!rst_n_cam)
            target_bypass_sync <= 2'b00;
        else
            target_bypass_sync <= {target_bypass_sync[0], SW[1]};
    end

    led_centroid u_led_centroid (
        .clk(cam_pclk),
        .rst_n(rst_n_cam),
        .gate_bypass(target_bypass_sync[1]),     // SW1: 0 = V2, 1 = V1 bypass

        .frame_start(frame_start),
        .frame_end(frame_end),

        .pixel_valid(mask_valid),
        .is_red(is_red),
        .pixel_x(mask_x),
        .pixel_y(mask_y),

        .frame_done(cam_result_done),
        .target_valid(cam_target_valid),
        .target_x(cam_target_x),
        .target_y(cam_target_y),
        .red_count(red_count),
        .confidence(),
        .reject_flags(),
        .bbox_x_min(),
        .bbox_x_max(),
        .bbox_y_min(),
        .bbox_y_max()
    );

    // ====================================================
    // 5. Vision result CDC: PCLK -> 50 MHz
    // ====================================================

    wire vision_cdc_ready;
    wire vision_cdc_overrun;

    wire        nav_frame_done;
    wire        nav_target_valid;
    wire [9:0]  nav_target_x;
    wire [9:0]  nav_target_y;

    vision_result_cdc u_vision_cdc (
        .src_clk(cam_pclk),
        .dst_clk(CLOCK_50),
        .rst_n(cdc_rst_n),

        .src_frame_done(cam_result_done),
        .src_target_valid(cam_target_valid),
        .src_target_x(cam_target_x),
        .src_target_y(cam_target_y),

        .src_ready(vision_cdc_ready),
        .src_overrun(vision_cdc_overrun),

        .dst_frame_done(nav_frame_done),
        .dst_target_valid(nav_target_valid),
        .dst_target_x(nav_target_x),
        .dst_target_y(nav_target_y)
    );

    // ====================================================
    // 6. Ultrasonic trigger pacing
    // ====================================================

    localparam int SONAR_PACE_WIDTH =
        (SONAR_PACE_CLKS > 1) ? $clog2(SONAR_PACE_CLKS) : 1;

    logic [SONAR_PACE_WIDTH-1:0] sonar_pace = '0;
    logic sonar_start = 1'b0;

    // Ed example uses a 16-cycle start pulse.
    // The sonar driver synchronises this input internally.
    always_ff @(posedge sonar_clk or negedge rst_n_sonar) begin
        if (!rst_n_sonar) begin
            sonar_pace  <= '0;
            sonar_start <= 1'b0;
        end else begin
            if (sonar_pace == SONAR_PACE_WIDTH'(SONAR_PACE_CLKS-1))
                sonar_pace <= '0;
            else
                sonar_pace <= sonar_pace + 1'b1;

            sonar_start <=
                (sonar_pace < SONAR_PACE_WIDTH'(16));
        end
    end

    // ====================================================
    // 7. Ed ultrasonic driver
    // ====================================================

    wire sonar_echo = GPIO_1[29];
    wire sonar_trig;

    assign GPIO_1[27] = sonar_trig;

    wire [11:0] sonar_raw_distance;
    wire sonar_raw_ready;
    wire sonar_raw_valid;

    sonar_range u_sonar_range (
        .clk(sonar_clk),
        .start_measure(sonar_start),
        .rst(~rst_n_sonar),
        .echo(sonar_echo),

        .trig(sonar_trig),
        .distance(sonar_raw_distance),
        .ready(sonar_raw_ready),
        .valid(sonar_raw_valid)
    );

    // ====================================================
    // 8. Sonar CDC: 43.904 MHz -> 50 MHz
    // ====================================================

    wire sonar_cdc_ready;
    wire sonar_cdc_overrun;

    wire        sonar_measure_valid;
    wire [15:0] sonar_measurement_mm;

    sonar_result_cdc u_sonar_cdc (
        .src_clk(sonar_clk),
        .dst_clk(CLOCK_50),
        .rst_n(cdc_rst_n),

        .src_measurement_valid(sonar_raw_valid),
        .src_distance(sonar_raw_distance),

        .src_ready(sonar_cdc_ready),
        .src_overrun(sonar_cdc_overrun),

        .measurement_valid(sonar_measure_valid),
        .measurement_mm(sonar_measurement_mm)
    );

    wire        sonar_valid;
    wire [15:0] distance_mm;
    wire        sonar_forward_clear;
    wire        sonar_fresh;

    // V2 ultrasonic filtering and forward-motion safety.
    sonar_filter #(
        .MAX_AGE_CYCLES(12_500_000),
        .MIN_RANGE_MM(16'd20),
        .MAX_RANGE_MM(16'd3500),
        .OUTLIER_MM(16'd80),
        .STOP_MM(16'd250),
        .MAX_MISSES(2)
    ) u_sonar_filter (
        .clk(CLOCK_50),
        .rst_n(rst_n_50),

        // SW2: 0 = V2 sonar filter, 1 = V1 sonar behaviour.
        .bypass(SW[2]),

        .measurement_valid(sonar_measure_valid),
        .measurement_mm(sonar_measurement_mm),

        .sonar_valid(sonar_valid),
        .distance_mm(distance_mm),
        .sonar_fresh(sonar_fresh),
        .forward_clear(sonar_forward_clear),

        // Optional diagnostics.
        .update(),
        .raw_mm(),
        .window_count(),
        .close_suspect(),
        .fault_no_echo(),
        .fault_stale(),
        .rejected_count(),
        .bridged_count(),
        .bypass_active()
    );

    // ====================================================
    // 9. IR receiver and latched safety
    // ====================================================

    (* async_reg = "true" *) logic [1:0] ir_sync = 2'b11;

    always_ff @(posedge CLOCK_50 or negedge rst_n_50) begin
        if (!rst_n_50)
            ir_sync <= 2'b11;
        else
            ir_sync <= {ir_sync[0], IRDA_RXD};
    end

    wire        ir_valid;
    wire [31:0] ir_code;
    wire        ir_repeat;

    nec_decoder #(
        .CLK_HZ(50_000_000)
    ) u_ir_decoder (
        .clk(CLOCK_50),
        .rst(~rst_n_50),
        .ir(ir_sync[1]),
        .valid(ir_valid),
        .code(ir_code),
        .repeat_code(ir_repeat)
    );

    wire emergency_stop;
    wire motion_permitted;

    ir_safety_latch u_ir_safety (
        .clk(CLOCK_50),
        .rst_n(rst_n_50),
        .ir_valid(ir_valid),
        .ir_code(ir_code),

        .emergency_stop(emergency_stop),
        .motion_permitted(motion_permitted)
    );

    // ====================================================
    // 10. Docking FSM, steering and motor safety
    // ====================================================

    wire motor_enable;
    wire signed [15:0] left_pwm;
    wire signed [15:0] right_pwm;

    wire [2:0] state_debug;
    wire [9:0] target_x_debug;

    motion_interface u_motion (
        .clk(CLOCK_50),
        .rst_n(rst_n_50),

        .arm(arm),
        .emergency_stop(emergency_stop),

        .frame_done(nav_frame_done),
        .target_valid(nav_target_valid),
        .target_x(nav_target_x),

        .sonar_valid(sonar_valid),
        .distance_mm(distance_mm),

        .motor_enable(motor_enable),
        .left_pwm(left_pwm),
        .right_pwm(right_pwm),

        .state_debug(state_debug),
        .target_x_debug(target_x_debug)
    );

    // Final output gating.
    // Camera-frame freshness safety gate.
    wire camera_target_fresh;
    wire [9:0] camera_fresh_x;

    camera_frame_watchdog #(
        .MAX_AGE_CYCLES(7_500_000)
    ) u_camera_watchdog (
        .clk(CLOCK_50),
        .rst_n(rst_n_50),
        .frame_done(nav_frame_done),
        .target_valid(nav_target_valid),
        .target_x(nav_target_x),
        .target_fresh(camera_target_fresh),
        .fresh_target_x(camera_fresh_x)
    );

    // V3 reverse-navigation interface. The V3 bay-sequencer will drive
    // these signals. Until then, idle constants permit forward-only V2
    // operation; the top-level safety bench forces them for reverse tests.
    wire nav_rev_request = 1'b0;
    wire signed [15:0] nav_rev_left_pwm = 16'sd0;
    wire signed [15:0] nav_rev_right_pwm = 16'sd0;

    // Preserve the V2 hierarchy names for existing integration tests.
    wire safe_motor_enable;
    wire signed [9:0] chassis_left;
    wire signed [9:0] chassis_right;

    wire [2:0] reverse_mode;
    wire reverse_stop_active;
    wire reverse_permitted;
    wire reverse_active;
    wire reverse_timeout;
    wire reverse_fault;
    wire [2:0] reverse_status;
    wire [15:0] reverse_ms;

    motion_supervisor u_motion_supervisor (
        .clk(CLOCK_50),
        .rst_n(rst_n_50),
        .arm(arm),
        .emergency_stop(emergency_stop),
        .motion_permitted(motion_permitted),
        .camera_target_fresh(camera_target_fresh),
        .sonar_valid(sonar_valid),
        .sonar_forward_clear(sonar_forward_clear),
        .distance_mm(distance_mm),
        .fwd_enable(motor_enable),
        .fwd_left_pwm(left_pwm),
        .fwd_right_pwm(right_pwm),
        .rev_request(nav_rev_request),
        .rev_left_pwm(nav_rev_left_pwm),
        .rev_right_pwm(nav_rev_right_pwm),
        .reverse_enable(SW[3]),
        .drive_enable(safe_motor_enable),
        .left_cmd(chassis_left),
        .right_cmd(chassis_right),
        .mode(reverse_mode),
        .stop_active(reverse_stop_active),
        .reverse_permitted(reverse_permitted),
        .reverse_active(reverse_active),
        .reverse_timeout(reverse_timeout),
        .reverse_fault(reverse_fault),
        .reverse_status(reverse_status),
        .reverse_ms(reverse_ms)
    );

    // ====================================================
    // 11. UGV02 UART output
    // ====================================================

    wire chassis_tx;
    wire chassis_busy;

    assign GPIO_1[31] = chassis_tx;

    chassis_drive #(
        .CLK_HZ(50_000_000),
        .BAUD(115_200),
        .PERIOD_MS(UART_PERIOD_MS)
    ) u_chassis (
        .clk(CLOCK_50),
        .reset(~rst_n_50),

        .enable(safe_motor_enable),
        .left(chassis_left),
        .right(chassis_right),

        .tx(chassis_tx),
        .busy(chassis_busy)
    );

    // ====================================================
    // 12. Status LEDs
    // ====================================================

    // Slow-domain camera configuration status enters
    // the 50 MHz domain via two-flop synchronisers.
    (* async_reg = "true" *) logic [1:0] cfg_sync = 2'b00;

    always_ff @(posedge CLOCK_50 or negedge rst_n_50) begin
        if (!rst_n_50)
            cfg_sync <= 2'b00;
        else
            cfg_sync <= {cfg_sync[0], cfg_done};
    end

    // V3 camera health and dropout recovery (Luke), 50 MHz domain.
    // Frame events cross as bit 0 of the adapter's good/bad counters
    // (one change per frame), synchronised inside the monitor.
    // SW4 ON = bypass: report only, never reconfigure the camera (R9).
    (* async_reg = "true" *) logic [1:0] cam_recover_bypass_sync = 2'b00;

    always_ff @(posedge CLOCK_50 or negedge rst_n_50) begin
        if (!rst_n_50)
            cam_recover_bypass_sync <= 2'b00;
        else
            cam_recover_bypass_sync <= {cam_recover_bypass_sync[0], SW[4]};
    end

    wire        camera_ok;
    wire [2:0]  cam_health_state;
    wire [23:0] cam_good_interval;
    wire [27:0] cam_last_outage;
    wire [7:0]  cam_dropouts;
    wire [7:0]  cam_recoveries;

    camera_health_monitor #(
        .LOST_TIMEOUT(CAM_LOST_TIMEOUT),
        .RECOVER_TIMEOUT(CAM_RECOVER_TIMEOUT)
    ) u_camera_health (
        .clk(CLOCK_50),
        .rst_n(rst_n_50),

        .good_toggle_async(cam_good_frames[0]),
        .bad_toggle_async(cam_bad_frames[0]),
        .cfg_done(cfg_sync[1]),
        .auto_recover_en(~cam_recover_bypass_sync[1]),

        .camera_ok(camera_ok),
        .reconfig(cam_reconfig),
        .state(cam_health_state),

        .good_interval(cam_good_interval),
        .last_outage(cam_last_outage),
        .dropouts(cam_dropouts),
        .recoveries(cam_recoveries)
    );

    // ====================================================
    // V1 TEMPORARY CAMERA DIAGNOSTICS
    // ====================================================
    // 50 MHz domain only. Uses existing vision CDC outputs.
    // SW0 must remain OFF during hardware testing.

    logic [3:0] diag_frame_count;
    logic       diag_frame_heartbeat;
    logic       diag_target_valid;
    logic [9:0] diag_target_x;

    always_ff @(posedge CLOCK_50 or negedge rst_n_50) begin
        if (!rst_n_50) begin
            diag_frame_count     <= 4'd0;
            diag_frame_heartbeat <= 1'b0;
            diag_target_valid    <= 1'b0;
            diag_target_x        <= 10'd0;
        end else if (nav_frame_done) begin
            diag_frame_count <= diag_frame_count + 1'b1;

            // Toggle every 16 completed frames.
            if (diag_frame_count == 4'd15)
                diag_frame_heartbeat <= ~diag_frame_heartbeat;

            diag_target_valid <= nav_target_valid;
            diag_target_x     <= nav_target_x;
        end
    end

    // LEDR0: OV7670 configuration complete
    // LEDR1: Camera frame heartbeat
    // LEDR2: Red target detected in latest frame
    // LEDR3: Target left of centre
    // LEDR4: Target right of centre
    // LEDR5: Emergency stop active
    // LEDR6: Sonar valid
    // LEDR7: Camera healthy (confirmed good frames, no dropout)
    // LEDR8: Vision CDC overrun
    // LEDR9: Sonar CDC overrun

    assign LEDR[0] = cfg_sync[1];
    assign LEDR[1] = diag_frame_heartbeat;
    assign LEDR[2] = diag_target_valid;
    assign LEDR[3] = diag_target_valid && (diag_target_x < 10'd280);
    assign LEDR[4] = diag_target_valid && (diag_target_x > 10'd360);
    assign LEDR[5] = emergency_stop;
    assign LEDR[6] = sonar_valid;
    assign LEDR[7] = camera_ok;
    assign LEDR[8] = vision_cdc_overrun;
    assign LEDR[9] = sonar_cdc_overrun;

endmodule

`default_nettype wire


