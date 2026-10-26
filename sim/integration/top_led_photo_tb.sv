`timescale 1ns/1ps
`default_nettype none

// Group 26 V2 (M2, Wilson) - whole-robot regression from the camera pins.
//
// The REAL major_project_top, with nothing forced and no stand-ins, sees a
// busy lab photo with LEDs on it, and the bench checks what goes out of the
// UART to the chassis. Every external device is Ed's behavioural model:
//
//   ov7670_model   configured over SCCB by the real ov7670_config; its scene
//                  is a lab photo (sim/data/*_scene.hex, from
//                  sim/tools/make_led_frames.py), streamed at 40 MHz PCLK
//   hcsr04_model   on GPIO_1[27]/[29], object at 1000 mm (later 200 mm)
//   ir_remote_model on IRDA_RXD: the operator presses CH+ to release the stop
//   ugv02_model    on GPIO_1[31]: decodes the JSON drive commands
//
// Operator procedure: SW0 OFF at power-up, press CH+ on the remote, then SW0
// ON. (The IR latch starts set; docking_fsm holds STOPPED while it is set
// and leaves STOPPED only through SW0 OFF -> IDLE. Arming before the release
// leaves the robot in STOPPED: the "SW0 OFF->ON" note in the README.)
//
// Scenes, in order (the camera is never stopped; a scene is swapped at VSYNC
// and the second complete frame after the swap is checked):
//   photo_leds        red LED right of centre, on the pink phone (430,205),
//                     with green and blue LEDs as distractors
//   photo_red_left    red LED left of centre (150,330), blue distractor
//   photo_green_blue  green and blue LEDs only: no red target
//   photo_only        the photo alone
//   photo_leds        red target reacquired
//   photo_leds + sonar 200 mm: docked, wheels commanded to zero
//
// Per scene: vision result (valid, x, y), LEDR2/3/4, FSM state, the gated
// wheel PWM against steering_controller's formula, and the PWM the UGV02
// model decoded from the UART.

module top_led_photo_tb;

    logic       CLOCK_50 = 1'b0;
    logic [3:0] KEY = 4'b1111;
    logic [9:0] SW  = 10'd0;
    wire        IRDA_RXD;
    wire  [9:0] LEDR;
    wire  [35:0] GPIO_1;

    always #10 CLOCK_50 = ~CLOCK_50;

    major_project_top #(
        .CAMERA_FPS(50),
        .UART_PERIOD_MS(10)
    ) dut (
        .CLOCK_50(CLOCK_50), .KEY(KEY), .SW(SW), .IRDA_RXD(IRDA_RXD),
        .GPIO_1(GPIO_1), .LEDR(LEDR)
    );

    // ---------------- camera ---------------------------------------------
    wire pclk, vsync, href;
    wire [7:0] cam_d;
    assign GPIO_1[21] = pclk;
    assign GPIO_1[22] = href;
    assign GPIO_1[23] = vsync;
    assign GPIO_1[19:12] = cam_d;
    pullup (GPIO_1[24]);

    int cam_frames, cam_writes, cam_errors;
    ov7670_model #(
        .VERBOSE(0),
        .SCENE_FILE("sim/data/photo_leds_scene.hex")
    ) cam (
        .sio_c(GPIO_1[25]), .sio_d(GPIO_1[24]), .xclk(GPIO_1[20]),
        .reset_n(GPIO_1[11]), .pwdn(GPIO_1[10]),
        .pclk(pclk), .vsync(vsync), .href(href), .d(cam_d),
        .scene_mode(0), .scene_x0(0), .scene_y0(0), .scene_code(8'h00),
        .frames_done(cam_frames), .write_count(cam_writes), .error_count(cam_errors)
    );

    // ---------------- ultrasonic -------------------------------------------
    int sonar_distance_mm = 1000;
    wire sonar_echo;
    int sonar_meas, sonar_errs, sonar_notes, sonar_last_us;
    assign GPIO_1[29] = sonar_echo;
    hcsr04_model #(.VERBOSE(0)) sonar (
        .trig(GPIO_1[27]), .echo(sonar_echo),
        .distance_mm(sonar_distance_mm), .object_present(1'b1),
        .measurements(sonar_meas), .errors(sonar_errs),
        .notes(sonar_notes), .last_echo_us(sonar_last_us)
    );

    // ---------------- IR remote --------------------------------------------
    int ir_sent, ir_rep, ir_dec, ir_rep_dec, ir_mon_err;
    logic [31:0] ir_last;
    ir_remote_model #(.VERBOSE(0)) remote (
        .irda_rxd(IRDA_RXD),
        .frames_sent(ir_sent), .repeats_sent(ir_rep),
        .frames_decoded(ir_dec), .repeats_decoded(ir_rep_dec),
        .last_decoded(ir_last), .monitor_errors(ir_mon_err)
    );

    // ---------------- chassis -----------------------------------------------
    real ugv_x, ugv_y, ugv_th, ugv_vl, ugv_vr, ugv_cl, ugv_cr;
    int  ugv_cmds, ugv_problems;
    ugv02_model #(.VERBOSE(0)) ugv (
        .rx(GPIO_1[31]), .tx(),
        .x_m(ugv_x), .y_m(ugv_y), .heading_rad(ugv_th),
        .v_left(ugv_vl), .v_right(ugv_vr),
        .cmd_left(ugv_cl), .cmd_right(ugv_cr),
        .commands(ugv_cmds), .problems(ugv_problems)
    );

    // ---------------- results ------------------------------------------------
    int errors = 0, checks = 0;

    task automatic expect_eq(string what, int got, int exp);
        checks++;
        if (got != exp) begin errors++; $display("    FAIL %-30s got %0d, expected %0d", what, got, exp); end
        else $display("    ok   %-30s %0d", what, got);
    endtask

    task automatic expect_near(string what, int got, int exp, int tol);
        checks++;
        if (got < exp - tol || got > exp + tol) begin
            errors++; $display("    FAIL %-30s got %0d, expected %0d +/-%0d", what, got, exp, tol);
        end else $display("    ok   %-30s %0d (expected %0d)", what, got, exp);
    endtask

    // Independent model of steering_controller (V2 defaults): centre 320,
    // deadband 20, speed 80 / 65 (|e| > 20) / 50 (|e| > 80), Kp 1/4,
    // clamp 0..120.
    function automatic int steer(int x, bit left);
        int e = x - 320, ae, base, c = 0, v;
        ae = (e < 0) ? -e : e;
        base = (ae > 80) ? 50 : (ae > 20) ? 65 : 80;
        if (ae > 20) c = e / 4;
        v = left ? base + c : base - c;
        return (v < 0) ? 0 : (v > 120) ? 120 : v;
    endfunction

    // Every vision result as it reaches the 50 MHz domain.
    bit r_valid; int r_x, r_y, n_results = 0;
    always @(posedge CLOCK_50)
        if (dut.nav_frame_done) begin
            r_valid = dut.nav_target_valid;
            r_x = int'(dut.nav_target_x);
            r_y = int'(dut.nav_target_y);
            n_results = n_results + 1;
        end

    // Swap the scene during vertical sync, then wait for the second
    // complete frame result after the swap (the first is entirely new too;
    // the second makes the check independent of when the swap landed).
    task automatic show(string scene);
        int n0;
        @(posedge vsync);
        $readmemh({"sim/data/", scene, "_scene.hex"}, cam.video.scene_mem);
        n0 = n_results;
        wait (n_results >= n0 + 2);
        // Two UART periods so the chassis has received the new command.
        #25ms;
        $display("  %-17s red valid=%0b x=%0d y=%0d n=%0d | LEDR=%b state=%0d enable=%0b PWM L=%0d R=%0d | UGV02 PWM L=%0d R=%0d (%0d cmds)",
                 scene, r_valid, r_x, r_y, dut.red_count, LEDR, dut.state_debug,
                 dut.safe_motor_enable, int'(dut.chassis_left), int'(dut.chassis_right),
                 ugv_pwm(1), ugv_pwm(0), ugv_cmds);
        $fflush();
    endtask

    // The PWM the chassis model decoded, back from m/s (V_PER_PWM = 1/512).
    function automatic int ugv_pwm(bit left);
        real v;
        ugv.update();
        v = left ? ugv_cl : ugv_cr;
        return int'(v * 512.0);          // int'() of a real rounds to nearest
    endfunction

    task automatic check_driving_to(int x, int y);
        int l = steer(r_x, 1), r = steer(r_x, 0);
        expect_eq  ("target valid",          int'(r_valid), 1);
        expect_near("target x",              r_x, x, 3);
        expect_near("target y",              r_y, y, 3);
        expect_eq  ("LEDR2 target seen",     int'(LEDR[2]), 1);
        expect_eq  ("LEDR3 target left",     int'(LEDR[3]), (r_x < 280) ? 1 : 0);
        expect_eq  ("LEDR4 target right",    int'(LEDR[4]), (r_x > 360) ? 1 : 0);
        expect_eq  ("FSM APPROACH (2)",      int'(dut.state_debug), 2);
        expect_eq  ("safe_motor_enable",     int'(dut.safe_motor_enable), 1);
        expect_eq  ("gated PWM left",        int'(dut.chassis_left),  l);
        expect_eq  ("gated PWM right",       int'(dut.chassis_right), r);
        expect_near("UGV02 decoded PWM left",  ugv_pwm(1), l, 1);
        expect_near("UGV02 decoded PWM right", ugv_pwm(0), r, 1);
    endtask

    task automatic check_stopped(string why, int state);
        expect_eq("target valid",        int'(r_valid), (state == 3) ? 1 : 0);
        expect_eq($sformatf("FSM %s (%0d)", why, state), int'(dut.state_debug), state);
        expect_eq("safe_motor_enable",   int'(dut.safe_motor_enable), 0);
        expect_eq("gated PWM left",      int'(dut.chassis_left), 0);
        expect_eq("gated PWM right",     int'(dut.chassis_right), 0);
        expect_eq("UGV02 decoded PWM left",  ugv_pwm(1), 0);
        expect_eq("UGV02 decoded PWM right", ugv_pwm(0), 0);
    endtask

    initial begin
        $display("top_led_photo_tb: real major_project_top, Ed camera/sonar/IR/UGV02 models, lab-photo scenes");

        // ---- power-up: camera configuration -----------------------------
        fork
            wait (dut.cfg_done);
            begin #80ms; $fatal(1, "FAIL: top_led_photo_tb: camera not configured within 80 ms"); end
        join_any
        disable fork;
        $display("  [%0t] camera configured over SCCB (%0d writes)", $time, cam_writes);

        // ---- operator: CH+ releases the IR stop, then arm -------------------
        if (dut.emergency_stop !== 1'b1)
            $fatal(1, "FAIL: top_led_photo_tb: IR stop latch should be set after reset");
        remote.press(8'h1A);                                    // CH+
        fork
            wait (dut.emergency_stop == 1'b0);
            begin #300ms; $fatal(1, "FAIL: top_led_photo_tb: CH+ did not release the IR stop"); end
        join_any
        disable fork;
        #1ms SW[0] = 1'b1;
        $display("  [%0t] IR stop released by CH+, SW0 armed", $time);

        $display("Scene: red LED right of centre (pink phone), green + blue distractors");
        show("photo_leds");
        check_driving_to(430, 205);

        $display("Scene: red LED left of centre");
        show("photo_red_left");
        check_driving_to(150, 330);

        $display("Scene: green + blue LEDs only");
        show("photo_green_blue");
        check_stopped("ACQUIRE", 1);

        $display("Scene: photo, no LEDs");
        show("photo_only");
        check_stopped("ACQUIRE", 1);

        $display("Scene: red LED back");
        show("photo_leds");
        check_driving_to(430, 205);

        $display("Scene: red LED, obstacle now at 200 mm");
        sonar_distance_mm = 200;
        fork
            wait (dut.state_debug == 3'd3);
            begin #400ms; end
        join_any
        disable fork;
        show("photo_leds");
        check_stopped("DOCKED", 3);

        expect_eq("vision CDC overrun (LEDR8)", int'(LEDR[8]), 0);
        expect_eq("SCCB protocol errors",       cam_errors, 0);
        expect_eq("sonar model rule errors",    sonar_errs, 0);
        expect_eq("UGV02 protocol problems",    ugv_problems, 0);

        if (errors != 0)
            $fatal(1, "FAIL: top_led_photo_tb: %0d errors in %0d checks", errors, checks);
        $display("PASS: top_led_photo_tb (%0d checks, %0d camera frames, %0d UART drive commands)",
                 checks, cam_frames, ugv_cmds);
        $finish;
    end

    initial begin
        #3s;
        $fatal(1, "FAIL: top_led_photo_tb: timeout");
    end

endmodule

`default_nettype wire
