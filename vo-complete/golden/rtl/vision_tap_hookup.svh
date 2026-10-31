// ====================================================
// vision_tap: golden-model capture probe (R7), Wilson (M2)
// ====================================================
// Paste into rtl/integration/major_project_top.sv after u_led_centroid
// (section 4), and add golden/rtl/vision_tap.sv to the Quartus project.
// It only watches existing nets; nothing the robot uses depends on it.
// Record with SignalTap: clock cam_pclk, data u_vision_tap|tap_word_r[55..0]
// and u_vision_tap|tap_store_r, storage qualifier "Input port" = tap_store_r,
// trigger u_vision_tap|tap_trigger_r = 1, depth 32K. See golden/README.md.
//
// To remove it for the final build, delete this block (or leave it: about
// 120 ALMs, no memory; the memory belongs to SignalTap).

    wire [55:0] vision_tap_word;
    wire        vision_tap_store;
    wire        vision_tap_trigger;

    vision_tap #(
        .BAND_ROWS(8),        // dense rows per frame (all pixels recorded)
        .BAND_FOLLOW(1'b1)    // keep the band on the last valid target
    ) u_vision_tap (
        .clk(cam_pclk),
        .rst_n(rst_n_cam),

        .cam_frame_start(cam_frame_start),
        .cam_valid(cam_pixel_valid),
        .cam_pixel(cam_pixel),
        .cam_x(cam_x),
        .cam_y(cam_y),

        .frame_start(frame_start),
        .frame_end(frame_end),
        .frame_good(cam_frame_good),
        .pixel_valid(pixel_valid),
        .is_red(is_red),

        .result_done(cam_result_done),
        .target_valid(cam_target_valid),
        .target_x(cam_target_x),
        .target_y(cam_target_y),
        .red_count(red_count),
        .bypass(target_bypass_sync[1]),

        .tap_word(vision_tap_word),
        .tap_store(vision_tap_store),
        .tap_trigger(vision_tap_trigger)
    );
