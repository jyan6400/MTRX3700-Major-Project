`timescale 1ns/1ps
`default_nettype none

// Group 26 - beacon visual odometry (Wilson, M2): the three LED beacons in one frame.
//
//   adapter stream -> led_colour_mask -> colour_run_filter -> per colour: count, sum x, sum y, box
//                                                             -> after frame_end, one shared divider:
//                                                                centroid in 1/16 pixel (Q4)
//
// Streaming: no frame buffer, no RAM. Same framing as led_centroid: frame_start clears the
// accumulators (its own pixel counts), a pixel on the frame_end cycle counts, and the frame's
// result is computed after frame_end. A frame closed while the previous one is still being divided
// (about 210 clocks) is dropped (dropped pulses); real frames are 800,000 clocks apart.
//
// Gates per colour, as led_centroid's but with no divide: too few / too many pixels, box aspect
// over MAX_ASPECT, fill under MIN_FILL/256 of the box, and EDGE: the blob touches the frame edge,
// so it is clipped and its centroid biased. valid = frame_good and no flag.
//
// Outputs hold from `done` until the next done. Flattened buses, colour k at [k*w +: w]:
// 0 red, 1 green, 2 blue. Golden model: golden/vo/vo_model.py detect_frame / blob_result.
module beacon_detect #(
    parameter int RUN_LEN    = 3,
    parameter int MIN_PIXELS = 6,
    parameter int MAX_PIXELS = 6000,
    parameter int MAX_ASPECT = 4,
    parameter int MIN_FILL   = 64,
    parameter int W          = 640,
    parameter int H          = 480,
    parameter int EDGE_X     = 2
) (
    input  wire        clk,               // camera PCLK
    input  wire        rst_n,
    input  wire        frame_start,       // camera_stream_adapter outputs
    input  wire        frame_end,
    input  wire        frame_good,
    input  wire        pixel_valid,
    input  wire [15:0] pixel_rgb565,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_y,

    output logic        done,             // one clock: a new result
    output logic        res_good,         // the frame was whole (adapter frame_good)
    output logic [2:0]  valid,            // per colour: detected and passed every gate
    output logic [41:0] x_q4,             // 3 x 14: centroid x, 1/16 px
    output logic [41:0] y_q4,             // 3 x 14
    output logic [56:0] count,            // 3 x 19
    output logic [14:0] flags,            // 3 x 5: {edge, low_fill, aspect, too_many, too_few}
    output logic [119:0] bbox,            // 3 x 40: {x_min, x_max, y_min, y_max} (filtered coordinates)
    output logic        dropped,          // a frame closed while busy: no result for it
    output wire  [2:0]  dbg_mask          // filtered {red, green, blue} of the current pixel
);
    // ---------------- classification and run filter (combinational, zero latency) -------------
    wire m_r, m_g, m_b;
    led_colour_mask u_mask (
        .pixel_rgb565(pixel_rgb565), .pixel_x(pixel_x), .pixel_y(pixel_y), .pixel_valid(pixel_valid),
        .mask_valid(), .is_red(m_r), .is_green(m_g), .is_blue(m_b), .mask_x(), .mask_y()
    );
    wire       f_valid, f_r, f_g, f_b;
    wire [9:0] f_x, f_y;
    colour_run_filter #(.RUN_LEN(RUN_LEN)) u_filt (
        .clk(clk), .rst_n(rst_n), .pixel_valid(pixel_valid),
        .is_red_in(m_r), .is_green_in(m_g), .is_blue_in(m_b),
        .pixel_x(pixel_x), .pixel_y(pixel_y),
        .mask_valid(f_valid), .is_red(f_r), .is_green(f_g), .is_blue(f_b),
        .mask_x(f_x), .mask_y(f_y)
    );
    assign dbg_mask = {f_r, f_g, f_b};
    wire [2:0] hit = {f_b, f_g, f_r} & {3{f_valid}};     // bit k = colour k

    // ---------------- per-colour accumulators ------------------------------------------------
    logic [18:0] n_a  [3];
    logic [27:0] sx_a [3], sy_a [3];
    logic [9:0]  x0_a [3], x1_a [3], y0_a [3], y1_a [3];
    logic [2:0]  any_a;

    logic [18:0] n_n  [3];
    logic [27:0] sx_n [3], sy_n [3];
    logic [9:0]  x0_n [3], x1_n [3], y0_n [3], y1_n [3];
    logic [2:0]  any_n;

    always_comb begin
        for (int k = 0; k < 3; k++) begin
            if (frame_start) begin
                n_n[k]  = hit[k] ? 19'd1 : 19'd0;
                sx_n[k] = hit[k] ? 28'(f_x) : 28'd0;
                sy_n[k] = hit[k] ? 28'(f_y) : 28'd0;
                any_n[k] = hit[k];
                x0_n[k] = f_x; x1_n[k] = f_x; y0_n[k] = f_y; y1_n[k] = f_y;
            end else begin
                n_n[k]  = n_a[k]  + (hit[k] ? 19'd1 : 19'd0);
                sx_n[k] = sx_a[k] + (hit[k] ? 28'(f_x) : 28'd0);
                sy_n[k] = sy_a[k] + (hit[k] ? 28'(f_y) : 28'd0);
                any_n[k] = any_a[k] | hit[k];
                x0_n[k] = x0_a[k]; x1_n[k] = x1_a[k]; y0_n[k] = y0_a[k]; y1_n[k] = y1_a[k];
                if (hit[k]) begin
                    if (!any_a[k] || f_x < x0_a[k]) x0_n[k] = f_x;
                    if (!any_a[k] || f_x > x1_a[k]) x1_n[k] = f_x;
                    if (!any_a[k] || f_y < y0_a[k]) y0_n[k] = f_y;
                    if (!any_a[k] || f_y > y1_a[k]) y1_n[k] = f_y;
                end
            end
        end
    end

    // ---------------- the frame being divided -------------------------------------------------
    logic        busy;
    logic        s_good;
    logic [18:0] s_n  [3];
    logic [27:0] s_sx [3], s_sy [3];
    logic [9:0]  s_x0 [3], s_x1 [3], s_y0 [3], s_y1 [3];
    logic [2:0]  job;                        // 0..5: x then y of colour job/2
    logic        div_start, div_busy, div_done;
    logic [31:0] div_num, div_quo;
    logic [18:0] div_den;
    logic [18:0] div_rem;
    logic [13:0] q_x [3], q_y [3];
    logic        waiting;                   // a division is running

    vo_divider #(.NW(32), .DW(19)) u_div (
        .clk(clk), .rst_n(rst_n), .start(div_start), .num(div_num), .den(div_den),
        .busy(div_busy), .done(div_done), .quo(div_quo), .rem(div_rem)
    );

    wire [1:0] jc = job[2:1];
    always_comb begin
        div_num = job[0] ? {s_sy[jc], 4'b0} : {s_sx[jc], 4'b0};
        div_den = s_n[jc];
    end

    // gates of a snapshot, no division
    function automatic logic [4:0] gates(input logic [18:0] n, input logic [9:0] x0, input logic [9:0] x1,
                                         input logic [9:0] y0, input logic [9:0] y1);
        logic [10:0] bw, bh;
        logic [4:0]  f;
        bw = (n != 0) ? 11'(x1) - 11'(x0) + 11'd1 : 11'd0;
        bh = (n != 0) ? 11'(y1) - 11'(y0) + 11'd1 : 11'd0;
        f = '0;
        f[0] = (n < 19'(MIN_PIXELS)) || (n == 0);
        f[1] = (n > 19'(MAX_PIXELS));
        f[2] = (14'(bw) > 14'(MAX_ASPECT) * 14'(bh)) || (14'(bh) > 14'(MAX_ASPECT) * 14'(bw));
        f[3] = ({n, 8'b0} < 27'(MIN_FILL) * 27'(bw) * 27'(bh));
        f[4] = (n != 0) && ((x0 <= 10'(EDGE_X)) || (y0 == 10'd0) || (x1 >= 10'(W - 1 - EDGE_X)) || (y1 >= 10'(H - 1)));
        return f;
    endfunction

    logic [4:0] gf [3];
    always_comb
        for (int k = 0; k < 3; k++) gf[k] = gates(s_n[k], s_x0[k], s_x1[k], s_y0[k], s_y1[k]);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int k = 0; k < 3; k++) begin
                n_a[k] <= '0; sx_a[k] <= '0; sy_a[k] <= '0;
                x0_a[k] <= '0; x1_a[k] <= '0; y0_a[k] <= '0; y1_a[k] <= '0;
                s_n[k] <= '0; s_sx[k] <= '0; s_sy[k] <= '0;
                s_x0[k] <= '0; s_x1[k] <= '0; s_y0[k] <= '0; s_y1[k] <= '0;
                q_x[k] <= '0; q_y[k] <= '0;
            end
            any_a     <= '0;
            busy      <= 1'b0;
            s_good    <= 1'b0;
            job       <= '0;
            div_start <= 1'b0;
            waiting   <= 1'b0;
            done      <= 1'b0;
            dropped   <= 1'b0;
            res_good  <= 1'b0;
            valid     <= '0;
            x_q4      <= '0;
            y_q4      <= '0;
            count     <= '0;
            flags     <= '0;
            bbox      <= '0;
        end else begin
            done      <= 1'b0;
            dropped   <= 1'b0;
            div_start <= 1'b0;

            // accumulate; frame_end snapshots and empties
            for (int k = 0; k < 3; k++) begin
                n_a[k] <= n_n[k]; sx_a[k] <= sx_n[k]; sy_a[k] <= sy_n[k];
                x0_a[k] <= x0_n[k]; x1_a[k] <= x1_n[k]; y0_a[k] <= y0_n[k]; y1_a[k] <= y1_n[k];
            end
            any_a <= any_n;
            if (frame_end) begin
                for (int k = 0; k < 3; k++) begin
                    n_a[k] <= '0; sx_a[k] <= '0; sy_a[k] <= '0;
                end
                any_a <= '0;
                if (busy) begin
                    dropped <= 1'b1;
                end else begin
                    busy   <= 1'b1;
                    s_good <= frame_good;
                    for (int k = 0; k < 3; k++) begin
                        s_n[k] <= n_n[k]; s_sx[k] <= sx_n[k]; s_sy[k] <= sy_n[k];
                        s_x0[k] <= any_n[k] ? x0_n[k] : 10'd0; s_x1[k] <= any_n[k] ? x1_n[k] : 10'd0;
                        s_y0[k] <= any_n[k] ? y0_n[k] : 10'd0; s_y1[k] <= any_n[k] ? y1_n[k] : 10'd0;
                    end
                    job     <= '0;
                    waiting <= 1'b0;
                end
            end

            // divide: six jobs, a colour with no pixels is skipped (its centroid is 0)
            if (busy) begin
                if (!waiting) begin
                    if (job == 3'd6) begin
                        busy     <= 1'b0;
                        done     <= 1'b1;
                        res_good <= s_good;
                        for (int k = 0; k < 3; k++) begin
                            flags[k*5 +: 5]  <= gf[k];
                            valid[k]         <= s_good && (gf[k] == 5'd0);
                            x_q4[k*14 +: 14] <= q_x[k];
                            y_q4[k*14 +: 14] <= q_y[k];
                            count[k*19 +: 19] <= s_n[k];
                            bbox[k*40 +: 40] <= {s_x0[k], s_x1[k], s_y0[k], s_y1[k]};
                        end
                    end else if (s_n[jc] == 19'd0) begin
                        if (job[0]) q_y[jc] <= '0; else q_x[jc] <= '0;
                        job <= job + 3'd1;
                    end else begin
                        div_start <= 1'b1;
                        waiting   <= 1'b1;
                    end
                end else if (div_done) begin
                    if (job[0]) q_y[jc] <= div_quo[13:0]; else q_x[jc] <= div_quo[13:0];
                    job     <= job + 3'd1;
                    waiting <= 1'b0;
                end
            end
        end
    end
endmodule

`default_nettype wire
